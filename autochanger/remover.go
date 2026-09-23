package autochanger

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/gantry"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

var Model = resource.NewModel("viam", "mirka", "autochanger-remover")

const (
	defaultPort        = 1
	defaultMoveTimeout = 10 * time.Second
)

// Config is the machine configuration for one remover. There is no geometry
// configuration: the envelope lives in model.json, and the machine's frame must
// be configured without its optional geometry field, which would replace it.
type Config struct {
	Address   string `json:"address"`
	Port      int    `json:"port,omitempty"`
	TimeoutMs uint   `json:"timeout_ms,omitempty"`
}

func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.Address == "" {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "address")
	}
	if c.Port != 0 && (c.Port < 1 || c.Port > 8) {
		return nil, nil, fmt.Errorf("%s: port must be between 1 and 8, got %d", path, c.Port)
	}
	return nil, nil, nil
}

// Registered as a gantry, not a generic component: a LinkInFrame carries one
// geometry, and this needs three. See KinematicModel.
func init() {
	resource.RegisterComponent(gantry.API, Model, resource.Registration[gantry.Gantry, *Config]{
		Constructor: newRemover,
	})
}

func newRemover(
	ctx context.Context,
	deps resource.Dependencies,
	config resource.Config,
	logger logging.Logger,
) (gantry.Gantry, error) {
	conf, err := resource.NativeConfig[*Config](config)
	if err != nil {
		return nil, err
	}

	port := conf.Port
	if port == 0 {
		port = defaultPort
	}
	moveTimeout := defaultMoveTimeout
	if conf.TimeoutMs != 0 {
		moveTimeout = time.Duration(conf.TimeoutMs) * time.Millisecond
	}

	r := newRemoverFor(config.ResourceName(), NewClient(conf.Address, port, logger),
		moveTimeout, logger)
	r.startPolling()
	logger.CInfow(ctx, "remover built",
		"address", conf.Address, "io_link_port", port,
		"move_timeout", moveTimeout.String(),
		"seeded_offset_mm", KnifeTravelMM)
	return r, nil
}

// newRemoverFor is the only construction path, so the seeded offset cannot be
// bypassed.
func newRemoverFor(
	name resource.Name,
	client *Client,
	moveTimeout time.Duration,
	logger logging.Logger,
) *remover {
	r := &remover{
		name:        name,
		client:      client,
		moveTimeout: moveTimeout,
		logger:      logger,
		startedAt:   time.Now(),
	}
	// The frame system queries this component as soon as it is built, before any
	// read. Release is the fail-safe guess: claiming the knife is retracted when
	// it is out leaves 25mm of blade unmodelled.
	r.snap.Store(&snapshot{offsetMM: KnifeTravelMM})
	return r
}

// pollInterval leaves the position exact whenever the knife is parked, which
// is whenever anything plans against it.
const pollInterval = 100 * time.Millisecond

// positionMaxAge bounds how long the position may go unverified while the
// drive sits still, so a knife moved outside this module is still noticed.
const positionMaxAge = 5 * time.Second

// arrivalToleranceCounts: the drive lands within one count, measured at 71,
// 1088 and 1500.
const arrivalToleranceCounts = 10

// snapshot is the drive at one instant. Every read method returns a field of
// it, so no read method performs I/O.
type snapshot struct {
	at time.Time
	// posAt dates offsetMM alone, which refreshes far less often than the rest.
	posAt    time.Time
	offsetMM float64
	state    State
	// A failed poll keeps the previous offset rather than inventing one.
	ok  bool
	err error
}

type remover struct {
	resource.AlwaysRebuild

	name        resource.Name
	client      *Client
	moveTimeout time.Duration
	logger      logging.Logger

	// Acquired with TryLock, so a concurrent move fails rather than queueing.
	mu sync.Mutex

	// One goroutine writes it, everyone else loads it. The frame system resolves
	// this component once per planning node -- 27 concurrent callers measured for
	// one question -- and serving each from the device reset the connection.
	snap atomic.Pointer[snapshot]

	failures int
	everRead bool
	statsMu  sync.Mutex

	stopPolling context.CancelFunc
	pollDone    chan struct{}

	startedAt time.Time
}

// startPolling's context is created here rather than derived from the
// constructor's, which is cancelled once the resource is built.
func (r *remover) startPolling() {
	ctx, cancel := context.WithCancel(context.Background())
	r.stopPolling = cancel
	r.pollDone = make(chan struct{})
	go func() {
		defer close(r.pollDone)
		t := time.NewTicker(pollInterval)
		defer t.Stop()
		r.refresh(ctx)
		for {
			select {
			case <-ctx.Done():
				return
			case <-t.C:
				r.refresh(ctx)
			}
		}
	}()
}

// refresh is the only place in the component that performs I/O for state.
//
// Process data every cycle (~25ms); the position register only when a move
// settles, because an acyclic transfer costs ~860ms. Nothing consumes the
// knife's position mid-stroke -- the arm is stationary near the changer.
func (r *remover) refresh(ctx context.Context) {
	prev := r.snap.Load()

	pd, err := r.client.ReadPD(ctx)
	if err != nil {
		r.noteRefreshFailure(ctx, prev, err)
		return
	}
	state := DecodeState(pd)

	// A change of taught point cannot be the only trigger: a move from one
	// intermediate to another never changes it.
	offset, posAt := prev.offsetMM, prev.posAt
	settled := !prev.ok || prev.state.Moving || state.Position != prev.state.Position
	if !state.Moving && (settled || time.Since(prev.posAt) > positionMaxAge) {
		counts, err := r.client.ReadPositionCounts(ctx)
		if err != nil {
			r.noteRefreshFailure(ctx, prev, err)
			return
		}
		offset, posAt = clampToTravel(OffsetForCounts(counts)), time.Now()
	}

	r.snap.Store(&snapshot{at: time.Now(), posAt: posAt, offsetMM: offset, state: state, ok: true})
	if n, recovered := r.noteSuccess(); recovered {
		r.logger.CInfow(ctx, "remover reads recovered",
			"after_consecutive_failures", n,
			"since_module_start", time.Since(r.startedAt).Round(time.Millisecond).String())
	}
}

func (r *remover) noteRefreshFailure(ctx context.Context, prev *snapshot, err error) {
	// A cancelled context is the poller shutting down, not a device fault.
	if ctx.Err() != nil {
		return
	}
	// Keep the last known offset: this module is the only thing that moves the
	// knife, so a drive that cannot be reached also cannot have moved.
	n, neverRead := r.noteFailure()
	r.snap.Store(&snapshot{at: prev.at, offsetMM: prev.offsetMM, state: prev.state, err: err})
	msg := "remover poll failed; keeping last known knife offset"
	fields := []any{
		"offset_mm", prev.offsetMM,
		"consecutive_failures", n,
		"never_read_since_start", neverRead,
		"since_module_start", time.Since(r.startedAt).Round(time.Millisecond).String(),
		"err", err,
	}
	// Loud on the first failure and periodically after, so a drive that stays
	// down does not bury the rest of the log.
	if n == 1 || n%100 == 0 {
		r.logger.CErrorw(ctx, msg, fields...)
	} else {
		r.logger.CDebugw(ctx, msg, fields...)
	}
}

func (r *remover) Name() resource.Name {
	return r.name
}

func (r *remover) Status(ctx context.Context) (map[string]interface{}, error) {
	snap := r.snap.Load()
	st := snap.state
	if snap.err != nil {
		return nil, snap.err
	}
	return map[string]interface{}{
		"position": st.Position.String(),
		"moving":   st.Moving,
		"ready":    st.Ready,
		"raw":      int(st.Raw),
	}, nil
}

// setPosition waits for arrival. offsetMM applies only to
// PositionIntermediate.
//
// A second call mid-move fails rather than queueing, departing from the RDK's
// SingleOperationManager convention: the only caller is the sequencer, so
// concurrent moves are a defect in it, and cancelling would leave a disc
// ungripped.
func (r *remover) setPosition(ctx context.Context, want Position, offsetMM float64) error {
	if !r.mu.TryLock() {
		return fmt.Errorf("remover: a move is already in progress")
	}
	defer r.mu.Unlock()

	snap := r.snap.Load()
	if snap.err != nil {
		return fmt.Errorf("remover: cannot reach the drive: %w", snap.err)
	}
	if !snap.state.Ready {
		return fmt.Errorf("remover: drive is not ready (pd=0x%04X); acknowledge faults with quit_error",
			snap.state.Raw)
	}

	var cmd byte
	// Set only for intermediate moves, where the process data cannot distinguish
	// arrival. The end stops need no target, and checking counts there would make
	// the module sensitive to end-stop drift.
	targetCounts := -1
	switch want {
	case PositionIn:
		cmd = CmdMoveIn
	case PositionOut:
		cmd = CmdMoveOut
	case PositionIntermediate:
		cmd = CmdMoveIntermediate
		// Never cached: homing rewrites this parameter to the full stroke, and runs
		// by hand outside the module, so a stale value sends the knife to full
		// extension on a request for a 1mm gap.
		targetCounts = int(math.Round(offsetMM * PosImpPerMM))
		if err := r.client.ISDUWriteFloat32(ctx, IsduPosImp, 0, float32(offsetMM*PosImpPerMM)); err != nil {
			return fmt.Errorf("setting intermediate position: %w", err)
		}
	default:
		return fmt.Errorf("remover: cannot move to position %v", want)
	}

	if err := r.client.SystemCommand(ctx, cmd); err != nil {
		return fmt.Errorf("commanding move to %s: %w", want, err)
	}
	return r.waitForPosition(ctx, want, targetCounts)
}

// waitForPosition blocks until the drive is stopped at the commanded target.
//
// Process data alone cannot signal arrival for an intermediate move: a knife
// already at one intermediate reports the same bit the instant a move to
// another is commanded. The deadline is the only thing that catches a jam,
// which answers every request perfectly while never reaching the target.
func (r *remover) waitForPosition(ctx context.Context, want Position, targetCounts int) error {
	deadline := time.Now().Add(r.moveTimeout)
	for {
		pd, err := r.client.ReadPD(ctx)
		if err != nil {
			return err
		}
		state := DecodeState(pd)

		if !state.Moving && state.Position == want {
			counts, err := r.client.ReadPositionCounts(ctx)
			if err != nil {
				return err
			}
			off := counts - int32(targetCounts)
			if targetCounts < 0 || (off <= arrivalToleranceCounts && off >= -arrivalToleranceCounts) {
				// Publish what was just measured rather than waiting a poll cycle.
				r.snap.Store(&snapshot{
					at: time.Now(), posAt: time.Now(),
					offsetMM: clampToTravel(OffsetForCounts(counts)), state: state, ok: true,
				})
				return nil
			}
			if time.Now().After(deadline) {
				return fmt.Errorf("remover: timed out after %s; drive reports %s and is stopped at %d counts, "+
					"but %d was commanded", r.moveTimeout, want, counts, targetCounts)
			}
		} else if time.Now().After(deadline) {
			return fmt.Errorf("remover: timed out after %s waiting for position %s (pd=0x%04X)",
				r.moveTimeout, want, state.Raw)
		}

		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(pollInterval):
		}
	}
}

func (r *remover) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, ok := cmd["command"].(string)
	if !ok {
		return nil, fmt.Errorf("missing 'command' field")
	}

	switch command {
	case "diagnostics":
		pd, pdErr := r.client.ReadPD(ctx)
		counts, posErr := r.client.ReadPositionCounts(ctx)
		out := map[string]interface{}{
			"transport":         r.client.Stats(),
			"module_age":        time.Since(r.startedAt).Round(time.Millisecond).String(),
			"last_offset_mm":    r.lastOffset(),
			"ever_read":         r.hasEverRead(),
			"live_read_ok":      pdErr == nil && posErr == nil,
			"knife_travel_mm":   KnifeTravelMM,
			"knife_travel_cnts": KnifeTravelCounts,
		}
		if pdErr == nil {
			st := DecodeState(pd)
			out["pd_raw"] = fmt.Sprintf("0x%04X", st.Raw)
			out["position"] = st.Position.String()
			out["moving"] = st.Moving
			out["ready"] = st.Ready
		} else {
			out["pd_error"] = pdErr.Error()
		}
		if posErr == nil {
			out["position_counts"] = counts
			out["position_mm"] = OffsetForCounts(counts)
		} else {
			out["position_error"] = posErr.Error()
		}
		return out, nil

	case "quit_error":
		// The acknowledgement is edge triggered: assert the bit, then clear it so
		// a later fault can be acknowledged too.
		if err := r.client.WritePD(ctx, pdOutQuitError); err != nil {
			return nil, fmt.Errorf("asserting quit error: %w", err)
		}
		if err := r.client.WritePD(ctx, 0); err != nil {
			return nil, fmt.Errorf("clearing quit error: %w", err)
		}
		return map[string]interface{}{"acknowledged": true}, nil

	default:
		return nil, fmt.Errorf("unknown command %q (supported: quit_error, diagnostics); "+
			"moving and stopping are gantry methods, state is served by GetStatus, and homing is a "+
			"commissioning procedure documented in the README", command)
	}
}

func (r *remover) Close(ctx context.Context) error {
	if r.stopPolling == nil {
		return nil
	}
	r.stopPolling()
	select {
	case <-r.pollDone:
	case <-ctx.Done():
		return ctx.Err()
	}
	return nil
}

// --- gantry.Gantry ---------------------------------------------------------
//
// Units are millimeters, matching the RDK's own gantries.

func (r *remover) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return KinematicModel()
}

// CurrentInputs cannot fail. The frame system resolves every frame together,
// so an error here fails pose queries for unrelated resources -- an unreachable
// AL1342 was observed making the arm unlocatable. Serving the last known offset
// is sound because this module is the only thing that moves the knife.
func (r *remover) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{r.snap.Load().offsetMM}, nil
}

func (r *remover) lastOffset() float64 {
	return r.snap.Load().offsetMM
}

func (r *remover) hasEverRead() bool {
	r.statsMu.Lock()
	defer r.statsMu.Unlock()
	return r.everRead
}

func (r *remover) noteFailure() (failures int, neverRead bool) {
	r.statsMu.Lock()
	defer r.statsMu.Unlock()
	r.failures++
	return r.failures, !r.everRead
}

func (r *remover) noteSuccess() (failures int, recovered bool) {
	r.statsMu.Lock()
	defer r.statsMu.Unlock()
	r.everRead = true
	failures, r.failures = r.failures, 0
	return failures, failures > 0
}

func (r *remover) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	for _, step := range inputSteps {
		if len(step) != 1 {
			return fmt.Errorf("remover: expected 1 input, got %d", len(step))
		}
		if err := r.moveToOffset(ctx, step[0]); err != nil {
			return err
		}
	}
	return nil
}

// moveToOffset drives the knife to any point on its travel. The named
// positions are the drive's command vocabulary, not its reachable set.
func (r *remover) moveToOffset(ctx context.Context, offsetMM float64) error {
	pos, err := commandForOffset(offsetMM)
	if err != nil {
		return err
	}
	return r.setPosition(ctx, pos, offsetMM)
}

func (r *remover) Position(ctx context.Context, extra map[string]interface{}) ([]float64, error) {
	inputs, err := r.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	return []float64{inputs[0]}, nil
}

// MoveToPosition ignores speeds: the drive's is an ISDU parameter set at
// commissioning.
func (r *remover) MoveToPosition(
	ctx context.Context,
	positionsMm, speedsMmPerSec []float64,
	extra map[string]interface{},
) error {
	if len(positionsMm) != 1 {
		return fmt.Errorf("remover: expected 1 position, got %d", len(positionsMm))
	}
	return r.moveToOffset(ctx, positionsMm[0])
}

func (r *remover) Lengths(ctx context.Context, extra map[string]interface{}) ([]float64, error) {
	return []float64{KnifeTravelMM}, nil
}

func (r *remover) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	model, err := KinematicModel()
	if err != nil {
		return nil, err
	}
	inputs, err := r.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	gif, err := model.Geometries(inputs)
	if err != nil {
		return nil, err
	}
	return gif.Geometries(), nil
}

func (r *remover) IsMoving(ctx context.Context) (bool, error) {
	return r.snap.Load().state.Moving, nil
}

func (r *remover) Stop(ctx context.Context, extra map[string]interface{}) error {
	return r.client.SystemCommand(ctx, CmdStop)
}

// Home refuses. Homing needs the sliding plate physically removed first (manual
// p.49); with it fitted the taught zero is short by the plate's thickness, and
// the symptom appears much later as discs failing to release. Viam cannot
// enforce that precondition.
func (r *remover) Home(ctx context.Context, extra map[string]interface{}) (bool, error) {
	return false, fmt.Errorf(
		"remover: homing is a commissioning procedure, not a command -- it requires the " +
			"sliding plate to be physically removed first. See the README")
}
