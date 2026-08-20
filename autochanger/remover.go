package autochanger

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"sync"
	"time"

	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/utils"
)

var RemoverModel = resource.NewModel("viam", "mirka", "autochanger-remover")

// Festo Simplified Motion Series process data (app note 100290 tab. 7/8).
const (
	smsMoveIn      uint16 = 0x0001 // PD-out bit 0
	smsMoveOut     uint16 = 0x0002 // PD-out bit 1
	smsQuitError   uint16 = 0x0004 // PD-out bit 2
	smsStateIn     uint16 = 0x0001 // PD-in bit 0
	smsStateOut    uint16 = 0x0002 // PD-in bit 1
	smsStateMove   uint16 = 0x0004 // PD-in bit 2
	smsStateDevice uint16 = 0x0008 // PD-in bit 3: device error
)

// Festo SMS ISDU indices (app note 100290 tab. 5/6).
const (
	isduSpeedIn    uint16 = 0x0100 // UInt8, 1..10 = 10..100%
	isduSpeedOut   uint16 = 0x0101 // UInt8, 1..10
	isduForce      uint16 = 0x0102 // UInt8, 1..10
	isduExecHome   uint16 = 0x0104 // write 0x01 = execute reference run
	isduEndPosOut  uint16 = 0x0106 // Float32 big-endian, mm
	isduCurrentPos uint16 = 0x0120 // Int32 big-endian, 0.01mm units
	isduSysCommand uint16 = 0x0002 // IO-Link SystemCommand
)

const sysCmdStopMotion byte = 0xCA // Festo app note 100290 tab. 6 (device-specific SystemCommand: stop motion)

// al1342 is the master surface the remover needs; *Master implements it.
// The seam exists for testability (stub in unit tests) and so this file
// compiles independently of master.go during parallel development.
type al1342 interface {
	ReadPortIn(port int) (uint16, error)
	ReadPortDiag(port int) (uint16, error)
	WritePortOut(port int, word uint16) error
	SetManifoldBits(port int, mask, value uint16) error
	ISDURead(port int, index uint16, sub uint8, maxLen int) ([]byte, error)
	ISDUWrite(port int, index uint16, sub uint8, data []byte) error
}

type RemoverConfig struct {
	ModbusAddress string  `json:"modbus_address"`
	RemoverPort   int     `json:"remover_port,omitempty"`
	ManifoldPort  int     `json:"manifold_port,omitempty"`
	NozzleValve   int     `json:"nozzle_valve,omitempty"`
	Position1MM   float64 `json:"position1_mm,omitempty"`
	Position3MM   float64 `json:"position3_mm,omitempty"`
	SpeedPct      int     `json:"speed_pct,omitempty"`
	ForcePct      int     `json:"force_pct,omitempty"`
	MoveTimeoutMs int     `json:"move_timeout_ms,omitempty"`
	BlowSeconds   float64 `json:"blow_seconds,omitempty"`
}

func (c *RemoverConfig) Validate(path string) ([]string, []string, error) {
	if c.ModbusAddress == "" {
		return nil, nil, utils.NewConfigValidationFieldRequiredError(path, "modbus_address")
	}
	if c.RemoverPort < 0 || c.RemoverPort > 8 {
		return nil, nil, fmt.Errorf("%s: remover_port must be 1-8", path)
	}
	if c.SpeedPct != 0 && (c.SpeedPct < 10 || c.SpeedPct > 100) {
		return nil, nil, fmt.Errorf("%s: speed_pct must be 10-100", path)
	}
	if c.ForcePct != 0 && (c.ForcePct < 10 || c.ForcePct > 100) {
		return nil, nil, fmt.Errorf("%s: force_pct must be 10-100", path)
	}
	if c.Position1MM < 0 || c.Position3MM < 0 || c.Position3MM > 25 {
		return nil, nil, fmt.Errorf("%s: knife positions must be within 0-25mm", path)
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(generic.API, RemoverModel, resource.Registration[resource.Resource, *RemoverConfig]{
		Constructor: newRemover,
	})
}

type remover struct {
	resource.AlwaysRebuild

	name          resource.Name
	master        al1342
	releaseMaster func()
	removerPort   int
	manifoldPort  int
	nozzleValve   int
	position1MM   float64
	position3MM   float64
	moveTimeout   time.Duration
	blowSeconds   float64
	logger        logging.Logger

	mu       sync.Mutex
	homed    bool
	position int // last commanded knife position, 0 = unknown

	betweenStepsHook func() // test-only: called between release_disc steps; nil in production
}

func newRemover(
	ctx context.Context,
	deps resource.Dependencies,
	config resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*RemoverConfig](config)
	if err != nil {
		return nil, err
	}

	r := &remover{
		name:         config.ResourceName(),
		removerPort:  1,
		manifoldPort: 2,
		position1MM:  1.5,
		position3MM:  25.0,
		moveTimeout:  5 * time.Second,
		blowSeconds:  0.5,
		logger:       logger,
	}
	if conf.RemoverPort != 0 {
		r.removerPort = conf.RemoverPort
	}
	if conf.ManifoldPort != 0 {
		r.manifoldPort = conf.ManifoldPort
	}
	r.nozzleValve = conf.NozzleValve
	if conf.Position1MM != 0 {
		r.position1MM = conf.Position1MM
	}
	if conf.Position3MM != 0 {
		r.position3MM = conf.Position3MM
	}
	if conf.MoveTimeoutMs != 0 {
		r.moveTimeout = time.Duration(conf.MoveTimeoutMs) * time.Millisecond
	}
	if conf.BlowSeconds != 0 {
		r.blowSeconds = conf.BlowSeconds
	}

	master, release, err := SharedMaster(conf.ModbusAddress, 2*time.Second)
	if err != nil {
		return nil, err
	}
	r.master = master
	r.releaseMaster = release

	speed := byte(5) // 50%
	if conf.SpeedPct != 0 {
		speed = byte(conf.SpeedPct / 10)
	}
	force := byte(5)
	if conf.ForcePct != 0 {
		force = byte(conf.ForcePct / 10)
	}
	for _, p := range []struct {
		idx uint16
		val byte
	}{{isduSpeedIn, speed}, {isduSpeedOut, speed}, {isduForce, force}} {
		if err := master.ISDUWrite(r.removerPort, p.idx, 0, []byte{p.val}); err != nil {
			release()
			return nil, fmt.Errorf("writing drive parameter 0x%04X: %w", p.idx, err)
		}
	}
	return r, nil
}

func (r *remover) Name() resource.Name { return r.name }

// waitForState polls PD-in until want is set and State Move is clear.
func (r *remover) waitForState(ctx context.Context, want uint16) error {
	deadline := time.Now().Add(r.moveTimeout)
	for {
		pd, err := r.master.ReadPortIn(r.removerPort)
		if err != nil {
			return err
		}
		if pd&smsStateDevice != 0 {
			return errors.New("knife drive reports device error (use quit_error, then re-home)")
		}
		if pd&want != 0 && pd&smsStateMove == 0 {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("knife move timed out after %s (PD-in %#04x)", r.moveTimeout, pd)
		}
		if !utils.SelectContextOrWait(ctx, 20*time.Millisecond) {
			return ctx.Err()
		}
	}
}

func float32BE(v float64) []byte {
	b := make([]byte, 4)
	binary.BigEndian.PutUint32(b, math.Float32bits(float32(v)))
	return b
}

// setPosition moves the knife to Mirka position 1, 2 or 3. mmOverride
// (>0) replaces the configured extension for positions 1 and 3.
func (r *remover) setPosition(ctx context.Context, pos int, mmOverride float64) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.setPositionLocked(ctx, pos, mmOverride)
}

// setPositionLocked is setPosition's body; caller must hold r.mu. Split out
// so release_disc can run its whole extend/blow/retract sequence under one
// lock acquisition instead of three, closing the window where a concurrent
// DoCommand could move the knife mid-sequence.
func (r *remover) setPositionLocked(ctx context.Context, pos int, mmOverride float64) error {
	switch pos {
	case 2:
		if !r.homed {
			return errors.New("position 2 requires homing first (run the home command; remove the sliding plate before homing)")
		}
		if err := r.master.WritePortOut(r.removerPort, smsMoveIn); err != nil {
			return err
		}
		if err := r.waitForState(ctx, smsStateIn); err != nil {
			return err
		}
	case 1, 3:
		mm := r.position1MM
		if pos == 3 {
			mm = r.position3MM
		}
		if mmOverride > 0 {
			mm = mmOverride
		}
		if err := r.master.ISDUWrite(r.removerPort, isduEndPosOut, 0, float32BE(mm)); err != nil {
			return fmt.Errorf("set end position %.2fmm: %w", mm, err)
		}
		if err := r.master.WritePortOut(r.removerPort, smsMoveOut); err != nil {
			return err
		}
		if err := r.waitForState(ctx, smsStateOut); err != nil {
			return err
		}
	default:
		return fmt.Errorf("knife position must be 1, 2 or 3, got %d", pos)
	}
	r.position = pos
	return nil
}

// releaseDisc runs manual step 8 (extend fully to shed the disc, blast air
// behind the knife, then return to the working gap) under a single r.mu
// acquisition, so a concurrent DoCommand (e.g. set_position) can't move the
// knife partway through the sequence.
func (r *remover) releaseDisc(ctx context.Context) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if err := r.setPositionLocked(ctx, 3, 0); err != nil {
		return err
	}
	if r.betweenStepsHook != nil {
		r.betweenStepsHook()
	}
	if err := r.blowLocked(ctx, r.blowSeconds); err != nil {
		return err
	}
	if r.betweenStepsHook != nil {
		r.betweenStepsHook()
	}
	return r.setPositionLocked(ctx, 1, 0)
}

func (r *remover) home(ctx context.Context) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if err := r.master.ISDUWrite(r.removerPort, isduExecHome, 0, []byte{0x01}); err != nil {
		return fmt.Errorf("execute reference run: %w", err)
	}
	// Homing ends with the slide at the reference end position (State In).
	if err := r.waitForState(ctx, smsStateIn); err != nil {
		return err
	}
	r.homed = true
	r.position = 2
	return nil
}

func (r *remover) blow(ctx context.Context, seconds float64) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.blowLocked(ctx, seconds)
}

// blowLocked is blow's body; caller must hold r.mu (see setPositionLocked).
func (r *remover) blowLocked(ctx context.Context, seconds float64) error {
	mask := uint16(1) << r.nozzleValve
	if err := r.master.SetManifoldBits(r.manifoldPort, mask, mask); err != nil {
		return fmt.Errorf("open nozzle valve: %w", err)
	}
	// Close the valve even on cancellation: never leave air blasting.
	utils.SelectContextOrWait(ctx, time.Duration(seconds*float64(time.Second)))
	if err := r.master.SetManifoldBits(r.manifoldPort, mask, 0); err != nil {
		return fmt.Errorf("close nozzle valve: %w", err)
	}
	return ctx.Err()
}

// Status satisfies resource.Resource; it returns the same payload as the
// "status" DoCommand.
func (r *remover) Status(ctx context.Context) (map[string]interface{}, error) {
	pd, err := r.master.ReadPortIn(r.removerPort)
	if err != nil {
		return nil, err
	}
	posBytes, err := r.master.ISDURead(r.removerPort, isduCurrentPos, 0, 4)
	positionMM := math.NaN()
	if err == nil && len(posBytes) == 4 {
		positionMM = float64(int32(binary.BigEndian.Uint32(posBytes))) / 100.0
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	return map[string]interface{}{
		"position":     r.position,
		"homed":        r.homed,
		"state_in":     pd&smsStateIn != 0,
		"state_out":    pd&smsStateOut != 0,
		"moving":       pd&smsStateMove != 0,
		"device_error": pd&smsStateDevice != 0,
		"position_mm":  positionMM,
	}, nil
}

func (r *remover) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, ok := cmd["command"].(string)
	if !ok {
		return nil, errors.New("missing 'command' field")
	}
	switch command {
	case "home":
		if err := r.home(ctx); err != nil {
			return nil, err
		}
		return map[string]interface{}{"homed": true}, nil

	case "set_position":
		pos, ok := cmd["position"].(float64)
		if !ok {
			return nil, errors.New("set_position requires numeric 'position' (1, 2 or 3)")
		}
		mm, _ := cmd["mm"].(float64)
		if err := r.setPosition(ctx, int(pos), mm); err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": int(pos)}, nil

	case "blow":
		seconds := r.blowSeconds
		if v, ok := cmd["seconds"].(float64); ok {
			seconds = v
		}
		if err := r.blow(ctx, seconds); err != nil {
			return nil, err
		}
		return map[string]interface{}{"blown": true}, nil

	case "release_disc":
		if err := r.releaseDisc(ctx); err != nil {
			return nil, err
		}
		return map[string]interface{}{"released": true}, nil

	case "quit_error":
		// Writes go straight to the master, outside r.mu: a stuck move holds
		// the lock for the full moveTimeout while it waits, and quit_error
		// must be able to reach the drive during that wait to clear the
		// fault, not queue up behind it.
		if err := r.master.WritePortOut(r.removerPort, smsQuitError); err != nil {
			return nil, err
		}
		if err := r.master.WritePortOut(r.removerPort, 0); err != nil {
			return nil, err
		}
		r.mu.Lock()
		r.homed = false // after a fault, require re-homing before position 2
		r.position = 0
		r.mu.Unlock()
		return map[string]interface{}{"cleared": true}, nil

	case "status":
		return r.Status(ctx)

	default:
		return nil, fmt.Errorf(
			"unknown command %q (supported: home, set_position, blow, release_disc, quit_error, status)", command)
	}
}

func (r *remover) Close(ctx context.Context) error {
	// Best-effort stop so a reconfigure never leaves the knife mid-travel.
	if err := r.master.ISDUWrite(r.removerPort, isduSysCommand, 0, []byte{sysCmdStopMotion}); err != nil {
		r.logger.Warnf("stop motion on close failed: %v", err)
	}
	if r.releaseMaster != nil {
		r.releaseMaster()
	}
	return nil
}
