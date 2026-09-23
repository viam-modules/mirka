package autochanger

import (
	"context"
	"math"
	"strconv"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// newTestRemover builds a remover pointed at the fake, bypassing registration.
func newTestRemover(t *testing.T, f *fakeMaster) *remover {
	t.Helper()
	r := newUnpolledRemover(t, f)
	// One synchronous refresh so the first snapshot is deterministic, then the
	// real poller, so move tests exercise the same path production does.
	r.refresh(context.Background())
	r.startPolling()
	t.Cleanup(func() { _ = r.Close(context.Background()) })
	return r
}

// newUnpolledRemover builds a remover whose snapshot is still the seed, for
// tests about what this component reports before it has read the drive.
func newUnpolledRemover(t *testing.T, f *fakeMaster) *remover {
	t.Helper()
	return newRemoverFor(resource.Name{Name: "test"}, NewClient(f.addr(), 1, logging.NewTestLogger(t)),
		2*time.Second, logging.NewTestLogger(t))
}

func TestValidateRejectsMissingAddress(t *testing.T) {
	c := &Config{}
	if _, _, err := c.Validate("path"); err == nil {
		t.Fatal("expected an error for a missing address")
	}
}

func TestValidateRejectsBadPort(t *testing.T) {
	for _, c := range []*Config{
		{Address: "host", Port: 9},
	} {
		if _, _, err := c.Validate("path"); err == nil {
			t.Fatalf("expected an error for %+v", c)
		}
	}
}

func TestValidateAcceptsMinimalConfig(t *testing.T) {
	c := &Config{Address: "192.168.50.10"}
	if _, _, err := c.Validate("path"); err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
}

func TestStatusReportsPosition(t *testing.T) {
	f := newFakeMaster(t, "0018")
	r := newTestRemover(t, f)

	st, err := r.Status(context.Background())
	if err != nil {
		t.Fatalf("Status: %v", err)
	}
	if st["position"] != "intermediate" {
		t.Fatalf("position = %v, want grip", st["position"])
	}
	if st["ready"] != true {
		t.Fatalf("ready = %v, want true", st["ready"])
	}
	if st["moving"] != false {
		t.Fatalf("moving = %v, want false", st["moving"])
	}
}

func TestStatusReportsUnknown(t *testing.T) {
	f := newFakeMaster(t, "0008")
	r := newTestRemover(t, f)

	st, err := r.Status(context.Background())
	if err != nil {
		t.Fatalf("Status: %v", err)
	}
	if st["position"] != "unknown" {
		t.Fatalf("position = %v, want unknown", st["position"])
	}
}

func TestCloseDoesNotMoveTheKnife(t *testing.T) {
	f := newFakeMaster(t, "000A")
	r := newTestRemover(t, f)

	if err := r.Close(context.Background()); err != nil {
		t.Fatalf("Close: %v", err)
	}
	for _, req := range f.seen() {
		if strings.Contains(req, "iolwriteacyclic") || strings.Contains(req, "pdout") {
			t.Fatalf("Close issued a motion request: %s", req)
		}
	}
}

func TestSetPositionWritesPosImpThenCommands(t *testing.T) {
	f := newFakeMaster(t, "0018")
	f.simulateDrive()
	r := newTestRemover(t, f)

	if err := r.setPosition(context.Background(), PositionIntermediate, 0.68); err != nil {
		t.Fatalf("setPosition: %v", err)
	}

	var sawPosImp, sawCommand bool
	for _, req := range f.seen() {
		if strings.Contains(req, `"index":264`) {
			sawPosImp = true
		}
		if strings.Contains(req, `"value":"D0"`) {
			if !sawPosImp {
				t.Fatal("sent the move command before writing PosImp")
			}
			sawCommand = true
		}
	}
	if !sawPosImp || !sawCommand {
		t.Fatalf("missing requests: posImp=%v command=%v", sawPosImp, sawCommand)
	}
}

// PosImp must be rewritten on every grip move. Homing overwrites it with the
// full stroke, and homing runs by hand outside the module, so skipping a
// repeated write would eventually send the knife to full extension on a request
// for the grip position.
func TestSetPositionAlwaysWritesPosImp(t *testing.T) {
	f := newFakeMaster(t, "0018")
	f.simulateDrive()
	r := newTestRemover(t, f)

	for i := 0; i < 2; i++ {
		if err := r.setPosition(context.Background(), PositionIntermediate, 0.68); err != nil {
			t.Fatalf("setPosition: %v", err)
		}
	}

	count := 0
	for _, req := range f.seen() {
		if strings.Contains(req, `"index":264`) {
			count++
		}
	}
	if count != 2 {
		t.Fatalf("PosImp written %d times, want 2 (once per move, never cached)", count)
	}
}

func TestEndStopsDoNotWritePosImp(t *testing.T) {
	f := newFakeMaster(t, "000A") // at release
	r := newTestRemover(t, f)

	if err := r.setPosition(context.Background(), PositionOut, 0); err != nil {
		t.Fatalf("setPosition: %v", err)
	}
	for _, req := range f.seen() {
		if strings.Contains(req, `"index":264`) {
			t.Fatalf("end stop move wrote PosImp: %s", req)
		}
	}
}

func TestSetPositionRefusesWhenNotReady(t *testing.T) {
	f := newFakeMaster(t, "0000") // ready bit clear
	r := newTestRemover(t, f)

	err := r.setPosition(context.Background(), PositionIn, 0)
	if err == nil {
		t.Fatal("expected a refusal when the drive is not ready")
	}
	for _, req := range f.seen() {
		if strings.Contains(req, "iolwriteacyclic") {
			t.Fatalf("issued a command despite not being ready: %s", req)
		}
	}
}

func TestSetPositionTimesOut(t *testing.T) {
	f := newFakeMaster(t, "000C") // moving forever, never arrives
	r := newTestRemover(t, f)
	r.moveTimeout = 300 * time.Millisecond

	err := r.setPosition(context.Background(), PositionIn, 0)
	if err == nil {
		t.Fatal("expected a timeout")
	}
	if !strings.Contains(err.Error(), "timed out") {
		t.Fatalf("error should name the timeout, got %v", err)
	}
}

func TestSetPositionFailsFastWhenBusy(t *testing.T) {
	f := newFakeMaster(t, "000C")
	r := newTestRemover(t, f)
	r.moveTimeout = 2 * time.Second

	started := make(chan struct{})
	go func() {
		close(started)
		_ = r.setPosition(context.Background(), PositionIn, 0)
	}()
	<-started
	time.Sleep(100 * time.Millisecond)

	err := r.setPosition(context.Background(), PositionOut, 0)
	if err == nil {
		t.Fatal("expected a busy error from the second move")
	}
	if !strings.Contains(err.Error(), "in progress") {
		t.Fatalf("error should say a move is in progress, got %v", err)
	}
}

func TestDoCommandRequiresCommandField(t *testing.T) {
	f := newFakeMaster(t, "0009")
	r := newTestRemover(t, f)

	if _, err := r.DoCommand(context.Background(), map[string]interface{}{}); err == nil {
		t.Fatal("expected an error for a missing command field")
	}
}

func TestDoCommandQuitError(t *testing.T) {
	f := newFakeMaster(t, "0009")
	r := newTestRemover(t, f)

	if _, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "quit_error"}); err != nil {
		t.Fatalf("quit_error: %v", err)
	}
	var writes []string
	for _, req := range f.seen() {
		if strings.Contains(req, "pdout/setdata") {
			writes = append(writes, req)
		}
	}
	if len(writes) != 2 {
		t.Fatalf("expected assert then clear, got %d pdout writes", len(writes))
	}
	if !strings.Contains(writes[0], `"newvalue":"0004"`) {
		t.Fatalf("first write should assert bit 2, got %s", writes[0])
	}
	if !strings.Contains(writes[1], `"newvalue":"0000"`) {
		t.Fatalf("second write should clear, got %s", writes[1])
	}
}

func TestDoCommandUnknown(t *testing.T) {
	f := newFakeMaster(t, "0009")
	r := newTestRemover(t, f)

	_, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "home"})
	if err == nil {
		t.Fatal("expected an error for an unknown command")
	}
	for _, want := range []string{"quit_error", "diagnostics"} {
		if !strings.Contains(err.Error(), want) {
			t.Fatalf("error should list %s, got %v", want, err)
		}
	}
}

// 0xCD is Restore Factory Settings and sits one hex digit from 0xCE Reference.
// Nothing in this package may emit it, and command bytes must be selected from
// named constants rather than computed.
func TestNoFactoryResetConstant(t *testing.T) {
	for _, c := range []byte{CmdMoveIn, CmdMoveOut, CmdStop, CmdMoveIntermediate} {
		if c == 0xCD {
			t.Fatal("0xCD must never be a command constant")
		}
		if c == 0xCE || c == 0xCF {
			t.Fatal("reference commands must not be reachable from the component")
		}
	}
}

// lastPosImpWritten decodes the most recent intermediate target the module sent.
func lastPosImpWritten(f *fakeMaster) (float64, bool) {
	var out float64
	var found bool
	for _, req := range f.seen() {
		if !strings.Contains(req, `"index":264`) {
			continue
		}
		m := valueRE.FindStringSubmatch(req)
		if m == nil {
			continue
		}
		bits, err := strconv.ParseUint(m[1], 16, 32)
		if err != nil {
			continue
		}
		out, found = float64(math.Float32frombits(uint32(bits))), true
	}
	return out, found
}

func TestGantryLengthsAndPosition(t *testing.T) {
	f := newFakeMaster(t, "000A") // at release
	r := newTestRemover(t, f)
	ctx := context.Background()

	lengths, err := r.Lengths(ctx, nil)
	if err != nil {
		t.Fatalf("Lengths: %v", err)
	}
	if len(lengths) != 1 || math.Abs(lengths[0]-KnifeTravelMM) > 0.001 {
		t.Fatalf("Lengths = %v, want [%v]", lengths, KnifeTravelMM)
	}

	pos, err := r.Position(ctx, nil)
	if err != nil {
		t.Fatalf("Position: %v", err)
	}
	if len(pos) != 1 || math.Abs(pos[0]-KnifeTravelMM) > 0.001 {
		t.Fatalf("Position = %v, want [%v] at release", pos, KnifeTravelMM)
	}
}

func TestCurrentInputsTracksTheDrive(t *testing.T) {
	// Driven by the position register, not the process data bits: mid-travel is
	// a real reading rather than an invented midpoint, which is the whole point
	// of using index 0x0120.
	for _, tc := range []struct {
		counts string
		want   float64
	}{
		{"00000000", 0},
		{"00000047", 0.6824},  // the 1.1mm grip gap
		{"000005DB", 14.4079}, // mid-travel, no position bit set
		{"00000A29", KnifeTravelMM},
	} {
		f := newFakeMaster(t, "0008") // no position bit asserted
		f.counts = tc.counts
		r := newTestRemover(t, f)
		in, err := r.CurrentInputs(context.Background())
		if err != nil {
			t.Fatalf("CurrentInputs(%s): %v", tc.counts, err)
		}
		if len(in) != 1 || math.Abs(in[0]-tc.want) > 0.001 {
			t.Fatalf("CurrentInputs(%s) = %v, want [%v]", tc.counts, in, tc.want)
		}
	}
}

func TestMoveToArbitraryIntermediate(t *testing.T) {
	f := newFakeMaster(t, "0018")
	f.simulateDrive()
	r := newTestRemover(t, f)
	ctx := context.Background()

	if err := r.MoveToPosition(ctx, []float64{10}, nil, nil); err != nil {
		t.Fatalf("MoveToPosition(10): %v", err)
	}
	// The target written must mean 10mm of displacement. Handing the joint
	// offset straight through as a gap -- the two measure from different zeros
	// -- would write PosImpOffsetMM too many millimetres.
	target, ok := lastPosImpWritten(f)
	if !ok {
		t.Fatalf("no intermediate target written for 10mm: %v", f.seen())
	}
	if mm := target / PosImpPerMM; math.Abs(mm-10) > 0.02 {
		t.Fatalf("commanded 10mm, wrote %v counts = %v mm", target, mm)
	}

	// The second move is the regression: both ends are PositionIntermediate, so the
	// process data bit is already set when the command is issued and cannot
	// signal arrival. Reported position used to stick at the first value while
	// the drive really moved, and the move reported success immediately.
	if err := r.MoveToPosition(ctx, []float64{20}, nil, nil); err != nil {
		t.Fatalf("MoveToPosition(20): %v", err)
	}
	if pos, err := r.Position(ctx, nil); err != nil || math.Abs(pos[0]-20) > 0.02 {
		t.Fatalf("after moving to 20mm the component reports %v (err %v)", pos, err)
	}
}

func TestMoveToPositionRefusesOffAxis(t *testing.T) {
	f := newFakeMaster(t, "0009")
	r := newTestRemover(t, f)

	err := r.MoveToPosition(context.Background(), []float64{40}, nil, nil)
	if err == nil {
		t.Fatal("expected 40mm to be refused")
	}
	for _, req := range f.seen() {
		if strings.Contains(req, "iolwriteacyclic") {
			t.Fatalf("refused move still commanded the drive: %s", req)
		}
	}
}

func TestHomeRefuses(t *testing.T) {
	f := newFakeMaster(t, "0009")
	r := newTestRemover(t, f)

	ok, err := r.Home(context.Background(), nil)
	if err == nil {
		t.Fatal("expected Home to refuse")
	}
	if ok {
		t.Fatal("Home reported success while refusing")
	}
	if !strings.Contains(err.Error(), "README") {
		t.Fatalf("error should point at the README, got %v", err)
	}
	for _, req := range f.seen() {
		if strings.Contains(req, "iolwriteacyclic") {
			t.Fatalf("Home issued a command: %s", req)
		}
	}
}

func TestIsMoving(t *testing.T) {
	for _, tc := range []struct {
		pd   string
		want bool
	}{
		{"000C", true},
		{"0009", false},
	} {
		f := newFakeMaster(t, tc.pd)
		r := newTestRemover(t, f)
		got, err := r.IsMoving(context.Background())
		if err != nil {
			t.Fatalf("IsMoving(%s): %v", tc.pd, err)
		}
		if got != tc.want {
			t.Fatalf("IsMoving(%s) = %v, want %v", tc.pd, got, tc.want)
		}
	}
}

func TestGeometriesFollowTheKnife(t *testing.T) {
	zero := geomsX(t, "0009")
	release := geomsX(t, "000A")
	if moved := release - zero; math.Abs(moved-KnifeTravelMM) > 0.001 {
		t.Fatalf("blade moved %v mm between positions 2 and 3, want %v", moved, KnifeTravelMM)
	}
}

func geomsX(t *testing.T, pd string) float64 {
	t.Helper()
	f := newFakeMaster(t, pd)
	r := newTestRemover(t, f)
	gs, err := r.Geometries(context.Background(), nil)
	if err != nil {
		t.Fatalf("Geometries: %v", err)
	}
	for _, g := range gs {
		if strings.HasSuffix(g.Label(), "blade") {
			return g.Pose().Point().X
		}
	}
	t.Fatalf("no blade geometry in %v", gs)
	return 0
}

func TestPollerOwnsTheConnection(t *testing.T) {
	// The read methods must not touch the transport whatever their callers do:
	// the frame system resolves this component once per planning node, and
	// serving each from the device took the whole frame system down. The poller
	// reads process data every cycle and the position register only when a move
	// settles.
	f := newFakeMaster(t, "0009")
	r := newTestRemover(t, f)

	cancelled, cancel := context.WithCancel(context.Background())
	cancel()
	before := len(f.seen())
	for i := 0; i < 50; i++ {
		if _, err := r.CurrentInputs(cancelled); err != nil {
			t.Fatalf("CurrentInputs must not fail: %v", err)
		}
	}
	if grew := len(f.seen()) - before; grew > 1 {
		t.Fatalf("50 concurrent-style calls issued %d requests, want 0", grew)
	}

	// A move: the position register is read once it settles, not during.
	before = len(f.seen())
	f.setPD("000C")
	f.setCounts("00000410") // 1040 counts
	time.Sleep(4 * pollInterval)
	for _, req := range f.seen()[before:] {
		if strings.Contains(req, `"index":288`) {
			t.Fatal("read the position register while the drive was moving")
		}
	}
	f.setPD("0018")

	deadline := time.Now().Add(2 * time.Second)
	for {
		in, _ := r.CurrentInputs(context.Background())
		if math.Abs(in[0]-OffsetForCounts(1040)) <= 0.001 {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("poller never picked up the settled move; reporting %v", in[0])
		}
		time.Sleep(10 * time.Millisecond)
	}

	// Close stops it.
	if err := r.Close(context.Background()); err != nil {
		t.Fatalf("Close: %v", err)
	}
	quiet := len(f.seen())
	time.Sleep(3 * pollInterval)
	if grew := len(f.seen()) - quiet; grew > 0 {
		t.Fatalf("poller issued %d requests after Close", grew)
	}
}

func TestUnreachableDriveIsSeededAndLogged(t *testing.T) {
	// Before the drive has ever been read, the component still answers -- the
	// frame system queries it as soon as it is built, and erroring there took
	// down pose queries for the whole machine. Release is the fail-safe
	// assumption. The fault must still reach the log.
	f := newFakeMaster(t, "000A")
	logger, logs := logging.NewObservedTestLogger(t)
	r := newRemoverFor(resource.Name{Name: "test"}, NewClient(f.addr(), 1, logger),
		2*time.Second, logger)
	f.srv.Close()

	in, err := r.CurrentInputs(context.Background())
	if err != nil {
		t.Fatalf("CurrentInputs must answer before any read: %v", err)
	}
	if math.Abs(in[0]-KnifeTravelMM) > 0.001 {
		t.Fatalf("seeded offset %v, want %v (release)", in[0], KnifeTravelMM)
	}

	r.refresh(context.Background())
	for _, e := range logs.All() {
		if e.Level.String() == "error" && strings.Contains(e.Message, "poll failed") {
			return
		}
	}
	t.Fatalf("an unreachable drive must log at error level, got %v", logs.All())
}
