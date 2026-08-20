package autochanger

import (
	"context"
	"encoding/binary"
	"fmt"
	"math"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

// stubMaster implements al1342 for tests. pdIn is returned by ReadPortIn;
// onWrite lets a test flip pdIn when a move command lands, simulating the
// drive reaching its target.
type stubMaster struct {
	mu      sync.Mutex
	pdIn    uint16
	pdOut   []uint16
	isdu    map[string][]byte
	writes  []string // "isdu:<key>" and "pd:<word>" and "manifold:<mask>/<value>" in order
	onWrite func(word uint16)
}

func newStubMaster() *stubMaster {
	return &stubMaster{isdu: map[string][]byte{}}
}

func (s *stubMaster) ReadPortIn(port int) (uint16, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.pdIn, nil
}

func (s *stubMaster) ReadPortDiag(port int) (uint16, error) { return 0, nil }

func (s *stubMaster) WritePortOut(port int, word uint16) error {
	s.mu.Lock()
	s.pdOut = append(s.pdOut, word)
	s.writes = append(s.writes, fmt.Sprintf("pd:%#04x", word))
	cb := s.onWrite
	s.mu.Unlock()
	if cb != nil {
		cb(word)
	}
	return nil
}

func (s *stubMaster) SetManifoldBits(port int, mask, value uint16) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.writes = append(s.writes, fmt.Sprintf("manifold:%#04x/%#04x", mask, value))
	return nil
}

func (s *stubMaster) ISDURead(port int, index uint16, sub uint8, maxLen int) ([]byte, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.isdu[isduKey(uint16(port), index, uint16(sub))], nil
}

func (s *stubMaster) ISDUWrite(port int, index uint16, sub uint8, data []byte) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	key := isduKey(uint16(port), index, uint16(sub))
	s.isdu[key] = data
	s.writes = append(s.writes, "isdu:"+key)
	return nil
}

// reachOnMove wires the stub so a Move In lands at State In and a Move Out
// lands at State Out, immediately (no State Move phase).
func (s *stubMaster) reachOnMove() {
	s.onWrite = func(word uint16) {
		s.mu.Lock()
		defer s.mu.Unlock()
		switch {
		case word&smsMoveIn != 0:
			s.pdIn = smsStateIn
		case word&smsMoveOut != 0:
			s.pdIn = smsStateOut
		}
	}
}

func testRemover(t *testing.T, s *stubMaster) *remover {
	t.Helper()
	return &remover{
		master:       s,
		removerPort:  1,
		manifoldPort: 2,
		nozzleValve:  0,
		position1MM:  1.5,
		position3MM:  25.0,
		moveTimeout:  500 * time.Millisecond,
		blowSeconds:  0.01,
		logger:       logging.NewTestLogger(t),
	}
}

func TestSetPosition2MovesIn(t *testing.T) {
	s := newStubMaster()
	s.reachOnMove()
	r := testRemover(t, s)
	r.homed = true
	if err := r.setPosition(context.Background(), 2, 0); err != nil {
		t.Fatal(err)
	}
	// Position 2 is Move In only: no ISDU end-position write.
	if len(s.writes) != 1 || s.writes[0] != "pd:0x0001" {
		t.Fatalf("writes = %v, want single Move In", s.writes)
	}
}

func TestSetPosition3WritesEndPosThenMovesOut(t *testing.T) {
	s := newStubMaster()
	s.reachOnMove()
	r := testRemover(t, s)
	if err := r.setPosition(context.Background(), 3, 0); err != nil {
		t.Fatal(err)
	}
	want := []string{"isdu:" + isduKey(1, 0x0106, 0), "pd:0x0002"}
	if len(s.writes) != 2 || s.writes[0] != want[0] || s.writes[1] != want[1] {
		t.Fatalf("writes = %v, want %v", s.writes, want)
	}
	// 25.0 mm as big-endian float32
	bits := binary.BigEndian.Uint32(s.isdu[isduKey(1, 0x0106, 0)])
	if math.Float32frombits(bits) != 25.0 {
		t.Fatalf("end position = %v, want 25.0", math.Float32frombits(bits))
	}
}

func TestSetPosition2RequiresHoming(t *testing.T) {
	s := newStubMaster()
	s.reachOnMove()
	r := testRemover(t, s)
	if err := r.setPosition(context.Background(), 2, 0); err == nil {
		t.Fatal("expected error: position 2 without homing")
	}
}

func TestMoveTimesOutWhenNeverReached(t *testing.T) {
	s := newStubMaster() // no reachOnMove: state bits never change
	r := testRemover(t, s)
	if err := r.setPosition(context.Background(), 3, 0); err == nil {
		t.Fatal("expected timeout error")
	}
	if len(s.writes) == 0 || s.writes[len(s.writes)-1] != "pd:0x0000" {
		t.Fatalf("writes = %v, want last write pd:0x0000 (motion-bit clear on timeout)", s.writes)
	}
	if r.position != 0 {
		t.Fatalf("position = %d, want 0 (unknown after failed move)", r.position)
	}
}

// TestQuitErrorResetsState covers the quit_error path: it must clear the
// stale homed/position state left behind by whatever fault triggered it.
func TestQuitErrorResetsState(t *testing.T) {
	s := newStubMaster()
	r := testRemover(t, s)
	r.homed = true
	r.position = 2

	got, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "quit_error"})
	if err != nil {
		t.Fatal(err)
	}
	if got["cleared"] != true {
		t.Fatalf("response = %v, want cleared:true", got)
	}
	if r.homed || r.position != 0 {
		t.Fatalf("homed=%v position=%d, want false/0", r.homed, r.position)
	}
	want := []string{"pd:0x0004", "pd:0x0000"}
	if len(s.writes) != len(want) || s.writes[0] != want[0] || s.writes[1] != want[1] {
		t.Fatalf("writes = %v, want %v", s.writes, want)
	}
}

// TestHomeSuccess covers a successful reference run overwriting a stale
// homed flag left over from before the run started.
func TestHomeSuccess(t *testing.T) {
	s := newStubMaster()
	s.reachOnMove()
	// Homing has no PD-out Move write for the stub's onWrite hook to react
	// to (it runs over ISDU), so simulate the drive already parked at the
	// reference end position when waitForState polls.
	s.pdIn = smsStateIn
	r := testRemover(t, s)
	r.homed = true // stale, from a previous run

	got, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "home"})
	if err != nil {
		t.Fatal(err)
	}
	if got["homed"] != true {
		t.Fatalf("response = %v, want homed:true", got)
	}
	wantWrite := "isdu:" + isduKey(1, isduExecHome, 0)
	found := false
	for _, w := range s.writes {
		if w == wantWrite {
			found = true
		}
	}
	if !found {
		t.Fatalf("writes = %v, want %s", s.writes, wantWrite)
	}
	if !r.homed || r.position != 2 {
		t.Fatalf("homed=%v position=%d, want true/2", r.homed, r.position)
	}
}

// TestHomeFailureClearsHomed proves a failed reference run doesn't leave a
// stale homed=true: the previous datum is invalid whether or not the retry
// succeeds.
func TestHomeFailureClearsHomed(t *testing.T) {
	s := newStubMaster() // PD-in never changes: waitForState times out
	r := testRemover(t, s)
	r.homed = true

	if err := r.home(context.Background()); err == nil {
		t.Fatal("expected timeout error")
	}
	if r.homed {
		t.Fatal("r.homed = true, want false after failed home")
	}
}

func TestDeviceErrorSurfaced(t *testing.T) {
	s := newStubMaster()
	s.onWrite = func(word uint16) {
		s.mu.Lock()
		s.pdIn = smsStateDevice
		s.mu.Unlock()
	}
	r := testRemover(t, s)
	if err := r.setPosition(context.Background(), 3, 0); err == nil {
		t.Fatal("expected device error")
	}
}

func TestReleaseDiscSequence(t *testing.T) {
	s := newStubMaster()
	s.reachOnMove()
	r := testRemover(t, s)
	if _, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "release_disc"}); err != nil {
		t.Fatal(err)
	}
	// pos 3 (ISDU 25.0 + Move Out) → nozzle on → nozzle off → pos 1 (ISDU 1.5 + Move Out)
	want := []string{
		"isdu:" + isduKey(1, 0x0106, 0), "pd:0x0002",
		"manifold:0x0001/0x0001", "manifold:0x0001/0x0000",
		"isdu:" + isduKey(1, 0x0106, 0), "pd:0x0002",
	}
	if len(s.writes) != len(want) {
		t.Fatalf("writes = %v, want %v", s.writes, want)
	}
	for i := range want {
		if s.writes[i] != want[i] {
			t.Fatalf("write[%d] = %s, want %s (all: %v)", i, s.writes[i], want[i], s.writes)
		}
	}
}

func TestStatusDecodesPD(t *testing.T) {
	s := newStubMaster()
	s.pdIn = smsStateOut
	// current position 12.34mm = 1234 as big-endian int32 at ISDU 0x0120
	s.isdu[isduKey(1, 0x0120, 0)] = []byte{0x00, 0x00, 0x04, 0xD2}
	r := testRemover(t, s)
	got, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "status"})
	if err != nil {
		t.Fatal(err)
	}
	if got["state_out"] != true || got["moving"] != false {
		t.Fatalf("status = %v", got)
	}
	if got["position_mm"] != 12.34 {
		t.Fatalf("position_mm = %v, want 12.34", got["position_mm"])
	}
}

// TestReleaseDiscAtomic proves release_disc holds r.mu across its entire
// extend/blow/retract sequence. betweenStepsHook fires at the two step
// boundaries; if it ever runs outside the critical section, r.mu.TryLock
// succeeds there (the lock is up for grabs) and unlockedWindows goes
// nonzero. A racing-goroutine test can't catch a same-goroutine re-Lock
// reliably (Go mutexes aren't FIFO — the barging goroutine typically wins),
// so this probes the lock state directly instead of racing for it.
func TestReleaseDiscAtomic(t *testing.T) {
	s := newStubMaster()
	s.reachOnMove()
	r := testRemover(t, s)

	unlockedWindows := 0
	hookCalls := 0
	r.betweenStepsHook = func() {
		hookCalls++
		if r.mu.TryLock() {
			unlockedWindows++
			r.mu.Unlock()
		}
	}

	if _, err := r.DoCommand(context.Background(), map[string]interface{}{"command": "release_disc"}); err != nil {
		t.Fatal(err)
	}

	if unlockedWindows != 0 {
		t.Fatalf("r.mu was unlocked at %d of the between-step boundaries; release_disc must hold it throughout", unlockedWindows)
	}
	if hookCalls != 2 {
		t.Fatalf("betweenStepsHook fired %d times, want 2", hookCalls)
	}

	want := []string{
		"isdu:" + isduKey(1, 0x0106, 0), "pd:0x0002",
		"manifold:0x0001/0x0001", "manifold:0x0001/0x0000",
		"isdu:" + isduKey(1, 0x0106, 0), "pd:0x0002",
	}
	if len(s.writes) != len(want) {
		t.Fatalf("writes = %v, want %v", s.writes, want)
	}
	for i := range want {
		if s.writes[i] != want[i] {
			t.Fatalf("write[%d] = %s, want %s (all: %v)", i, s.writes[i], want[i], s.writes)
		}
	}
}
