package autochanger

import (
	"bytes"
	"testing"
	"time"
)

func newTestMaster(t *testing.T) (*fakeAL1342, *Master) {
	t.Helper()
	f, addr := startFakeAL1342(t)
	m, release, err := SharedMaster(addr, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(release)
	return f, m
}

func TestPortDataRoundTrip(t *testing.T) {
	f, m := newTestMaster(t)
	f.setReg(1002, 0x0005) // X01 PD-in: State In + State Move
	got, err := m.ReadPortIn(1)
	if err != nil || got != 0x0005 {
		t.Fatalf("ReadPortIn = %#x, %v", got, err)
	}
	if err := m.WritePortOut(1, 0x0002); err != nil {
		t.Fatal(err)
	}
	if f.getReg(1101) != 0x0002 {
		t.Fatalf("PD-out reg = %#x, want 0x0002", f.getReg(1101))
	}
}

func TestSetManifoldBitsPreservesOthers(t *testing.T) {
	f, m := newTestMaster(t)
	f.setReg(2101, 0x00F0)
	if err := m.SetManifoldBits(2, 0x0001, 0x0001); err != nil { // set bit 0
		t.Fatal(err)
	}
	if f.getReg(2101) != 0x00F1 {
		t.Fatalf("manifold = %#x, want 0x00F1", f.getReg(2101))
	}
	if err := m.SetManifoldBits(2, 0x0001, 0x0000); err != nil { // clear bit 0
		t.Fatal(err)
	}
	if f.getReg(2101) != 0x00F0 {
		t.Fatalf("manifold = %#x, want 0x00F0", f.getReg(2101))
	}
}

func TestISDUWriteRead(t *testing.T) {
	f, m := newTestMaster(t)
	want := []byte{0x41, 0xC8, 0x00, 0x00} // 25.0 as big-endian float32
	if err := m.ISDUWrite(1, 0x0106, 0, want); err != nil {
		t.Fatal(err)
	}
	if got := f.isdu[isduKey(1, 0x0106, 0)]; !bytes.Equal(got, want) {
		t.Fatalf("stored ISDU = %x, want %x", got, want)
	}
	got, err := m.ISDURead(1, 0x0106, 0, 4)
	if err != nil || !bytes.Equal(got, want) {
		t.Fatalf("ISDURead = %x, %v", got, err)
	}
}

func TestSharedMasterIsShared(t *testing.T) {
	_, addr := startFakeAL1342(t)
	m1, rel1, err := SharedMaster(addr, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	m2, rel2, err := SharedMaster(addr, time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if m1 != m2 {
		t.Fatal("expected same Master instance for same address")
	}
	rel1()
	if _, err := m2.ReadPortIn(1); err != nil {
		t.Fatalf("master closed while still referenced: %v", err)
	}
	rel2()
}

func TestISDUOddLengthData(t *testing.T) {
	_, m := newTestMaster(t)
	want := []byte{0xAA, 0xBB, 0xCC} // 3-byte payload exercises odd-length packing/unpacking
	if err := m.ISDUWrite(1, 0x0200, 0, want); err != nil {
		t.Fatal(err)
	}
	got, err := m.ISDURead(1, 0x0200, 0, 3)
	if err != nil || !bytes.Equal(got, want) {
		t.Fatalf("ISDURead odd-length = %x, want %x, err %v", got, want, err)
	}
}

func TestISDUUserIDWrap(t *testing.T) {
	_, m := newTestMaster(t)
	m.mu.Lock()
	m.userID = 255 // on next increment, will become 0, then skip to 1
	m.mu.Unlock()

	// First write: userID becomes 0, skipped to 1
	if err := m.ISDUWrite(1, 0x0300, 0, []byte{0x01}); err != nil {
		t.Fatal(err)
	}
	// Second write: userID becomes 2
	if err := m.ISDUWrite(1, 0x0301, 0, []byte{0x02}); err != nil {
		t.Fatal(err)
	}
	// Both should succeed without error
}

func TestISDUTimeout(t *testing.T) {
	f, m := newTestMaster(t)
	m.isduTimeout = 50 * time.Millisecond // shorter timeout for fast test
	f.dropISDU = true                     // fake won't respond

	_, err := m.ISDURead(1, 0x0400, 0, 0)
	if err == nil {
		t.Fatal("expected timeout error, got nil")
	}
	if errStr := err.Error(); !contains(errStr, "timeout") {
		t.Fatalf("expected timeout in error, got: %v", err)
	}
}

func contains(s, substr string) bool {
	for i := 0; i < len(s)-len(substr)+1; i++ {
		if s[i:i+len(substr)] == substr {
			return true
		}
	}
	return false
}
