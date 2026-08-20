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
