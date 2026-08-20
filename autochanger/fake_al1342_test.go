package autochanger

import (
	"fmt"
	"net"
	"sync"
	"testing"

	"github.com/simonvetter/modbus"
)

// fakeAL1342 simulates the AL1342 register surface: arbitrary holding
// registers, plus the acyclic ISDU channel (write regs 500.., response in
// regs 0..). ISDU values live in isdu keyed by "port/index/sub".
type fakeAL1342 struct {
	mu   sync.Mutex
	regs map[uint16]uint16
	isdu map[string][]byte
	// isduWrites records ISDUWrite calls in order, for sequence assertions.
	isduWrites []string
	// dropISDU, if true, causes execISDU to skip writing response registers.
	dropISDU bool
}

func isduKey(port, index uint16, sub uint16) string {
	return fmt.Sprintf("%d/0x%04X/%d", port, index, sub)
}

func newFakeAL1342() *fakeAL1342 {
	return &fakeAL1342{regs: map[uint16]uint16{}, isdu: map[string][]byte{}}
}

func (f *fakeAL1342) HandleCoils(req *modbus.CoilsRequest) ([]bool, error) {
	return nil, modbus.ErrIllegalFunction
}

func (f *fakeAL1342) HandleDiscreteInputs(req *modbus.DiscreteInputsRequest) ([]bool, error) {
	return nil, modbus.ErrIllegalFunction
}

func (f *fakeAL1342) HandleInputRegisters(req *modbus.InputRegistersRequest) ([]uint16, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	out := make([]uint16, req.Quantity)
	for i := range out {
		out[i] = f.regs[req.Addr+uint16(i)]
	}
	return out, nil
}

func (f *fakeAL1342) HandleHoldingRegisters(req *modbus.HoldingRegistersRequest) ([]uint16, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if !req.IsWrite {
		out := make([]uint16, req.Quantity)
		for i := range out {
			out[i] = f.regs[req.Addr+uint16(i)]
		}
		return out, nil
	}
	for i, v := range req.Args {
		f.regs[req.Addr+uint16(i)] = v
	}
	// A write that covers reg 503 (command|userID) triggers ISDU execution.
	if req.Addr <= 503 && req.Addr+req.Quantity > 503 {
		f.execISDU()
	}
	return nil, nil
}

// execISDU services the request channel synchronously: real hardware takes
// milliseconds, the fake is instant — the client's poll loop still works.
func (f *fakeAL1342) execISDU() {
	if f.dropISDU {
		return // test: simulate no response from hardware
	}
	port, index, sub := f.regs[500], f.regs[501], f.regs[502]
	cmdUser := f.regs[503]
	key := isduKey(port, index, sub)
	// Mirror the addressing fields + reflected command|userID.
	f.regs[0], f.regs[1], f.regs[2], f.regs[3] = port, index, sub, cmdUser
	switch cmdUser >> 8 {
	case 0x01: // read
		data := f.isdu[key]
		f.regs[4] = 0x0000
		f.regs[5] = uint16(len(data))
		for i := 0; i < len(data); i += 2 {
			w := uint16(data[i]) << 8
			if i+1 < len(data) {
				w |= uint16(data[i+1])
			}
			f.regs[6+uint16(i/2)] = w
		}
	case 0x02: // write
		n := int(f.regs[504])
		data := make([]byte, n)
		for i := 0; i < n; i++ {
			w := f.regs[505+uint16(i/2)]
			if i%2 == 0 {
				data[i] = byte(w >> 8)
			} else {
				data[i] = byte(w)
			}
		}
		f.isdu[key] = data
		f.isduWrites = append(f.isduWrites, key)
		f.regs[4] = 0x0000
		f.regs[5] = 0
	default:
		f.regs[4] = 0x00FF
		f.regs[6] = 0x7100 // service not available
	}
}

// set/get helpers for tests: PD words and ISDU values.
func (f *fakeAL1342) setReg(addr, val uint16) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.regs[addr] = val
}

func (f *fakeAL1342) getReg(addr uint16) uint16 {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.regs[addr]
}

// startFakeAL1342 serves the fake on an ephemeral localhost port and
// returns its address. Cleanup stops the server.
func startFakeAL1342(t *testing.T) (*fakeAL1342, string) {
	t.Helper()
	f := newFakeAL1342()
	l, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	addr := l.Addr().String()
	l.Close()
	srv, err := modbus.NewServer(&modbus.ServerConfiguration{
		URL: "tcp://" + addr, MaxClients: 5,
	}, f)
	if err != nil {
		t.Fatal(err)
	}
	if err := srv.Start(); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { srv.Stop() })
	return f, addr
}
