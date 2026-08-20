// Package autochanger controls the Mirka AutoChanger (remover knife +
// abrasive magazines) through an IFM AL1342 IO-Link master over Modbus TCP.
package autochanger

import (
	"fmt"
	"sync"
	"time"

	"github.com/simonvetter/modbus"
)

// AL1342 register map (per-port registers scale with port number).
// See "IO-Link Master with Modbus TCP Interface" manual, ch. 9.3.
const (
	regPortIn   = 2   // + port*1000: PD-in, IO-Link input data
	regPortDiag = 1   // + port*1000: diagnostic + status word
	regPortOut  = 101 // + port*1000: PD-out, IO-Link output data
	regISDUReq  = 500 // acyclic request channel base
	regISDUResp = 0   // acyclic response channel base
)

type Master struct {
	mu          sync.Mutex
	client      *modbus.ModbusClient
	userID      uint8
	refs        int
	addr        string
	isduTimeout time.Duration
}

var (
	mastersMu sync.Mutex
	masters   = map[string]*Master{}
)

// SharedMaster returns the process-wide Master for addr, creating it on
// first use. Sharing is required: the manifold PD-out word is written by
// multiple components and must be read-modify-written under one mutex.
func SharedMaster(addr string, timeout time.Duration) (*Master, func(), error) {
	mastersMu.Lock()
	defer mastersMu.Unlock()
	if m, ok := masters[addr]; ok {
		m.refs++
		return m, m.releaseFunc(), nil
	}
	client, err := modbus.NewClient(&modbus.ClientConfiguration{
		URL:     "tcp://" + addr,
		Timeout: timeout,
	})
	if err != nil {
		return nil, nil, fmt.Errorf("constructing modbus client for %s: %w", addr, err)
	}
	if err := client.Open(); err != nil {
		return nil, nil, fmt.Errorf("connecting to AL1342 at %s: %w", addr, err)
	}
	m := &Master{client: client, refs: 1, addr: addr, isduTimeout: 2 * time.Second}
	masters[addr] = m
	return m, m.releaseFunc(), nil
}

func (m *Master) releaseFunc() func() {
	var once sync.Once
	return func() {
		once.Do(func() {
			mastersMu.Lock()
			defer mastersMu.Unlock()
			m.refs--
			if m.refs == 0 {
				m.client.Close()
				delete(masters, m.addr)
			}
		})
	}
}

func (m *Master) ReadPortIn(port int) (uint16, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.client.ReadRegister(uint16(port*1000+regPortIn), modbus.HOLDING_REGISTER)
}

func (m *Master) ReadPortDiag(port int) (uint16, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.client.ReadRegister(uint16(port*1000+regPortDiag), modbus.HOLDING_REGISTER)
}

func (m *Master) WritePortOut(port int, word uint16) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.client.WriteRegister(uint16(port*1000+regPortOut), word)
}

func (m *Master) SetManifoldBits(port int, mask, value uint16) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	old, err := m.client.ReadRegister(uint16(port*1000+regPortOut), modbus.HOLDING_REGISTER)
	if err != nil {
		return fmt.Errorf("read manifold word: %w", err)
	}
	return m.client.WriteRegister(uint16(port*1000+regPortOut), (old&^mask)|(value&mask))
}

// isduExec runs one acyclic command while holding the mutex: the AL1342
// has a single command channel, so requests must be serialized.
func (m *Master) isduExec(port int, index uint16, sub uint8, cmd uint8, data []byte) ([]byte, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if len(data) > 34 {
		return nil, fmt.Errorf("ISDU data too long: %d bytes (max 34)", len(data))
	}
	m.userID++
	if m.userID == 0 { // a CHANGED userID triggers execution; 0 never used
		m.userID = 1
	}
	req := make([]uint16, 5+(len(data)+1)/2)
	req[0] = uint16(port)
	req[1] = index
	req[2] = uint16(sub)
	req[3] = uint16(cmd)<<8 | uint16(m.userID)
	req[4] = uint16(len(data))
	for i, b := range data {
		if i%2 == 0 {
			req[5+i/2] |= uint16(b) << 8
		} else {
			req[5+i/2] |= uint16(b)
		}
	}
	if err := m.client.WriteRegisters(regISDUReq, req); err != nil {
		return nil, fmt.Errorf("write ISDU request: %w", err)
	}

	deadline := time.Now().Add(m.isduTimeout)
	for {
		resp, err := m.client.ReadRegisters(regISDUResp, 22, modbus.HOLDING_REGISTER)
		if err != nil {
			return nil, fmt.Errorf("read ISDU response: %w", err)
		}
		if resp[3] == req[3] { // reflected command|userID: command finished
			switch resp[4] {
			case 0x0000, 0x000F:
			default:
				return nil, fmt.Errorf(
					"ISDU port %d index 0x%04X sub %d failed: error 0x%02X additional 0x%02X",
					port, index, sub, byte(resp[6]>>8), byte(resp[6]))
			}
			n := int(resp[5])
			if n > 32 { // response data area is regs 6..21 = 32 bytes; the device controls resp[5]
				n = 32
			}
			out := make([]byte, n)
			for i := 0; i < n; i++ {
				w := resp[6+i/2]
				if i%2 == 0 {
					out[i] = byte(w >> 8)
				} else {
					out[i] = byte(w)
				}
			}
			return out, nil
		}
		if time.Now().After(deadline) {
			return nil, fmt.Errorf("ISDU port %d index 0x%04X: timeout waiting for response", port, index)
		}
		time.Sleep(20 * time.Millisecond)
	}
}

func (m *Master) ISDURead(port int, index uint16, sub uint8, maxLen int) ([]byte, error) {
	out, err := m.isduExec(port, index, sub, 0x01, nil)
	if err != nil {
		return nil, err
	}
	if maxLen > 0 && len(out) > maxLen {
		out = out[:maxLen]
	}
	return out, nil
}

func (m *Master) ISDUWrite(port int, index uint16, sub uint8, data []byte) error {
	_, err := m.isduExec(port, index, sub, 0x02, data)
	return err
}
