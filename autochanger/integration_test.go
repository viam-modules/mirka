package autochanger

import (
	"context"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

func TestReleaseDiscEndToEnd(t *testing.T) {
	f, addr := startFakeAL1342(t)

	// Drive simulation: any Move Out/In write lands at its target state.
	// A goroutine watches PD-out because the real Master writes it directly.
	stop := make(chan struct{})
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		for {
			select {
			case <-stop:
				return
			default:
			}
			switch out := f.getReg(1101); {
			case out&smsMoveIn != 0:
				f.setReg(1002, smsStateIn)
			case out&smsMoveOut != 0:
				f.setReg(1002, smsStateOut)
			}
			time.Sleep(2 * time.Millisecond)
		}
	}()
	t.Cleanup(func() { close(stop); wg.Wait() })

	res, err := newRemover(context.Background(), nil, resource.Config{
		Name:                "remover",
		ConvertedAttributes: &RemoverConfig{ModbusAddress: addr, BlowSeconds: 0.01},
	}, logging.NewTestLogger(t))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { res.Close(context.Background()) })

	if _, err := res.DoCommand(context.Background(),
		map[string]interface{}{"command": "release_disc"}); err != nil {
		t.Fatal(err)
	}

	// Constructor writes speed-in/speed-out/force, then release_disc
	// writes end-pos twice (25 then 1.5): 5 ISDU writes total, in order.
	wantWrites := []string{
		isduKey(1, isduSpeedIn, 0), isduKey(1, isduSpeedOut, 0), isduKey(1, isduForce, 0),
		isduKey(1, isduEndPosOut, 0), isduKey(1, isduEndPosOut, 0),
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if len(f.isduWrites) != len(wantWrites) {
		t.Fatalf("ISDU writes = %v, want %v", f.isduWrites, wantWrites)
	}
	for i := range wantWrites {
		if f.isduWrites[i] != wantWrites[i] {
			t.Fatalf("ISDU write[%d] = %s, want %s", i, f.isduWrites[i], wantWrites[i])
		}
	}
}
