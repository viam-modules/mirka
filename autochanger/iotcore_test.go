package autochanger

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"net/http"
	"net/http/httptest"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

// captureServer records the last request body and replies with the given body.
func captureServer(t *testing.T, reply string, last *string) *httptest.Server {
	t.Helper()
	return httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		b, _ := io.ReadAll(r.Body)
		*last = string(b)
		w.Header().Set("Content-Type", "application/json")
		io.WriteString(w, reply)
	}))
}

func TestReadPD(t *testing.T) {
	var last string
	srv := captureServer(t, `{"cid":1,"data":{"value":"0018"},"code":200}`, &last)
	defer srv.Close()

	c := NewClient(strings.TrimPrefix(srv.URL, "http://"), 1, logging.NewTestLogger(t))
	got, err := c.ReadPD(context.Background())
	if err != nil {
		t.Fatalf("ReadPD: %v", err)
	}
	if got != 0x0018 {
		t.Fatalf("got 0x%04X, want 0x0018", got)
	}
	if !strings.Contains(last, "/iolinkmaster/port[1]/iolinkdevice/pdin/getdata") {
		t.Fatalf("wrong adr in request: %s", last)
	}
}

func TestReadPDDeviceError(t *testing.T) {
	var last string
	srv := captureServer(t, `{"cid":1,"error":"01","code":530}`, &last)
	defer srv.Close()

	c := NewClient(strings.TrimPrefix(srv.URL, "http://"), 1, logging.NewTestLogger(t))
	if _, err := c.ReadPD(context.Background()); err == nil {
		t.Fatal("expected an error for device code 530")
	}
}

func TestISDUReadDecodesHex(t *testing.T) {
	var last string
	// index 16, Vendor Name, as returned by the real device: "Festo\0"
	srv := captureServer(t, `{"cid":1,"data":{"value":"466573746F00"},"code":200}`, &last)
	defer srv.Close()

	c := NewClient(strings.TrimPrefix(srv.URL, "http://"), 1, logging.NewTestLogger(t))
	got, err := c.ISDURead(context.Background(), 16, 0)
	if err != nil {
		t.Fatalf("ISDURead: %v", err)
	}
	if string(got) != "Festo\x00" {
		t.Fatalf("got %q, want %q", got, "Festo\x00")
	}
}

func TestISDUWriteFloat32EncodesBigEndian(t *testing.T) {
	var last string
	srv := captureServer(t, `{"cid":1,"code":200}`, &last)
	defer srv.Close()

	c := NewClient(strings.TrimPrefix(srv.URL, "http://"), 1, logging.NewTestLogger(t))
	if err := c.ISDUWriteFloat32(context.Background(), 264, 0, 75.0); err != nil {
		t.Fatalf("ISDUWriteFloat32: %v", err)
	}
	// 75.0 as IEEE-754 float32 is 0x42960000, the value used at the bench.
	if !strings.Contains(last, `"value":"42960000"`) {
		t.Fatalf("wrong encoding in request: %s", last)
	}
	if !strings.Contains(last, `"index":264`) {
		t.Fatalf("wrong index in request: %s", last)
	}
}

func TestSystemCommandEncodesByte(t *testing.T) {
	var last string
	srv := captureServer(t, `{"cid":1,"code":200}`, &last)
	defer srv.Close()

	c := NewClient(strings.TrimPrefix(srv.URL, "http://"), 1, logging.NewTestLogger(t))
	if err := c.SystemCommand(context.Background(), CmdMoveIntermediate); err != nil {
		t.Fatalf("SystemCommand: %v", err)
	}
	var req map[string]any
	if err := json.Unmarshal([]byte(last), &req); err != nil {
		t.Fatalf("request was not JSON: %v", err)
	}
	if !strings.Contains(last, `"value":"D0"`) {
		t.Fatalf("wrong command byte: %s", last)
	}
	if !strings.Contains(last, "iolwriteacyclic") {
		t.Fatalf("system command must use iolwriteacyclic: %s", last)
	}
}

func TestClassifyTransportError(t *testing.T) {
	// The distinction the whole investigation turns on: a reset says something
	// about the AL1342, a cancellation says only that the caller stopped
	// listening. Both arrive wrapped, so this must match on the chain.
	for _, tc := range []struct {
		err  error
		want string
	}{
		{nil, "ok"},
		{context.Canceled, "canceled"},
		{fmt.Errorf("posting: %w", context.Canceled), "canceled"},
		{context.DeadlineExceeded, "deadline_exceeded"},
		{fmt.Errorf("read tcp: %w", syscall.ECONNRESET), "conn_reset"},
		{fmt.Errorf("dial: %w", syscall.ECONNREFUSED), "conn_refused"},
		{errors.New("something else"), "other"},
	} {
		if got := classifyTransportError(tc.err); got != tc.want {
			t.Fatalf("classify(%v) = %q, want %q", tc.err, got, tc.want)
		}
	}
}

func TestCancelledCallerAbandonsTheQueue(t *testing.T) {
	// The hypothesis this build exists to test. A sync.Mutex here let a caller
	// block for a full HTTP timeout and then send with a dead context, which
	// surfaced as "context canceled" and looked like a device fault. A cancelled
	// caller must abandon the queue instead.
	release := make(chan struct{})
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		<-release
		io.WriteString(w, `{"cid":1,"data":{"value":"0009"},"code":200}`)
	}))
	defer srv.Close()
	defer close(release)

	c := NewClient(strings.TrimPrefix(srv.URL, "http://"), 1, logging.NewTestLogger(t))

	// Occupy the transport.
	go func() { _, _ = c.ReadPD(context.Background()) }()
	time.Sleep(50 * time.Millisecond)

	ctx, cancel := context.WithCancel(context.Background())
	go func() {
		time.Sleep(20 * time.Millisecond)
		cancel()
	}()

	start := time.Now()
	_, err := c.ReadPD(ctx)
	waited := time.Since(start)

	if err == nil {
		t.Fatal("expected the cancelled caller to fail")
	}
	if !errors.Is(err, context.Canceled) {
		t.Fatalf("want a cancellation, got %v", err)
	}
	// It must give up when cancelled, not sit out the full 3s HTTP timeout of
	// the request ahead of it.
	if waited > time.Second {
		t.Fatalf("cancelled caller waited %s; it should abandon the queue promptly", waited)
	}
}

func TestClientDisablesKeepAlives(t *testing.T) {
	// The AL1342 drops idle connections without a FIN. A pooled socket is
	// reused and the next request fails with "read: connection reset by peer",
	// which was observed failing frame system pose queries for the arm.
	c := NewClient("192.0.2.1", 1, logging.NewTestLogger(t))
	tr, ok := c.http.Transport.(*http.Transport)
	if !ok {
		t.Fatalf("transport is %T, want *http.Transport", c.http.Transport)
	}
	if !tr.DisableKeepAlives {
		t.Fatal("keep-alives must be disabled for the AL1342")
	}
}

func TestClientSerializesRequests(t *testing.T) {
	// The AL1342's embedded server has a low connection ceiling, and with
	// keep-alives disabled every concurrent caller is a separate socket. Driven
	// through the client directly: the component's read methods no longer touch
	// the transport at all.
	f := newFakeMaster(t, "0009")
	c := NewClient(f.addr(), 1, logging.NewTestLogger(t))

	var inFlight, peak int32
	f.setOnRequest(func() {
		if n := atomic.AddInt32(&inFlight, 1); n > atomic.LoadInt32(&peak) {
			atomic.StoreInt32(&peak, n)
		}
		time.Sleep(5 * time.Millisecond)
		atomic.AddInt32(&inFlight, -1)
	})

	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			_, _ = c.ReadPD(context.Background())
		}()
	}
	wg.Wait()

	if p := atomic.LoadInt32(&peak); p != 1 {
		t.Fatalf("peak concurrent requests = %d, want 1", p)
	}
}

var valueRE = regexp.MustCompile(`"value":"([0-9A-Fa-f]+)"`)

// fakeMaster serves a fixed process data word and position register, and
// records every request body.
type fakeMaster struct {
	srv *httptest.Server

	// A background poller reads these concurrently with the test that sets
	// them, so every field below is guarded.
	mu       sync.Mutex
	requests []string
	pd       string
	counts   string // 8 hex digits, as the drive returns them

	// onRequest, when set, runs inside the handler so a test can observe how
	// many requests are in flight at once.
	onRequest func()

	// simulate makes the fake obey move commands: an intermediate move lands on
	// whatever PosImp was last written, and the end stops land on theirs. Off by
	// default so tests about refusal and timeout can pin the drive still.
	simulate bool
	posImp   string
}

// countsForPD keeps the fake self-consistent: a test that says "parked at the
// release end" should not have to state the encoder reading as well.
func countsForPD(pd string) string {
	switch pd {
	case "0009": // at zero
		return "00000000"
	case "000A": // at release
		return "00000A29" // 2601
	case "0018": // at the 1.1mm grip gap
		return "00000047" // 71
	default:
		return "00000000"
	}
}

func newFakeMaster(t *testing.T, pd string) *fakeMaster {
	t.Helper()
	f := &fakeMaster{pd: pd, counts: countsForPD(pd)}
	f.srv = httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		b, _ := io.ReadAll(r.Body)
		body := string(b)
		f.mu.Lock()
		hook := f.onRequest
		f.requests = append(f.requests, body)
		pd, counts := f.pd, f.counts
		f.mu.Unlock()
		if hook != nil {
			hook()
		}
		w.Header().Set("Content-Type", "application/json")
		if strings.Contains(body, "pdin/getdata") {
			io.WriteString(w, `{"cid":1,"data":{"value":"`+pd+`"},"code":200}`)
			return
		}
		if strings.Contains(body, "iolreadacyclic") && strings.Contains(body, `"index":288`) {
			io.WriteString(w, `{"cid":1,"data":{"value":"`+counts+`"},"code":200}`)
			return
		}
		f.applyCommand(body)
		io.WriteString(w, `{"cid":1,"code":200}`)
	}))
	t.Cleanup(f.srv.Close)
	return f
}

func (f *fakeMaster) addr() string { return strings.TrimPrefix(f.srv.URL, "http://") }

// seen returns a copy of the request bodies recorded so far.
func (f *fakeMaster) seen() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]string(nil), f.requests...)
}

// setCounts moves the simulated drive.
func (f *fakeMaster) setCounts(counts string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.counts = counts
}

// setPD sets the process data word the drive reports.
func (f *fakeMaster) setPD(pd string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.pd = pd
}

func (f *fakeMaster) setOnRequest(fn func()) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.onRequest = fn
}

// simulateDrive makes the fake obey move commands.
func (f *fakeMaster) simulateDrive() {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.simulate = true
}

// applyCommand moves the simulated drive in response to a write.
func (f *fakeMaster) applyCommand(body string) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if !f.simulate || !strings.Contains(body, "iolwriteacyclic") {
		return
	}
	m := valueRE.FindStringSubmatch(body)
	if m == nil {
		return
	}
	switch {
	case strings.Contains(body, `"index":264`):
		bits, _ := strconv.ParseUint(m[1], 16, 32)
		f.posImp = fmt.Sprintf("%08X", int32(math.Round(float64(math.Float32frombits(uint32(bits))))))
	case strings.Contains(body, `"index":2,`):
		switch m[1] {
		case "C8":
			f.counts, f.pd = "00000000", "0009"
		case "C9":
			f.counts, f.pd = fmt.Sprintf("%08X", KnifeTravelCounts), "000A"
		case "D0":
			f.counts, f.pd = f.posImp, "0018"
		}
	}
}
