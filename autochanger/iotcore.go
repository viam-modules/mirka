// Package autochanger controls the Mirka AutoChanger remover knife through an
// IFM AL1342 IO-Link master.
package autochanger

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"net"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	"go.viam.com/rdk/logging"
)

// EMCS-ST system commands, written to IO-Link index 2. The only command bytes
// this package may emit. The reference commands 0xCE/0xCF are absent because
// homing needs the sliding plate removed first, and a computed byte one digit
// from 0xCD (Restore Factory Settings) is not worth carrying. See the README.
const (
	CmdMoveIn           byte = 0xC8
	CmdMoveOut          byte = 0xC9
	CmdStop             byte = 0xCA
	CmdMoveIntermediate byte = 0xD0
)

// isduSystemCommand is IO-Link's standard index for system commands.
const isduSystemCommand = 2

// httpTimeout bounds a single request, not physical completion -- the move
// timeout in remover.go does that.
const httpTimeout = 3 * time.Second

type Client struct {
	addr   string
	port   int
	http   *http.Client
	logger logging.Logger
	born   time.Time

	// sem admits one request at a time: the AL1342's embedded server has a low
	// connection ceiling, and with keep-alives disabled every concurrent caller
	// is a separate socket. A channel rather than a Mutex so the wait can be
	// abandoned -- a plain Lock() let a caller block for a full HTTP timeout and
	// then send with a dead context, which read like a device fault.
	sem chan struct{}

	inFlight atomic.Int64

	statsMu sync.Mutex
	stats   transportStats
}

type transportStats struct {
	requests    int64
	errors      int64
	byClass     map[string]int64
	maxWaitMS   float64
	maxRTTMS    float64
	maxInFlight int64
	lastError   string
	lastErrorAt time.Time
	firstOK     time.Time

	// Per-endpoint timing. Process data is a cyclic value the master already
	// holds; an ISDU read is an acyclic transfer to the device itself. They are
	// not the same price, and the poll rate has to be built around which.
	byAdr map[string]*adrStats
}

type adrStats struct {
	Count     int64   `json:"count"`
	MaxRTTMS  float64 `json:"max_rtt_ms"`
	MeanRTTMS float64 `json:"mean_rtt_ms"`
	totalRTT  float64
}

// NewClient returns a client for the master at addr (host or host:port),
// talking to the given IO-Link port.
func NewClient(addr string, port int, logger logging.Logger) *Client {
	if !strings.Contains(addr, ":") {
		addr += ":80"
	}
	return &Client{
		addr:   addr,
		port:   port,
		logger: logger,
		born:   time.Now(),
		sem:    make(chan struct{}, 1),
		http: &http.Client{
			Timeout: httpTimeout,
			// The AL1342 drops idle connections without a FIN, so a pooled
			// socket fails the next request with "connection reset by peer".
			// A handshake per request is nothing on a point-to-point link.
			Transport: &http.Transport{DisableKeepAlives: true},
		},
	}
}

// classifyTransportError separates faults the device caused from faults the
// caller caused. They must not be counted together: a cancellation says nothing
// about the AL1342.
func classifyTransportError(err error) string {
	switch {
	case err == nil:
		return "ok"
	case errors.Is(err, context.Canceled):
		return "canceled"
	case errors.Is(err, context.DeadlineExceeded):
		return "deadline_exceeded"
	case errors.Is(err, syscall.ECONNRESET):
		return "conn_reset"
	case errors.Is(err, syscall.ECONNREFUSED):
		return "conn_refused"
	case errors.Is(err, syscall.EHOSTUNREACH):
		return "host_unreachable"
	case errors.Is(err, syscall.ENETUNREACH):
		return "net_unreachable"
	case errors.Is(err, syscall.EPIPE):
		return "broken_pipe"
	}
	var ne net.Error
	if errors.As(err, &ne) && ne.Timeout() {
		return "timeout"
	}
	return "other"
}

// Stats returns a snapshot of the transport counters.
func (c *Client) Stats() map[string]interface{} {
	c.statsMu.Lock()
	defer c.statsMu.Unlock()

	byClass := map[string]interface{}{}
	for k, v := range c.stats.byClass {
		byClass[k] = v
	}
	byAdr := map[string]interface{}{}
	for k, v := range c.stats.byAdr {
		byAdr[k] = map[string]interface{}{
			"count": v.Count, "max_rtt_ms": v.MaxRTTMS, "mean_rtt_ms": v.MeanRTTMS,
		}
	}
	out := map[string]interface{}{
		"by_adr":        byAdr,
		"address":       c.addr,
		"port":          c.port,
		"requests":      c.stats.requests,
		"errors":        c.stats.errors,
		"by_class":      byClass,
		"max_wait_ms":   c.stats.maxWaitMS,
		"max_rtt_ms":    c.stats.maxRTTMS,
		"max_in_flight": c.stats.maxInFlight,
		"client_age":    time.Since(c.born).Round(time.Millisecond).String(),
	}
	if !c.stats.firstOK.IsZero() {
		out["first_ok_after"] = c.stats.firstOK.Sub(c.born).Round(time.Millisecond).String()
	} else {
		out["first_ok_after"] = "never"
	}
	if c.stats.lastError != "" {
		out["last_error"] = c.stats.lastError
		out["last_error_age"] = time.Since(c.stats.lastErrorAt).Round(time.Millisecond).String()
	}
	return out
}

// trace is one request's timing, recorded whether or not it succeeded.
type trace struct {
	adr    string
	waited time.Duration
	// rtt is zero if the request was never sent.
	rtt    time.Duration
	queued int64
	// ctxDeadBeforeSend is the smoking gun for queueing-induced cancellation:
	// the context was already dead by the time this request reached the wire.
	ctxDeadBeforeSend bool
}

func (c *Client) record(ctx context.Context, t trace, err error) {
	class := classifyTransportError(err)

	c.statsMu.Lock()
	if c.stats.byClass == nil {
		c.stats.byClass = map[string]int64{}
	}
	c.stats.requests++
	c.stats.byClass[class]++
	if ms := t.waited.Seconds() * 1000; ms > c.stats.maxWaitMS {
		c.stats.maxWaitMS = ms
	}
	if ms := t.rtt.Seconds() * 1000; ms > c.stats.maxRTTMS {
		c.stats.maxRTTMS = ms
	}
	if t.queued > c.stats.maxInFlight {
		c.stats.maxInFlight = t.queued
	}
	if c.stats.byAdr == nil {
		c.stats.byAdr = map[string]*adrStats{}
	}
	key := t.adr
	if i := strings.LastIndex(key, "iolinkdevice/"); i >= 0 {
		key = key[i+len("iolinkdevice/"):]
	}
	a := c.stats.byAdr[key]
	if a == nil {
		a = &adrStats{}
		c.stats.byAdr[key] = a
	}
	a.Count++
	ms := t.rtt.Seconds() * 1000
	a.totalRTT += ms
	a.MeanRTTMS = a.totalRTT / float64(a.Count)
	if ms > a.MaxRTTMS {
		a.MaxRTTMS = ms
	}
	firstOK := false
	if err == nil && c.stats.firstOK.IsZero() {
		c.stats.firstOK, firstOK = time.Now(), true
	}
	if err != nil {
		c.stats.errors++
		c.stats.lastError = err.Error()
		c.stats.lastErrorAt = time.Now()
	}
	n := c.stats.requests
	c.statsMu.Unlock()

	if c.logger == nil {
		return
	}
	switch {
	case err != nil:
		fields := []any{
			"class", class,
			"adr", t.adr,
			"waited_ms", t.waited.Seconds() * 1000,
			"rtt_ms", t.rtt.Seconds() * 1000,
			"in_flight_on_arrival", t.queued,
			"ctx_dead_before_send", t.ctxDeadBeforeSend,
			"since_client_start", time.Since(c.born).Round(time.Millisecond).String(),
			"err", err,
		}
		// A cancellation is the caller's own doing, so it is not a device fault
		// and must not be logged as one -- but it is still worth seeing, because
		// waited_ms and ctx_dead_before_send together say whether this client's
		// own queueing caused it.
		if class == "canceled" {
			c.logger.CWarnw(ctx, "AL1342 request abandoned by its caller", fields...)
		} else {
			c.logger.CErrorw(ctx, "AL1342 request failed", fields...)
		}
	case firstOK:
		// Dates the moment the drive first became reachable, which is what
		// separates a network interface coming up late from a broken device.
		c.logger.CInfow(ctx, "AL1342 first successful request",
			"adr", t.adr,
			"after", time.Since(c.born).Round(time.Millisecond).String(),
			"rtt_ms", t.rtt.Seconds()*1000,
			"failed_attempts_before", c.stats.errors)
	case n%500 == 0:
		c.logger.CInfow(ctx, "AL1342 transport summary", "stats", c.Stats())
	}
}

type iotRequest struct {
	Code string         `json:"code"`
	CID  int            `json:"cid"`
	Adr  string         `json:"adr"`
	Data map[string]any `json:"data,omitempty"`
}

type iotResponse struct {
	CID   int    `json:"cid"`
	Code  int    `json:"code"`
	Error string `json:"error,omitempty"`
	Data  struct {
		Value string `json:"value"`
	} `json:"data"`
}

// do posts one IoT Core request, failing on any device code other than 200.
func (c *Client) do(ctx context.Context, adr string, data map[string]any) (*iotResponse, error) {
	body, err := json.Marshal(iotRequest{Code: "request", CID: 1, Adr: adr, Data: data})
	if err != nil {
		return nil, fmt.Errorf("encoding request for %s: %w", adr, err)
	}

	queued := c.inFlight.Add(1)
	defer c.inFlight.Add(-1)

	waitStart := time.Now()
	select {
	case c.sem <- struct{}{}:
	case <-ctx.Done():
		waited := time.Since(waitStart)
		c.record(ctx, trace{adr: adr, waited: waited, queued: queued, ctxDeadBeforeSend: true}, ctx.Err())
		return nil, fmt.Errorf("%s at %s: gave up after %s waiting behind another request: %w",
			adr, c.addr, waited.Round(time.Millisecond), ctx.Err())
	}
	t := trace{
		adr:               adr,
		waited:            time.Since(waitStart),
		queued:            queued,
		ctxDeadBeforeSend: ctx.Err() != nil,
	}
	defer func() { <-c.sem }()

	req, err := http.NewRequestWithContext(ctx, http.MethodPost, "http://"+c.addr, bytes.NewReader(body))
	if err != nil {
		return nil, fmt.Errorf("building request for %s: %w", adr, err)
	}
	req.Header.Set("Content-Type", "application/json")

	sendStart := time.Now()
	resp, err := c.http.Do(req)
	t.rtt = time.Since(sendStart)
	c.record(ctx, t, err)
	if err != nil {
		return nil, fmt.Errorf("%s at %s: %w", adr, c.addr, err)
	}
	defer resp.Body.Close()

	var out iotResponse
	if err := json.NewDecoder(resp.Body).Decode(&out); err != nil {
		return nil, fmt.Errorf("decoding response for %s: %w", adr, err)
	}
	if out.Code != 200 {
		return nil, fmt.Errorf("%s at %s: device code %d error %q", adr, c.addr, out.Code, out.Error)
	}
	return &out, nil
}

func (c *Client) portAdr(suffix string) string {
	return fmt.Sprintf("/iolinkmaster/port[%d]/iolinkdevice/%s", c.port, suffix)
}

func (c *Client) ReadPD(ctx context.Context) (uint16, error) {
	resp, err := c.do(ctx, c.portAdr("pdin/getdata"), nil)
	if err != nil {
		return 0, err
	}
	v, err := strconv.ParseUint(resp.Data.Value, 16, 16)
	if err != nil {
		return 0, fmt.Errorf("process data %q is not a 16-bit hex word: %w", resp.Data.Value, err)
	}
	return uint16(v), nil
}

func (c *Client) WritePD(ctx context.Context, word uint16) error {
	_, err := c.do(ctx, c.portAdr("pdout/setdata"),
		map[string]any{"newvalue": fmt.Sprintf("%04X", word)})
	return err
}

func (c *Client) ISDURead(ctx context.Context, index, subindex int) ([]byte, error) {
	resp, err := c.do(ctx, c.portAdr("iolreadacyclic"),
		map[string]any{"index": index, "subindex": subindex})
	if err != nil {
		return nil, err
	}
	b, err := hex.DecodeString(resp.Data.Value)
	if err != nil {
		return nil, fmt.Errorf("ISDU index %d returned %q, not hex: %w", index, resp.Data.Value, err)
	}
	return b, nil
}

// ReadPositionCounts returns the drive's actual position in its own units. The
// only source correct mid-stroke: the process data bits say nothing between
// their three taught points.
func (c *Client) ReadPositionCounts(ctx context.Context) (int32, error) {
	b, err := c.ISDURead(ctx, IsduPosActual, 0)
	if err != nil {
		return 0, err
	}
	if len(b) != 4 {
		return 0, fmt.Errorf("actual position returned %d bytes, want 4", len(b))
	}
	return int32(binary.BigEndian.Uint32(b)), nil
}

// ISDUWriteFloat32 writes a big-endian IEEE-754 float32 parameter.
func (c *Client) ISDUWriteFloat32(ctx context.Context, index, subindex int, v float32) error {
	_, err := c.do(ctx, c.portAdr("iolwriteacyclic"), map[string]any{
		"index":    index,
		"subindex": subindex,
		"value":    fmt.Sprintf("%08X", math.Float32bits(v)),
	})
	return err
}

// SystemCommand writes one of the package's named command bytes to index 2.
func (c *Client) SystemCommand(ctx context.Context, cmd byte) error {
	_, err := c.do(ctx, c.portAdr("iolwriteacyclic"), map[string]any{
		"index":    isduSystemCommand,
		"subindex": 0,
		"value":    fmt.Sprintf("%02X", cmd),
	})
	return err
}
