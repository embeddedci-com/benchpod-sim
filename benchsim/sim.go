// Package benchsim is a simulated EmbeddedCI BenchPod for the public website demo: it speaks the
// pod's LAN protocol (one TCP connection per command, newline-terminated JSON, chunked replies,
// the raw UART proxy) faithfully enough that the unmodified BenchPod MCP server and Python SDK
// drive it as if it were a pod on the LAN.
//
// The pod is wired to a simulated DUT: the NUCLEO-F446RE "process node" thermostat of
// examples/process-node-stm32 (dut*.go). Its firmware is modelled at the level the pod can
// observe: the UART console (boot log, every console command), the BMP280 it polls over I2C
// (answered by the pod's emulated sensor, sensors.go), the analog input and heater output, the
// alarm and strobe pins, and its supply current. Every observable line keeps a short, time-stamped
// history (signals.go), so an LA capture or a UART decode shows what the DUT actually did.
//
// One Sim per visitor: state is independent, every method is safe for concurrent use, and memory
// and goroutines are bounded (capture sizes, histories and connections are capped).
package benchsim

import (
	"bufio"
	"encoding/json"
	"errors"
	"io"
	"net"
	"sync"
	"time"
)

const (
	// maxConns caps the connections one Sim serves at once (a UART session, a capture and a
	// few short commands is the most a client needs); the rest are refused with "busy".
	maxConns = 12
	// maxRequestBytes caps one request line; the pod's own buffer is smaller.
	maxRequestBytes = 64 << 10
	// requestReadTimeout is how long a client gets to send its request line.
	requestReadTimeout = 15 * time.Second
	// logMax caps Log().
	logMax = 1000
)

// LogEntry is one command the simulated pod received, for tests and abuse review.
type LogEntry struct {
	Time   time.Time      `json:"time"`
	Cmd    string         `json:"cmd"`
	Args   map[string]any `json:"args,omitempty"`
	Status string         `json:"status"` // ok | error
	Error  string         `json:"error,omitempty"`
}

// Option configures a Sim.
type Option func(*Sim)

// WithLaVoltage starts the pod with its LA I/O bank already selected (1800 or 3300 mV), as a
// bench whose host set it at power-up would be. Without it the bank is unset, as on a real pod
// after boot, and the pod refuses LA operations until la_voltage is sent.
func WithLaVoltage(mv int) Option {
	return func(s *Sim) {
		if mv == 1800 || mv == 3300 {
			s.pod.laMV = mv
		}
	}
}

// WithDUTPowered starts with the DUT's rail on and the node already booted (a bench left
// running), instead of the default cold bench with the DUT off.
func WithDUTPowered() Option {
	return func(s *Sim) { s.startPowered = true }
}

// Hooks connect a Sim running the full firmware command layer to the cloud-link side of the pod.
type Hooks struct {
	// Command answers the verbs the link side owns (OTA, cloud_status); ok false falls through.
	Command func(name string, req map[string]any, cloud bool) (data any, errMsg string, ok bool)
	// Efuse reports a rail change as it happens (the firmware's efuse.event).
	Efuse func(n int, enabled, fault bool)
	// CapsChanged asks for a new capabilities frame (cloud_client_request_caps_resend).
	CapsChanged func()
	// Logged reports a command that came over a stream connection: a tunnel, or the LAN port.
	Logged func(req map[string]any, cloud bool, err error)
	// Reboot resets the pod from a command (psram_recover).
	Reboot func()
	// ProxyChanged reports a cloud_proxy set or clear: the link reconnects through it.
	ProxyChanged func()
}

// WithFirmware runs the firmware's whole command layer (tiers, gates, the lease, every verb)
// instead of the demo's narrower set: a pod for testing what talks to pods, not a public demo.
func WithFirmware(version string, h Hooks) Option {
	return func(s *Sim) {
		s.firmware = true
		s.fwVersion = version
		s.hooks = h
	}
}

// Sim is one simulated BenchPod with its DUT.
type Sim struct {
	mu    sync.Mutex
	epoch time.Time
	rng   uint64

	pod  podState
	dut  dut
	sig  signals
	logs []LogEntry

	startPowered bool
	firmware     bool
	fwVersion    string
	hooks        Hooks
	flash        podFlash
	cmdConn      *conn

	conns     map[net.Conn]struct{}
	listeners map[net.Listener]struct{}
	done      chan struct{}
	closeOnce sync.Once
	wg        sync.WaitGroup
}

// New returns a simulated pod wired to a powered-off process node.
func New(opts ...Option) *Sim {
	s := &Sim{
		epoch:     time.Now(),
		rng:       0x9E3779B97F4A7C15,
		conns:     map[net.Conn]struct{}{},
		listeners: map[net.Listener]struct{}{},
		done:      make(chan struct{}),
	}
	s.pod.init()
	s.flash = podFlash{lanPolicy: "open", sigPolicy: "audit"}
	s.cmdConn = &conn{}
	s.dut.init()
	for _, o := range opts {
		o(s)
	}
	s.sig.pushBase(0, s.baseWord())
	if s.startPowered {
		// Boot "in the past" so the node is at its prompt when the first visitor command lands.
		s.mu.Lock()
		s.pod.efuse[1].enabled = true
		s.dutRailOn(0)
		s.mu.Unlock()
		s.epoch = s.epoch.Add(-3 * time.Second)
		s.mu.Lock()
		s.advance(s.now())
		s.mu.Unlock()
	}
	return s
}

// now is the simulation time: nanoseconds since the Sim started, on the monotonic clock.
func (s *Sim) now() int64 { return int64(time.Since(s.epoch)) }

// wall turns a simulation time back into a wall-clock time.
func (s *Sim) wall(t int64) time.Time { return s.epoch.Add(time.Duration(t)) }

// Serve accepts pod connections on ln until ln is closed (or the Sim is). Each connection
// carries one command, like the firmware's LAN API.
func (s *Sim) Serve(ln net.Listener) error {
	s.mu.Lock()
	select {
	case <-s.done:
		s.mu.Unlock()
		return net.ErrClosed
	default:
	}
	s.listeners[ln] = struct{}{}
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		delete(s.listeners, ln)
		s.mu.Unlock()
	}()
	for {
		c, err := ln.Accept()
		if err != nil {
			select {
			case <-s.done:
				return nil
			default:
			}
			if errors.Is(err, net.ErrClosed) {
				return nil
			}
			var ne net.Error
			if errors.As(err, &ne) && ne.Timeout() {
				continue
			}
			return err
		}
		s.mu.Lock()
		full := len(s.conns) >= maxConns
		if !full {
			s.conns[c] = struct{}{}
			s.wg.Add(1)
		}
		s.mu.Unlock()
		if full {
			_, _ = c.Write(encodeLine(map[string]any{"status": "error", "message": "busy: too many connections"}))
			_ = c.Close()
			continue
		}
		if s.firmware {
			if s.lanOff() {
				s.mu.Lock()
				delete(s.conns, c)
				s.mu.Unlock()
				s.wg.Done()
				_ = c.Close()
				continue
			}
			go s.serveStream(c, false, -1)
			continue
		}
		go s.serveConn(c)
	}
}

// Close stops every connection and listener. A closed Sim refuses new connections.
func (s *Sim) Close() error {
	s.closeOnce.Do(func() {
		close(s.done)
		s.mu.Lock()
		for ln := range s.listeners {
			_ = ln.Close()
		}
		for c := range s.conns {
			_ = c.Close()
		}
		s.mu.Unlock()
	})
	s.wg.Wait()
	return nil
}

// Log returns the commands the pod received, oldest first (at most the last 1000).
func (s *Sim) Log() []LogEntry {
	s.mu.Lock()
	defer s.mu.Unlock()
	return append([]LogEntry(nil), s.logs...)
}

func (s *Sim) logCmd(req request, err error) {
	e := LogEntry{Time: time.Now(), Cmd: req.str("cmd"), Args: map[string]any(req), Status: "ok"}
	if err != nil {
		e.Status, e.Error = "error", err.Error()
	}
	s.mu.Lock()
	s.logs = append(s.logs, e)
	if len(s.logs) > logMax {
		s.logs = append([]LogEntry(nil), s.logs[len(s.logs)-logMax:]...)
	}
	s.mu.Unlock()
}

func (s *Sim) serveConn(c net.Conn) {
	defer s.wg.Done()
	defer func() {
		s.mu.Lock()
		delete(s.conns, c)
		s.mu.Unlock()
		_ = c.Close()
	}()
	if tc, ok := c.(*net.TCPConn); ok {
		_ = tc.SetNoDelay(true)
	}
	_ = c.SetReadDeadline(time.Now().Add(requestReadTimeout))
	br := bufio.NewReaderSize(c, 4096)
	line, err := readLine(br, maxRequestBytes)
	if err != nil || len(line) == 0 {
		return
	}
	var req request
	if json.Unmarshal(line, &req) != nil || req == nil {
		_, _ = c.Write(encodeLine(errReply("invalid json")))
		return
	}
	_ = c.SetReadDeadline(time.Time{})
	s.dispatch(&conn{Conn: c, br: br, sim: s}, req)
}

// readLine reads one newline-terminated line of at most max bytes.
func readLine(br *bufio.Reader, max int) ([]byte, error) {
	var out []byte
	for {
		chunk, err := br.ReadSlice('\n')
		out = append(out, chunk...)
		if len(out) > max {
			return nil, errors.New("request too long")
		}
		if err == nil {
			return trimEOL(out), nil
		}
		if !errors.Is(err, bufio.ErrBufferFull) {
			if len(out) > 0 && errors.Is(err, io.EOF) {
				return trimEOL(out), nil
			}
			return nil, err
		}
	}
}

func trimEOL(b []byte) []byte {
	for len(b) > 0 && (b[len(b)-1] == '\n' || b[len(b)-1] == '\r') {
		b = b[:len(b)-1]
	}
	return b
}

// sleepUntil waits until simulation time t, or returns false when the Sim closes first.
func (s *Sim) sleepUntil(t int64) bool {
	d := time.Duration(t - s.now())
	if d <= 0 {
		return true
	}
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-timer.C:
		return true
	case <-s.done:
		return false
	}
}

// rand returns a deterministic pseudo-random uint64 (xorshift), for measurement noise.
// Callers hold s.mu.
func (s *Sim) rand() uint64 {
	x := s.rng
	x ^= x << 13
	x ^= x >> 7
	x ^= x << 17
	s.rng = x
	return x
}

// noise returns a value in [-amp, amp] (roughly triangular, like summed thermal noise).
func (s *Sim) noise(amp float64) float64 {
	a := float64(s.rand()%10001)/10000 - 0.5
	b := float64(s.rand()%10001)/10000 - 0.5
	return (a + b) * amp
}
