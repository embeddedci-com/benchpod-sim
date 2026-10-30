// Package podsim simulates a BenchPod for tests and local development: the cloud side of its
// firmware (stm32h563/src/cloud_client.c), with the pod's hardware and command layer in a
// benchsim.Sim behind it. It has an Ed25519 identity, logs in to the server's pod WebSocket exactly
// like the firmware (challenge, signed nonce, host-bound v2), announces its capabilities, runs the
// JSON commands the server and the web app send through the Sim, and carries tunnels as the Sim's
// LAN protocol.
//
// What it models on purpose, because the real pod does and the server's bugs live there:
//
//   - ONE cloud command runs at a time; a second gets the error "device busy" at once.
//   - A reboot is a new boot id, the LA I/O voltage unset again, target power off, the lease gone
//     and an OTA image staged in PSRAM lost, with the connection down for a while.
//   - A reconnect without a reboot (a dropped link) keeps all of that, like the firmware.
//   - While a cloud consumer holds the pod's lease (lease.state frames), LAN commands are limited
//     to light reads.
//
// It is dev/test only: nothing in the production image imports it.
package podsim

import (
	"bufio"
	"context"
	"crypto/ed25519"
	"crypto/rand"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net"
	"net/http"
	"net/url"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/embeddedci-com/benchpod-sim/benchsim"

	"github.com/coder/websocket"
)

// Config is how a simulated pod is set up.
type Config struct {
	// Server is the server's origin, e.g. http://127.0.0.1:8080 (the API is under /api).
	Server string
	// Name is the pod's name on the owner's account.
	Name string
	// Email and Password sign in the owner, for registering the pod.
	Email, Password string
	// Firmware is the version the pod announces.
	Firmware string
	// Board is the announced board (the firmware's BOARD_NAME, "stm32h563", by default).
	Board string
	// OTAVersion is the version the pod runs after a committed OTA (default Firmware + ".1").
	OTAVersion string
	// V1Auth makes the pod log in with the original nonce-only signature instead of host-bound v2.
	V1Auth bool
	// Logf receives the pod's log lines (default log.Printf).
	Logf func(format string, args ...any)
}

// Pod is one simulated BenchPod.
type Pod struct {
	cfg  Config
	priv ed25519.PrivateKey
	pub  ed25519.PublicKey
	http *http.Client

	mu       sync.Mutex
	deviceID string
	st       state
	// down: while set and in the future the pod does not connect (it is rebooting or offline).
	downUntil time.Time
	// cancelSession ends the connection of the current session (nil when not connected).
	cancelSession context.CancelFunc
	connected     bool
	gen           int // connection generation: replies to an older one are dropped
	conn          *websocket.Conn
	latency       map[string]time.Duration
	cmdLog        []CmdLogEntry
	cmdSeq        int
	tunnels       map[string]*tunnel
	sim           *benchsim.Sim
	// kick wakes the connect loop when the down time changes.
	kick chan struct{}
}

// state is what a reboot resets.
type state struct {
	bootID    string
	bootCount int
	firmware  string
	lease     lease
	ota       otaState
	busy      bool
}

type lease struct {
	Held     bool
	Holder   string
	Deadline time.Time
}

// New makes a pod with a fresh identity.
func New(cfg Config) (*Pod, error) {
	if cfg.Firmware == "" {
		cfg.Firmware = "3.8.0"
	}
	if cfg.Board == "" {
		cfg.Board = "stm32h563"
	}
	if cfg.OTAVersion == "" {
		cfg.OTAVersion = cfg.Firmware + ".1"
	}
	if cfg.Logf == nil {
		cfg.Logf = log.Printf
	}
	cfg.Server = strings.TrimRight(cfg.Server, "/")
	pub, priv, err := ed25519.GenerateKey(rand.Reader)
	if err != nil {
		return nil, err
	}
	p := &Pod{
		cfg:  cfg,
		priv: priv,
		pub:  pub,
		http: &http.Client{Timeout: 20 * time.Second},
		kick: make(chan struct{}, 1),
		latency: map[string]time.Duration{
			// target_status reads the eFuse power-good lines; the server asks it first on every
			// connect, which is what the la_voltage push used to lose the "device busy" race to.
			"target_status": 5 * time.Millisecond,
			"la_voltage":    5 * time.Millisecond,
		},
	}
	p.st = p.freshState(0)
	p.st.firmware = cfg.Firmware
	p.sim = benchsim.New(benchsim.WithFirmware(cfg.Firmware, benchsim.Hooks{
		Command:      p.hostCommand,
		Efuse:        p.pushEfuseEvent,
		CapsChanged:  p.resendCapabilities,
		Reboot:       func() { p.Reboot(time.Second) },
		ProxyChanged: func() { time.AfterFunc(300*time.Millisecond, func() { p.Drop(0) }) }, // after the reply went out
	}))
	return p, nil
}

func (p *Pod) logf(format string, args ...any) {
	p.cfg.Logf("podsim["+p.cfg.Name+"]: "+format, args...)
}

// freshState is the pod right after power-up: a new boot id, the LA bank unset (the firmware's
// safety default), target power off, no lease, nothing staged.
func (p *Pod) freshState(bootCount int) state {
	b := make([]byte, 8)
	_, _ = rand.Read(b)
	return state{bootID: hex.EncodeToString(b), bootCount: bootCount + 1, firmware: p.st.firmware}
}

// PublicKey is the pod's identity as the server stores it (base64url, no padding).
func (p *Pod) PublicKey() string { return base64.RawURLEncoding.EncodeToString(p.pub) }

// DeviceID is the server-assigned id, empty until Register.
func (p *Pod) DeviceID() string {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.deviceID
}

// apiURL joins a path onto the server's /api.
func (p *Pod) apiURL(path string) string { return p.cfg.Server + "/api" + path }

// Register signs the owner in and registers (or re-registers) the pod with its public key.
func (p *Pod) Register(ctx context.Context) error {
	token, err := p.login(ctx)
	if err != nil {
		return err
	}
	body, _ := json.Marshal(map[string]any{
		"name":       p.cfg.Name,
		"public_key": p.PublicKey(),
		"parameters": map[string]string{"kind": "podsim"},
	})
	req, _ := http.NewRequestWithContext(ctx, http.MethodPost, p.apiURL("/benchpod/devices"), strings.NewReader(string(body)))
	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Authorization", "Bearer "+token)
	resp, err := p.http.Do(req)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	raw, _ := io.ReadAll(resp.Body)
	if resp.StatusCode != http.StatusCreated && resp.StatusCode != http.StatusOK {
		return fmt.Errorf("register pod: %s: %s", resp.Status, strings.TrimSpace(string(raw)))
	}
	var out struct {
		ID string `json:"id"`
	}
	if err := json.Unmarshal(raw, &out); err != nil || out.ID == "" {
		return fmt.Errorf("register pod: no id in %s", raw)
	}
	p.mu.Lock()
	p.deviceID = out.ID
	p.mu.Unlock()
	p.logf("registered as %s", out.ID)
	return nil
}

// login returns a session token for the owner.
func (p *Pod) login(ctx context.Context) (string, error) {
	body, _ := json.Marshal(map[string]string{"email": p.cfg.Email, "password": p.cfg.Password})
	req, _ := http.NewRequestWithContext(ctx, http.MethodPost, p.apiURL("/auth/password/login"), strings.NewReader(string(body)))
	req.Header.Set("Content-Type", "application/json")
	resp, err := p.http.Do(req)
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()
	raw, _ := io.ReadAll(resp.Body)
	if resp.StatusCode != http.StatusOK {
		return "", fmt.Errorf("sign in %s: %s: %s", p.cfg.Email, resp.Status, strings.TrimSpace(string(raw)))
	}
	var out struct {
		Token string `json:"token"`
	}
	if err := json.Unmarshal(raw, &out); err != nil || out.Token == "" {
		return "", fmt.Errorf("sign in: no token in %s", raw)
	}
	return out.Token, nil
}

// Unregister deletes the pod from the owner's account (what "Delete" in the web app does) and
// disconnects it. Reregister brings it back as a new pod with a new identity.
func (p *Pod) Unregister(ctx context.Context) error {
	token, err := p.login(ctx)
	if err != nil {
		return err
	}
	id := p.DeviceID()
	req, _ := http.NewRequestWithContext(ctx, http.MethodDelete, p.apiURL("/benchpod/devices/"+id), nil)
	req.Header.Set("Authorization", "Bearer "+token)
	resp, err := p.http.Do(req)
	if err != nil {
		return err
	}
	resp.Body.Close()
	if resp.StatusCode >= 300 {
		return fmt.Errorf("delete pod: %s", resp.Status)
	}
	p.mu.Lock()
	p.deviceID = ""
	cancel := p.cancelSession
	p.mu.Unlock()
	if cancel != nil {
		cancel()
	}
	return nil
}

// Reregister gives the pod a fresh identity, registers it under name (when not empty) and lets
// the connect loop log in with it.
func (p *Pod) Reregister(ctx context.Context, name string) error {
	pub, priv, err := ed25519.GenerateKey(rand.Reader)
	if err != nil {
		return err
	}
	p.mu.Lock()
	p.pub, p.priv = pub, priv
	if name != "" {
		p.cfg.Name = name
	}
	p.downUntil = time.Time{}
	p.mu.Unlock()
	return p.Register(ctx)
}

// Run connects the pod and keeps it connected (reconnecting after drops and reboots) until ctx
// ends. Register first.
func (p *Pod) Run(ctx context.Context) error {
	if p.DeviceID() == "" {
		return fmt.Errorf("podsim: not registered")
	}
	backoff := time.Second
	for ctx.Err() == nil {
		if p.DeviceID() == "" { // unregistered: wait for Reregister
			select {
			case <-time.After(200 * time.Millisecond):
			case <-ctx.Done():
				return nil
			}
			continue
		}
		if wait := p.downFor(); wait > 0 {
			select {
			case <-time.After(wait):
			case <-p.kick:
			case <-ctx.Done():
				return nil
			}
			continue
		}
		p.mu.Lock()
		gen := p.gen
		p.mu.Unlock()
		err := p.session(ctx)
		if ctx.Err() != nil {
			return nil
		}
		p.logf("session ended: %v", err)
		// A session that got connected retries at once; failed connects back off like the firmware.
		p.mu.Lock()
		ran := p.gen != gen
		p.mu.Unlock()
		wait := 300 * time.Millisecond
		if ran {
			backoff = time.Second
		} else {
			wait = backoff
			backoff = min(backoff*2, 30*time.Second)
		}
		select {
		case <-time.After(wait):
		case <-p.kick:
		case <-ctx.Done():
			return nil
		}
	}
	return nil
}

func (p *Pod) downFor() time.Duration {
	p.mu.Lock()
	defer p.mu.Unlock()
	if d := time.Until(p.downUntil); d > 0 {
		return d
	}
	return 0
}

// session is one connection: challenge, signed login, capabilities, then the read loop.
func (p *Pod) session(ctx context.Context) error {
	id := p.DeviceID()
	nonce, err := p.challenge(ctx, id)
	if err != nil {
		return err
	}
	nonceBytes, err := base64.RawURLEncoding.DecodeString(nonce)
	if err != nil {
		return fmt.Errorf("bad nonce: %w", err)
	}
	u, err := url.Parse(p.cfg.Server)
	if err != nil {
		return err
	}
	var msg []byte
	q := url.Values{}
	q.Set("device_id", id)
	q.Set("nonce", nonce)
	if p.cfg.V1Auth {
		msg = append([]byte("benchpod-ws-auth:v1\x00"), nonceBytes...)
	} else {
		// The host the pod connected to, as it names it for SNI and the certificate check.
		msg = append([]byte("benchpod-ws-auth:v2\x00"+strings.ToLower(u.Hostname())+"\x00"), nonceBytes...)
		q.Set("auth", "v2")
	}
	q.Set("signature", base64.RawURLEncoding.EncodeToString(ed25519.Sign(p.priv, msg)))
	scheme := "ws"
	if u.Scheme == "https" {
		scheme = "wss"
	}
	wsURL := fmt.Sprintf("%s://%s/api/benchpod/ws?%s", scheme, u.Host, q.Encode())

	sctx, cancel := context.WithCancel(ctx)
	defer cancel()
	dctx, dcancel := context.WithTimeout(sctx, 15*time.Second)
	conn, _, err := websocket.Dial(dctx, wsURL, &websocket.DialOptions{HTTPClient: p.linkClient()})
	dcancel()
	if err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	defer conn.CloseNow()
	conn.SetReadLimit(8 << 20)
	defer p.closeTunnels()

	p.mu.Lock()
	p.gen++
	gen := p.gen
	p.conn = conn
	p.connected = true
	p.cancelSession = cancel
	p.st.busy = false // a command in flight died with the link
	p.mu.Unlock()
	defer func() {
		p.mu.Lock()
		if p.gen == gen {
			p.connected = false
			p.conn = nil
			p.cancelSession = nil
		}
		p.mu.Unlock()
	}()

	if err := p.send(gen, p.capabilities()); err != nil {
		return err
	}
	p.sendEfuseSnapshot(gen)
	p.logf("connected (boot %s)", p.bootID())
	go p.otaStatusTicker(sctx, gen)

	for {
		_, data, err := conn.Read(sctx)
		if err != nil {
			return err
		}
		p.onFrame(gen, data)
	}
}

// linkClient is how the pod reaches the server: through the stored HTTP proxy when there is one
// (altcp_bp_proxy.c: a CONNECT, with Proxy-Authorization when it has credentials).
func (p *Pod) linkClient() *http.Client {
	host, port, user, pass := p.sim.ProxySetting()
	if host == "" {
		return p.http
	}
	addr := net.JoinHostPort(host, strconv.Itoa(port))
	return &http.Client{Timeout: 20 * time.Second, Transport: &http.Transport{
		DisableKeepAlives: true,
		DialContext: func(ctx context.Context, _, target string) (net.Conn, error) {
			c, err := (&net.Dialer{Timeout: 5 * time.Second}).DialContext(ctx, "tcp", addr)
			if err != nil {
				return nil, err
			}
			auth := ""
			if user != "" {
				auth = "Proxy-Authorization: Basic " + base64.StdEncoding.EncodeToString([]byte(user+":"+pass)) + "\r\n"
			}
			_ = c.SetDeadline(time.Now().Add(10 * time.Second))
			if _, err := fmt.Fprintf(c, "CONNECT %s HTTP/1.1\r\nHost: %s\r\n%s\r\n", target, target, auth); err != nil {
				_ = c.Close()
				return nil, err
			}
			br := bufio.NewReader(c)
			resp, err := http.ReadResponse(br, &http.Request{Method: http.MethodConnect})
			if err != nil {
				_ = c.Close()
				return nil, fmt.Errorf("proxy: %w", err)
			}
			if resp.StatusCode != http.StatusOK {
				_ = c.Close()
				return nil, fmt.Errorf("proxy: %s", resp.Status)
			}
			_ = c.SetDeadline(time.Time{})
			if br.Buffered() > 0 {
				return nil, fmt.Errorf("proxy: sent data before the tunnel")
			}
			return c, nil
		},
	}}
}

func (p *Pod) bootID() string {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.st.bootID
}

// challenge asks the server for a single-use nonce (unauthenticated, like the firmware).
func (p *Pod) challenge(ctx context.Context, id string) (string, error) {
	req, _ := http.NewRequestWithContext(ctx, http.MethodPost, p.apiURL("/benchpod/devices/"+id+"/challenge"), nil)
	resp, err := p.linkClient().Do(req)
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()
	raw, _ := io.ReadAll(resp.Body)
	if resp.StatusCode != http.StatusCreated {
		return "", fmt.Errorf("challenge: %s: %s", resp.Status, strings.TrimSpace(string(raw)))
	}
	var out struct {
		Nonce string `json:"nonce"`
	}
	if err := json.Unmarshal(raw, &out); err != nil || out.Nonce == "" {
		return "", fmt.Errorf("challenge: no nonce in %s", raw)
	}
	return out.Nonce, nil
}

// capabilities is the frame the firmware sends right after the handshake: the Sim's features and
// the link's own fields.
func (p *Pod) capabilities() map[string]any {
	out := p.sim.Capabilities()
	p.mu.Lock()
	defer p.mu.Unlock()
	out["type"] = "capabilities"
	out["device_id"] = p.deviceID
	out["firmware_version"] = p.st.firmware
	out["board"] = p.cfg.Board
	out["boot_id"] = p.st.bootID
	out["unclean_resets"] = 0
	out["safe_mode"] = false
	out["safe_reason"] = ""
	out["reset_cause"] = "power-on"
	out["last_crash"] = "none"
	out["ws_auth_v2"] = !p.cfg.V1Auth
	return out
}

// resendCapabilities is cloud_client_request_caps_resend.
func (p *Pod) resendCapabilities() {
	p.mu.Lock()
	gen := p.gen
	p.mu.Unlock()
	_ = p.send(gen, p.capabilities())
}

// send writes one JSON frame on the connection generation gen; a frame for an older connection is
// dropped, like the firmware dropping the reply to a command that outlived its link.
func (p *Pod) send(gen int, v any) error {
	p.mu.Lock()
	conn, cur := p.conn, p.gen
	p.mu.Unlock()
	if conn == nil || cur != gen {
		return fmt.Errorf("link not connected")
	}
	raw, err := json.Marshal(v)
	if err != nil {
		return err
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	return conn.Write(ctx, websocket.MessageText, raw)
}

// onFrame is the firmware's frame router (cl_process_ws_frames).
func (p *Pod) onFrame(gen int, data []byte) {
	var env struct {
		Type string `json:"type"`
	}
	if json.Unmarshal(data, &env) != nil {
		return
	}
	switch env.Type {
	case "command.request":
		go p.onCommandRequest(gen, data)
	case "lease.state":
		p.onLeaseState(data)
	case "ota.begin", "ota.data", "ota.end", "ota.abort", "ota.commit":
		p.onOTAFrame(gen, env.Type, data)
	case "tunnel.open":
		p.onTunnelOpen(gen, data)
	case "tunnel.data":
		p.onTunnelData(gen, data)
	case "tunnel.close":
		p.onTunnelClose(data)
	case "ping":
		_ = p.send(gen, map[string]any{"type": "pong"})
	}
}

// Reboot power-cycles the pod: the connection drops, the pod comes back after down with a new
// boot id and everything the firmware loses on a reset lost.
func (p *Pod) Reboot(down time.Duration) {
	p.mu.Lock()
	p.st = p.freshState(p.st.bootCount)
	p.downUntil = time.Now().Add(down)
	cancel := p.cancelSession
	p.mu.Unlock()
	p.logf("reboot (down %s)", down)
	p.closeTunnels()
	p.sim.Reboot()
	if cancel != nil {
		cancel()
	}
	select {
	case p.kick <- struct{}{}:
	default:
	}
}

// Drop ends the connection without a reboot (the network went away); the pod reconnects after
// down with its boot id and state intact.
func (p *Pod) Drop(down time.Duration) {
	p.mu.Lock()
	p.downUntil = time.Now().Add(down)
	cancel := p.cancelSession
	p.mu.Unlock()
	p.logf("link dropped (down %s)", down)
	if cancel != nil {
		cancel()
	}
	select {
	case p.kick <- struct{}{}:
	default:
	}
}

// SetLatency sets how long the pod takes to run cmd (0 removes the delay).
func (p *Pod) SetLatency(cmd string, d time.Duration) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if d <= 0 {
		delete(p.latency, cmd)
		return
	}
	p.latency[cmd] = d
}

// ServeLAN answers the pod's LAN API on ln until ln closes (the firmware's :8080).
func (p *Pod) ServeLAN(ln net.Listener) error { return p.sim.Serve(ln) }

// Close ends the simulated hardware and every connection to it.
func (p *Pod) Close() { _ = p.sim.Close() }
