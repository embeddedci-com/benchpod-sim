package podsim

import (
	"encoding/base64"
	"encoding/json"
	"net"
	"strconv"
	"sync"
	"time"
)

// A cloud tunnel is a byte stream the server opens to the pod (tunnel.open); the firmware runs its
// per-connection protocol on it, as for a LAN client: JSON lines, captures, DAC uploads, the UART
// proxy. So a tunnel here is a pipe into the Sim, with the frames on the WebSocket around it.

const (
	tunnelChunk       = 1408 // the most raw bytes the server sends or accepts per tunnel.data frame
	tunnelBytesPerSec = 1 << 20
)

type tunnel struct {
	id     string
	conn   net.Conn
	mu     sync.Mutex
	cond   *sync.Cond
	in     [][]byte // bytes from the server, in order, until the Sim reads them
	closed bool     // the server closed it
}

func (p *Pod) onTunnelOpen(gen int, data []byte) {
	var m struct {
		TunnelID string `json:"tunnel_id"`
		MaxTier  any    `json:"max_tier"`
	}
	_ = json.Unmarshal(data, &m)
	maxTier := 3
	switch v := m.MaxTier.(type) {
	case float64:
		maxTier = int(v)
	case string:
		if n, err := strconv.Atoi(v); err == nil {
			maxTier = n
		}
	}
	if maxTier < 0 || maxTier > 3 {
		maxTier = 3
	}
	here, there := net.Pipe()
	t := &tunnel{id: m.TunnelID, conn: here}
	t.cond = sync.NewCond(&t.mu)
	p.mu.Lock()
	if p.tunnels == nil {
		p.tunnels = map[string]*tunnel{}
	}
	old := p.tunnels[m.TunnelID]
	p.tunnels[m.TunnelID] = t
	p.mu.Unlock()
	if old != nil {
		old.shut()
	}
	go p.sim.ServeTunnel(there, maxTier)
	go t.feed()
	go p.tunnelOut(gen, t)
}

// shut ends a tunnel from this side: the Sim sees the connection close.
func (t *tunnel) shut() {
	t.mu.Lock()
	t.closed = true
	t.cond.Broadcast()
	t.mu.Unlock()
	_ = t.conn.Close()
}

// feed hands the server's bytes to the Sim in order without holding up the link's read loop.
func (t *tunnel) feed() {
	for {
		t.mu.Lock()
		for len(t.in) == 0 && !t.closed {
			t.cond.Wait()
		}
		if t.closed {
			t.mu.Unlock()
			return
		}
		b := t.in[0]
		t.in = t.in[1:]
		t.mu.Unlock()
		if _, err := t.conn.Write(b); err != nil {
			return
		}
	}
}

// tunnelOut sends what the Sim writes as tunnel.data frames; a connection the pod ended is a
// tunnel.close.
func (p *Pod) tunnelOut(gen int, t *tunnel) {
	buf := make([]byte, tunnelChunk)
	next := time.Now()
	for {
		n, err := t.conn.Read(buf)
		if n > 0 {
			// The cloud link is not loopback: a long read-back takes time, and a link drop can hit it.
			if now := time.Now(); next.Before(now) {
				next = now
			}
			next = next.Add(time.Duration(n) * time.Second / tunnelBytesPerSec)
			time.Sleep(time.Until(next))
			_ = p.send(gen, map[string]any{"type": "tunnel.data", "tunnel_id": t.id, "data_b64": base64.RawURLEncoding.EncodeToString(buf[:n])})
		}
		if err != nil {
			break
		}
	}
	t.mu.Lock()
	byServer := t.closed
	t.mu.Unlock()
	p.mu.Lock()
	if p.tunnels[t.id] == t {
		delete(p.tunnels, t.id)
	}
	p.mu.Unlock()
	t.shut()
	if !byServer {
		_ = p.send(gen, map[string]any{"type": "tunnel.close", "tunnel_id": t.id})
	}
}

func (p *Pod) onTunnelClose(data []byte) {
	var m struct {
		TunnelID string `json:"tunnel_id"`
	}
	_ = json.Unmarshal(data, &m)
	p.mu.Lock()
	t := p.tunnels[m.TunnelID]
	delete(p.tunnels, m.TunnelID)
	p.mu.Unlock()
	if t != nil {
		t.shut()
	}
}

func (p *Pod) onTunnelData(gen int, data []byte) {
	var m struct {
		TunnelID string `json:"tunnel_id"`
		DataB64  string `json:"data_b64"`
	}
	if json.Unmarshal(data, &m) != nil {
		return
	}
	raw, err := base64.RawURLEncoding.DecodeString(m.DataB64)
	if err != nil {
		return
	}
	p.mu.Lock()
	t := p.tunnels[m.TunnelID]
	p.mu.Unlock()
	if t == nil {
		return
	}
	t.mu.Lock()
	t.in = append(t.in, raw)
	t.cond.Broadcast()
	t.mu.Unlock()
}

// closeTunnels drops every tunnel: the link they rode on is gone.
func (p *Pod) closeTunnels() {
	p.mu.Lock()
	ts := p.tunnels
	p.tunnels = nil
	p.mu.Unlock()
	for _, t := range ts {
		t.shut()
	}
}
