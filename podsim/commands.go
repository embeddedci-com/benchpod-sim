package podsim

import (
	"encoding/json"
	"fmt"
	"time"
)

// CmdLogEntry is one command the pod saw, for tests to assert on.
type CmdLogEntry struct {
	Seq    int            `json:"seq"`
	At     time.Time      `json:"at"`
	Source string         `json:"source"` // cloud | lan
	Cmd    string         `json:"cmd"`
	Args   map[string]any `json:"args,omitempty"`
	Status string         `json:"status"` // ok | error
	Error  string         `json:"error,omitempty"`
	Boot   string         `json:"boot_id"`
}

const (
	cmdLogMax = 500
	cmdInMax  = 1280 // BP_CLOUD_CMD_IN_MAX
	busyGrace = 30 * time.Millisecond
	linkDelay = 40 * time.Millisecond
)

func (p *Pod) logCmd(source string, cmd map[string]any, status, errMsg string) {
	name, _ := cmd["cmd"].(string)
	p.mu.Lock()
	defer p.mu.Unlock()
	p.cmdSeq++
	e := CmdLogEntry{Seq: p.cmdSeq, At: time.Now(), Source: source, Cmd: name, Args: cmd, Status: status, Error: errMsg, Boot: p.st.bootID}
	p.cmdLog = append(p.cmdLog, e)
	if len(p.cmdLog) > cmdLogMax {
		p.cmdLog = p.cmdLog[len(p.cmdLog)-cmdLogMax:]
	}
}

func (p *Pod) logStream(req map[string]any, cloud bool, err error) {
	source, status, msg := "lan", "ok", ""
	if cloud {
		source = "tunnel"
	}
	if err != nil {
		status, msg = "error", err.Error()
	}
	p.logCmd(source, req, status, msg)
}

// Commands returns the commands the pod saw, oldest first.
func (p *Pod) Commands() []CmdLogEntry {
	p.mu.Lock()
	defer p.mu.Unlock()
	return append([]CmdLogEntry(nil), p.cmdLog...)
}

// onCommandRequest is cl_handle_command_request: the command goes to the hw worker, which runs
// ONE cloud command at a time; while one is in flight the next gets "device busy" at once.
func (p *Pod) onCommandRequest(gen int, data []byte) {
	// The link's read loop must keep running while the grace below waits.
	var req struct {
		RequestID string         `json:"request_id"`
		Command   map[string]any `json:"command"`
	}
	if json.Unmarshal(data, &req) != nil || req.Command == nil {
		p.respondError(gen, req.RequestID, "missing command")
		return
	}
	var sized struct {
		Command json.RawMessage `json:"command"`
	}
	if _ = json.Unmarshal(data, &sized); len(sized.Command) >= cmdInMax {
		p.respondError(gen, req.RequestID, fmt.Sprintf("command too large for this device (limit %d bytes)", cmdInMax-1))
		return
	}
	// The cloud link is tens of ms long. A caller that moves on when the last reply lands is one
	// round trip behind the pod, and on loopback it would not be.
	time.Sleep(linkDelay)
	// A background push and a caller can still meet on a command that takes a few ms: too short
	// to see over a real link. A slow command is refused below, as on a real pod.
	for wait := busyGrace; wait > 0; wait -= time.Millisecond {
		p.mu.Lock()
		busy := p.st.busy
		p.mu.Unlock()
		if !busy {
			break
		}
		time.Sleep(time.Millisecond)
	}
	p.mu.Lock()
	if p.st.busy {
		p.mu.Unlock()
		p.logCmd("cloud", req.Command, "error", "device busy")
		p.respondError(gen, req.RequestID, "device busy")
		return
	}
	p.st.busy = true
	boot := p.st.bootID
	p.mu.Unlock()

	go func() {
		name, _ := req.Command["cmd"].(string)
		p.mu.Lock()
		delay := p.latency[name]
		p.mu.Unlock()
		if delay > 0 {
			time.Sleep(delay)
		}
		res, errMsg := p.execute(req.Command)
		status := "ok"
		if errMsg != "" {
			status = "error"
		}
		p.logCmd("cloud", req.Command, status, errMsg)
		// A reboot while the command ran: the reply would go to a link that no longer exists.
		p.mu.Lock()
		rebooted := p.st.bootID != boot
		if !rebooted {
			p.st.busy = false
		}
		p.mu.Unlock()
		if rebooted {
			return
		}
		if errMsg != "" {
			p.respondError(gen, req.RequestID, errMsg)
			return
		}
		_ = p.send(gen, map[string]any{
			"type": "command.response", "request_id": req.RequestID, "device_id": p.DeviceID(),
			"status": "ok", "data": res,
		})
	}()
}

func (p *Pod) respondError(gen int, requestID, msg string) {
	_ = p.send(gen, map[string]any{
		"type": "command.response", "request_id": requestID, "device_id": p.DeviceID(),
		"status": "error", "error": msg,
	})
}

// RunLAN runs a command as a LAN client would (the pod's own command port): the lease gate
// applies, and so does the one-command-at-a-time rule.
func (p *Pod) RunLAN(cmd map[string]any) (data any, errMsg string) {
	p.mu.Lock()
	busy := p.st.busy
	p.mu.Unlock()
	if busy {
		p.logCmd("lan", cmd, "error", "device busy")
		return nil, "device busy"
	}
	raw, errMsg := p.sim.Exec(cmd, false)
	status := "ok"
	if errMsg != "" {
		status = "error"
	}
	p.logCmd("lan", cmd, status, errMsg)
	return raw, errMsg
}

func (p *Pod) leaseHeldLocked() (bool, string) {
	l := p.st.lease
	if !l.Held || !time.Now().Before(l.Deadline) {
		return false, ""
	}
	return true, l.Holder
}

// onLeaseState records a lease.state frame; the pod also ends "held" by itself at the deadline.
func (p *Pod) onLeaseState(data []byte) {
	var m struct {
		Held       bool   `json:"held"`
		Holder     string `json:"holder"`
		ExpiresInS int    `json:"expires_in_s"`
	}
	if json.Unmarshal(data, &m) != nil {
		return
	}
	expires := time.Duration(m.ExpiresInS) * time.Second
	p.sim.SetLease(m.Held, m.Holder, expires)
	p.mu.Lock()
	defer p.mu.Unlock()
	if !m.Held {
		p.st.lease = lease{}
		return
	}
	p.st.lease = lease{Held: true, Holder: m.Holder, Deadline: time.Now().Add(expires)}
}

// execute runs one command from the cloud channel: the Sim is the firmware's command layer.
func (p *Pod) execute(cmd map[string]any) (any, string) {
	if name, _ := cmd["cmd"].(string); name == "lease_state" {
		p.mu.Lock()
		defer p.mu.Unlock()
		held, holder := p.leaseHeldLocked()
		out := map[string]any{"held": held}
		if held {
			out["holder"] = holder
			out["expires_in_s"] = int(time.Until(p.st.lease.Deadline).Seconds())
		}
		return out, ""
	}
	data, msg := p.sim.Exec(cmd, true)
	if msg != "" {
		return nil, msg
	}
	return data, ""
}

// hostCommand answers the verbs this side owns; the Sim calls it after its gates.
func (p *Pod) hostCommand(name string, req map[string]any, cloud bool) (any, string, bool) {
	if name == "cloud_status" {
		return map[string]any{"state": "connected", "device_id": p.DeviceID()}, "", true
	}
	return p.otaCommand(name, req, cloud)
}

// pushEfuseEvent is the async efuse.event the firmware sends when a rail changes.
func (p *Pod) pushEfuseEvent(n int, enabled, fault bool) {
	p.mu.Lock()
	gen := p.gen
	p.mu.Unlock()
	_ = p.send(gen, map[string]any{"type": "efuse.event", "efuse": n, "enabled": enabled, "fault": fault})
}

// sendEfuseSnapshot is the on-connect efuse.event pair: the tracked EN state of both rails.
func (p *Pod) sendEfuseSnapshot(gen int) {
	for n := 1; n <= 2; n++ {
		enabled, fault := p.sim.EfuseState(n)
		_ = p.send(gen, map[string]any{"type": "efuse.event", "efuse": n, "enabled": enabled, "fault": fault})
	}
}

// State is a snapshot of the pod for tests.
type State struct {
	DeviceID  string     `json:"device_id"`
	BootID    string     `json:"boot_id"`
	BootCount int        `json:"boot_count"`
	Firmware  string     `json:"firmware"`
	Connected bool       `json:"connected"`
	Busy      bool       `json:"busy"`
	LaMV      int        `json:"la_mv"`
	Efuse1    bool       `json:"efuse1"`
	Efuse2    bool       `json:"efuse2"`
	LeaseHeld bool       `json:"lease_held"`
	Holder    string     `json:"lease_holder,omitempty"`
	OTA       OTASummary `json:"ota"`
}

// Snapshot reads the pod's state.
func (p *Pod) Snapshot() State {
	mv, _, e1, e2 := p.sim.LAState()
	p.mu.Lock()
	defer p.mu.Unlock()
	held, holder := p.leaseHeldLocked()
	return State{
		DeviceID: p.deviceID, BootID: p.st.bootID, BootCount: p.st.bootCount, Firmware: p.st.firmware,
		Connected: p.connected, Busy: p.st.busy, LaMV: mv,
		Efuse1: e1, Efuse2: e2,
		LeaseHeld: held, Holder: holder, OTA: p.otaSummaryLocked(),
	}
}
