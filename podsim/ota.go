package podsim

import (
	"context"
	"crypto/sha256"
	"crypto/x509"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"encoding/pem"
	"strings"
	"time"
)

// The firmware's OTA session (ota.c) as the server sees it: ota.begin opens a staging buffer in
// PSRAM, ota.data frames land at their offset, ota.end checks the SHA-256 and the pod reports
// "verified", ota.commit installs it (a firmware image reboots the pod) and ota.abort drops it.
// The staged image lives in PSRAM: a reconnect keeps it, a reboot loses it.

type otaState struct {
	State    string // idle | receiving | verified | error | installed
	Size     int
	SHA      string
	Target   string // "" = firmware, else a blob slot
	Buf      []byte
	Received int // contiguous bytes from offset 0
	Err      string
	Owner    string
	active   time.Time // the last thing the owner did
	lastSent int       // received count of the last ota.status frame
}

// setOTA updates what the Sim's capture gate knows about the session (OTA owns the PSRAM bus).
func (p *Pod) otaGate() {
	p.mu.Lock()
	st := p.st.ota.State
	p.mu.Unlock()
	p.sim.SetOTABusy(st == "receiving" || st == "verified")
}

// OTASummary is the OTA session as tests see it.
type OTASummary struct {
	State    string `json:"state"`
	Size     int    `json:"size"`
	Received int    `json:"received"`
	Error    string `json:"error,omitempty"`
}

func (p *Pod) otaSummaryLocked() OTASummary {
	o := p.st.ota
	s := o.State
	if s == "" {
		s = "idle"
	}
	return OTASummary{State: s, Size: o.Size, Received: o.Received, Error: o.Err}
}

const (
	otaStatusEvery = 8192
	otaStatusTick  = 500 * time.Millisecond
)

func (p *Pod) onOTAFrame(gen int, typ string, data []byte) {
	defer p.otaGate()
	switch typ {
	case "ota.begin":
		var m struct {
			Size   int    `json:"size"`
			SHA256 string `json:"sha256"`
			Target string `json:"target"`
		}
		_ = json.Unmarshal(data, &m)
		p.mu.Lock()
		if p.st.ota.lanHolds() {
			p.mu.Unlock()
			p.sendOTARefusal(gen)
			return
		}
		if m.Size <= 0 || len(m.SHA256) != 64 {
			p.st.ota = otaState{State: "error", Err: "bad size / bad sha256"}
		} else {
			p.st.ota = otaState{State: "receiving", Size: m.Size, SHA: strings.ToLower(m.SHA256), Target: m.Target, Buf: make([]byte, m.Size)}
			p.st.ota.Owner = "cloud"
		}
		p.mu.Unlock()
		p.sendOTAStatus(gen)
	case "ota.data":
		var m struct {
			Offset  int    `json:"offset"`
			DataB64 string `json:"data_b64"`
		}
		_ = json.Unmarshal(data, &m)
		chunk, err := base64.RawURLEncoding.DecodeString(m.DataB64)
		p.mu.Lock()
		o := &p.st.ota
		if o.State != "receiving" || o.lanHolds() {
			p.mu.Unlock()
			return
		}
		if err != nil || m.Offset < 0 || m.Offset+len(chunk) > o.Size {
			o.State, o.Err = "error", "bad ota.data frame"
			p.mu.Unlock()
			p.sendOTAStatus(gen)
			return
		}
		copy(o.Buf[m.Offset:], chunk)
		// The high-water mark is contiguous from byte 0, like the firmware's: a chunk past a gap
		// does not advance it.
		if m.Offset <= o.Received && m.Offset+len(chunk) > o.Received {
			o.Received = m.Offset + len(chunk)
		}
		report := o.Received-o.lastSent >= otaStatusEvery || o.Received == o.Size
		p.mu.Unlock()
		if report {
			p.sendOTAStatus(gen)
		}
	case "ota.end":
		p.mu.Lock()
		o := &p.st.ota
		if o.lanHolds() {
			p.mu.Unlock()
			p.sendOTARefusal(gen)
			return
		}
		switch {
		case o.State != "receiving":
			o.State, o.Err = "error", "no image is being received"
		case o.Received != o.Size:
			o.State, o.Err = "error", "incomplete image"
		default:
			sum := sha256.Sum256(o.Buf)
			if hex.EncodeToString(sum[:]) != o.SHA {
				o.State, o.Err = "error", "sha256 mismatch"
			} else {
				o.State, o.Err = "verified", ""
			}
		}
		p.mu.Unlock()
		p.sendOTAStatus(gen)
	case "ota.abort":
		p.mu.Lock()
		if p.st.ota.lanHolds() {
			p.mu.Unlock()
			p.sendOTARefusal(gen)
			return
		}
		p.st.ota = otaState{State: "idle"}
		p.mu.Unlock()
		p.sendOTAStatus(gen)
	case "ota.commit":
		p.mu.Lock()
		o := &p.st.ota
		if o.lanHolds() {
			p.mu.Unlock()
			p.sendOTARefusal(gen)
			return
		}
		if o.State != "verified" {
			o.State, o.Err = "error", "nothing verified to install"
			p.mu.Unlock()
			p.sendOTAStatus(gen)
			return
		}
		if o.Target != "" { // a blob goes into its W25Q slot; the pod keeps running
			o.State = "installed"
			p.installBlob(o)
			p.mu.Unlock()
			p.sendOTAStatus(gen)
			return
		}
		p.mu.Unlock()
		// A firmware image: the pod erases and rewrites its flash and reboots into it.
		go func() {
			time.Sleep(200 * time.Millisecond)
			p.mu.Lock()
			p.st.firmware = p.cfg.OTAVersion
			p.mu.Unlock()
			p.sim.SetFirmwareVersion(p.cfg.OTAVersion)
			p.Reboot(time.Second)
		}()
	}
}

const otaAbandonAfter = 60 * time.Second

// lanHolds: a LAN client's live session, which the cloud may not touch until it is abandoned.
func (o *otaState) lanHolds() bool {
	return o.Owner == "lan" && (o.State == "receiving" || o.State == "verified") && time.Since(o.active) < otaAbandonAfter
}

const otaBusyLAN = "busy: firmware update from the LAN is in progress"

// sendOTARefusal is what the cloud hears when another transport holds the session.
func (p *Pod) sendOTARefusal(gen int) {
	_ = p.send(gen, map[string]any{"type": "ota.status", "device_id": p.DeviceID(), "state": "error",
		"received": 0, "size": 0, "sig": "", "sig_key": "", "error": otaBusyLAN})
}

// sendOTAStatus is cl_ota_status_poll's frame.
func (p *Pod) sendOTAStatus(gen int) {
	p.mu.Lock()
	o := &p.st.ota
	o.lastSent = o.Received
	state := o.State
	if state == "" {
		state = "idle"
	}
	frame := map[string]any{
		"type": "ota.status", "device_id": p.deviceID, "state": state,
		"received": o.Received, "size": o.Size, "sig": "", "sig_key": "", "error": o.Err,
	}
	p.mu.Unlock()
	_ = p.send(gen, frame)
}

// otaStatusTicker reports progress at least every 500 ms while receiving: the server's paced push
// waits for these acknowledgements.
func (p *Pod) otaStatusTicker(ctx context.Context, gen int) {
	t := time.NewTicker(otaStatusTick)
	defer t.Stop()
	for {
		select {
		case <-t.C:
			p.mu.Lock()
			receiving := p.st.ota.State == "receiving"
			p.mu.Unlock()
			if receiving {
				p.sendOTAStatus(gen)
			}
		case <-ctx.Done():
			return
		}
	}
}

// otaCommand is command_handler_ota.c: the same session driven by JSON commands, for LAN clients
// and for reads through the cloud channel.
func (p *Pod) otaCommand(name string, req map[string]any, cloud bool) (any, string, bool) {
	defer p.otaGate()
	p.mu.Lock()
	defer p.mu.Unlock()
	o := &p.st.ota
	who := "lan"
	if cloud {
		who = "cloud"
	}
	foreign := func() string {
		live := o.State == "receiving" || o.State == "verified"
		if !live || o.Owner == "" || o.Owner == who || time.Since(o.active) >= otaAbandonAfter {
			o.active = time.Now()
			return ""
		}
		holder := map[string]string{"lan": "the LAN", "cloud": "the cloud"}[o.Owner]
		return "busy: firmware update from " + holder + " is in progress"
	}
	reply := func() map[string]any {
		st := o.State
		if st == "" {
			st = "idle"
		}
		return map[string]any{"state": st, "target": otaTargetName(o.Target), "received": o.Received, "size": o.Size,
			"frames_seen": 0, "sig": "", "sig_key": "", "owner": o.Owner, "error": o.Err}
	}
	switch name {
	case "ota_status":
		return reply(), "", true
	case "ota_abort":
		if why := foreign(); why != "" {
			return nil, why, true
		}
		*o = otaState{State: "idle"}
		return reply(), "", true
	case "ota_selftest":
		if o.State == "receiving" || o.State == "verified" {
			return nil, "busy", true
		}
		return map[string]any{"selftest": "pass"}, "", true
	case "ota_begin":
		size, _ := num(req["size"])
		sha, _ := req["sha256"].(string)
		if _, has := req["size"]; !has || sha == "" {
			return nil, "missing size/sha256", true
		}
		target, _ := req["target"].(string)
		if target != "" && otaTargetName(target) == "" {
			return nil, "unknown target", true
		}
		if target == "ca" && !cloud {
			return nil, "cloud_ca: change it from the cloud or over USB", true
		}
		if why := foreign(); why != "" {
			return nil, why, true
		}
		if size <= 0 || len(sha) != 64 {
			return nil, "bad size / bad sha256", true
		}
		*o = otaState{State: "receiving", Size: size, SHA: strings.ToLower(sha), Target: target, Buf: make([]byte, size), Owner: who, active: time.Now()}
		return reply(), "", true
	case "ota_data":
		if why := foreign(); why != "" {
			return nil, why, true
		}
		b64, ok := req["data"].(string)
		if !ok {
			return nil, "missing data", true
		}
		chunk, err := base64.RawURLEncoding.DecodeString(b64)
		if err != nil {
			return nil, "invalid data", true
		}
		off, _ := num(req["offset"])
		if o.State != "receiving" || off < 0 || off+len(chunk) > o.Size {
			return nil, "no image is being received", true
		}
		copy(o.Buf[off:], chunk)
		if off <= o.Received && off+len(chunk) > o.Received {
			o.Received = off + len(chunk)
		}
		return reply(), "", true
	case "ota_end":
		if why := foreign(); why != "" {
			return nil, why, true
		}
		switch {
		case o.State != "receiving":
			o.State, o.Err = "error", "no image is being received"
		case o.Received != o.Size:
			o.State, o.Err = "error", "incomplete image"
		default:
			sum := sha256.Sum256(o.Buf)
			if hex.EncodeToString(sum[:]) != o.SHA {
				o.State, o.Err = "error", "sha256 mismatch"
			} else {
				o.State, o.Err = "verified", ""
			}
		}
		return reply(), "", true
	case "ota_commit":
		if why := foreign(); why != "" {
			return nil, why, true
		}
		if o.State != "verified" {
			return nil, "no verified image staged", true
		}
		if o.Target != "" {
			o.State = "installed"
			p.installBlob(o)
			return reply(), "", true
		}
		go func() {
			time.Sleep(200 * time.Millisecond)
			p.mu.Lock()
			p.st.firmware = p.cfg.OTAVersion
			p.mu.Unlock()
			p.sim.SetFirmwareVersion(p.cfg.OTAVersion)
			p.Reboot(time.Second)
		}()
		return map[string]any{"committing": true}, "", true
	}
	return nil, "", false
}

func otaTargetName(t string) string {
	switch t {
	case "":
		return "firmware"
	case "gw0", "gw1", "esp", "ca", "tls", "bridge":
		return t
	}
	return ""
}

func num(v any) (int, bool) {
	if f, ok := v.(float64); ok {
		return int(f), true
	}
	return 0, false
}

// installBlob applies what a blob changes on the pod: a company CA is used from the next link,
// which it reloads once the install has been reported. Callers hold p.mu.
func (p *Pod) installBlob(o *otaState) {
	if o.Target != "ca" {
		return
	}
	var certs []map[string]any
	rest := o.Buf
	for {
		var block *pem.Block
		block, rest = pem.Decode(rest)
		if block == nil {
			break
		}
		if cert, err := x509.ParseCertificate(block.Bytes); err == nil {
			sum := sha256.Sum256(block.Bytes)
			certs = append(certs, map[string]any{"subject": cert.Subject.CommonName, "sha256": hex.EncodeToString(sum[:])})
		}
	}
	p.sim.SetCA(certs)
	time.AfterFunc(500*time.Millisecond, func() { p.Drop(0) })
}
