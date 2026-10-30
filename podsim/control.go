package podsim

import (
	"encoding/json"
	"net/http"
	"strconv"
	"time"
)

// ControlHandler is the simulated pod's side door for tests that run it as its own process: read
// its state and command log, reboot it, drop its link, slow a command down, and talk to it like a
// LAN client would.
//
//	GET  /healthz
//	GET  /state                      State
//	GET  /commands                   the commands it saw, oldest first
//	POST /reboot?down_ms=500         power-cycle
//	POST /drop?down_ms=500           the link drops, no reboot
//	POST /unregister                 delete the pod from the account (disconnects it)
//	POST /register?name=N            register it again as a new pod (new identity)
//	POST /latency?cmd=X&ms=300       how long command X takes (0 = no delay)
//	POST /lan/command                {"cmd":...} as a LAN client: {"status":"ok","data":..} or {"status":"error","message":..}
func (p *Pod) ControlHandler() http.Handler {
	mux := http.NewServeMux()
	writeJSON := func(w http.ResponseWriter, code int, v any) {
		w.Header().Set("Content-Type", "application/json")
		w.WriteHeader(code)
		_ = json.NewEncoder(w).Encode(v)
	}
	ms := func(r *http.Request, def int) time.Duration {
		if v, err := strconv.Atoi(r.URL.Query().Get("down_ms")); err == nil && v >= 0 {
			return time.Duration(v) * time.Millisecond
		}
		return time.Duration(def) * time.Millisecond
	}
	mux.HandleFunc("GET /healthz", func(w http.ResponseWriter, r *http.Request) { writeJSON(w, 200, map[string]any{"ok": true}) })
	mux.HandleFunc("GET /state", func(w http.ResponseWriter, r *http.Request) { writeJSON(w, 200, p.Snapshot()) })
	mux.HandleFunc("GET /commands", func(w http.ResponseWriter, r *http.Request) { writeJSON(w, 200, p.Commands()) })
	mux.HandleFunc("POST /reboot", func(w http.ResponseWriter, r *http.Request) {
		p.Reboot(ms(r, 500))
		writeJSON(w, 200, map[string]any{"ok": true})
	})
	mux.HandleFunc("POST /drop", func(w http.ResponseWriter, r *http.Request) {
		p.Drop(ms(r, 500))
		writeJSON(w, 200, map[string]any{"ok": true})
	})
	mux.HandleFunc("POST /unregister", func(w http.ResponseWriter, r *http.Request) {
		if err := p.Unregister(r.Context()); err != nil {
			writeJSON(w, 500, map[string]any{"error": err.Error()})
			return
		}
		writeJSON(w, 200, map[string]any{"ok": true})
	})
	mux.HandleFunc("POST /register", func(w http.ResponseWriter, r *http.Request) {
		if err := p.Reregister(r.Context(), r.URL.Query().Get("name")); err != nil {
			writeJSON(w, 500, map[string]any{"error": err.Error()})
			return
		}
		writeJSON(w, 200, map[string]any{"ok": true, "device_id": p.DeviceID()})
	})
	mux.HandleFunc("POST /latency", func(w http.ResponseWriter, r *http.Request) {
		v, _ := strconv.Atoi(r.URL.Query().Get("ms"))
		p.SetLatency(r.URL.Query().Get("cmd"), time.Duration(v)*time.Millisecond)
		writeJSON(w, 200, map[string]any{"ok": true})
	})
	mux.HandleFunc("POST /lan/command", func(w http.ResponseWriter, r *http.Request) {
		var cmd map[string]any
		if err := json.NewDecoder(r.Body).Decode(&cmd); err != nil {
			writeJSON(w, 400, map[string]any{"status": "error", "message": "bad json"})
			return
		}
		data, msg := p.RunLAN(cmd)
		if msg != "" {
			writeJSON(w, 200, map[string]any{"status": "error", "message": msg})
			return
		}
		writeJSON(w, 200, map[string]any{"status": "ok", "data": data})
	})
	return mux
}
