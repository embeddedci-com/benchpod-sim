package benchsim

import (
	"encoding/base64"
	"encoding/json"
	"strings"
	"testing"
)

// The wiring store: a profile saved in pieces reads back the same, the first one becomes the
// default, a stale if_revision is refused, and a reboot keeps everything.
func TestWiringStoreRoundTrip(t *testing.T) {
	s := New(WithFirmware("4.0.0", Hooks{}))
	defer s.Close()
	exec := func(cmd map[string]any) Reply {
		t.Helper()
		cmd["client_api"] = 2
		return s.ExecReply(cmd, true)
	}
	data := func(r Reply) map[string]any {
		t.Helper()
		if r.Error != "" {
			t.Fatalf("refused: %+v", r)
		}
		var m map[string]any
		if err := json.Unmarshal(r.Data, &m); err != nil {
			t.Fatal(err)
		}
		return m
	}
	if d := data(exec(map[string]any{"cmd": "wiring_list"})); d["default"] != nil || len(d["profiles"].([]any)) != 0 {
		t.Fatalf("empty store: %v", d)
	}
	profile := []byte(`{"version":2,"la_mv":3300,"signals":{"x":"` + strings.Repeat("a", 900) + `"}}`)
	put := func(name string, ifRev any) Reply {
		var last Reply
		for off := 0; off < len(profile); off += wiringPieceMax {
			end := min(off+wiringPieceMax, len(profile))
			cmd := map[string]any{"cmd": "wiring_put", "name": name, "offset": float64(off), "total": float64(len(profile)),
				"data_b64": base64.RawURLEncoding.EncodeToString(profile[off:end])}
			if off == 0 && ifRev != nil {
				cmd["if_revision"] = ifRev
			}
			if last = exec(cmd); last.Error != "" {
				return last
			}
		}
		return last
	}
	if d := data(put("k66", 0.0)); d["rev"] != 1.0 || d["default"] != true {
		t.Fatalf("first save: %v", d)
	}
	if r := put("k66", 0.0); r.Code != "BP0009" {
		t.Fatalf("stale if_revision: %+v", r)
	}
	if d := data(put("k66", 1.0)); d["rev"] != 2.0 {
		t.Fatalf("second save: %v", d)
	}
	data(put("nrf", nil))
	s.Reboot()
	var got []byte
	for {
		d := data(exec(map[string]any{"cmd": "wiring_get", "name": "k66", "offset": float64(len(got))}))
		b, _ := base64.RawURLEncoding.DecodeString(d["data_b64"].(string))
		got = append(got, b...)
		if len(b) == 0 || len(got) >= int(d["total"].(float64)) {
			break
		}
	}
	if string(got) != string(profile) {
		t.Fatalf("read back %q", got)
	}
	if r := exec(map[string]any{"cmd": "wiring_delete", "name": "k66"}); r.Code != "BP0013" {
		t.Fatalf("delete the default: %+v", r)
	}
	if d := data(exec(map[string]any{"cmd": "wiring_default", "name": "nrf"})); d["default"] != "nrf" {
		t.Fatalf("set default: %v", d)
	}
	data(exec(map[string]any{"cmd": "wiring_delete", "name": "k66"}))
	if r := exec(map[string]any{"cmd": "wiring_get", "name": "k66", "offset": 0.0}); r.Code != "BP0007" {
		t.Fatalf("deleted profile: %+v", r)
	}
	if r := exec(map[string]any{"cmd": "wiring_get", "name": "-bad", "offset": 0.0}); r.Code != "BP0008" {
		t.Fatalf("bad name: %+v", r)
	}
}
