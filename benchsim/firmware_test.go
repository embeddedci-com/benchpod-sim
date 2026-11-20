package benchsim

import (
	"bufio"
	"encoding/json"
	"net"
	"strings"
	"testing"
	"time"
)

// After a pod reset the DUT is off, and powering it boots it again: the banner reaches a UART proxy.
func TestFirmwareRebootThenDUTBootsAgain(t *testing.T) {
	s := New(WithFirmware("3.9.4", Hooks{}))
	defer s.Close()
	for round := 0; round < 2; round++ {
		exec := func(cmd map[string]any) {
			t.Helper()
			cmd["client_api"] = 2
			if _, msg := s.Exec(cmd, true); msg != "" {
				t.Fatalf("%v: %s", cmd, msg)
			}
		}
		exec(map[string]any{"cmd": "la_voltage", "mv": float64(3300)})
		here, there := net.Pipe()
		go s.ServeTunnel(there, 3)
		line, _ := json.Marshal(map[string]any{"cmd": "uart_proxy_start", "rx": 3, "tx": 4, "baud": 115200, "client_api": 2})
		go here.Write(append(line, '\n'))
		br := bufio.NewReader(here)
		if _, err := br.ReadString('\n'); err != nil {
			t.Fatal(err)
		}
		exec(map[string]any{"cmd": "target_power", "efuse": float64(1), "state": float64(1)})
		seen := ""
		_ = here.SetReadDeadline(time.Now().Add(8 * time.Second))
		for !strings.Contains(seen, "APP_OK") {
			buf := make([]byte, 512)
			n, err := br.Read(buf)
			seen += string(buf[:n])
			if err != nil {
				t.Fatalf("round %d: no banner: %v; got %q", round, err, seen)
			}
		}
		here.Close()
		s.Reboot()
	}
}

// Firmware 4.0.0 refuses a client below client API 2 (BP0030, with a message) except on the
// verbs every client may send, and answers an unknown verb with BP0002.
func TestClientAPIGate(t *testing.T) {
	s := New(WithFirmware("4.0.0", Hooks{}))
	defer s.Close()
	r := s.ExecReply(map[string]any{"cmd": "la_pins", "client_api": 1}, false)
	if r.Code != "BP0030" || r.Args["client_api"] != 1.0 || r.Args["min_client_api"] != 2.0 || r.Args["api"] != 2.0 ||
		!strings.HasPrefix(r.Error, "Update the BenchPod SDK, MCP or CLI") {
		t.Fatalf("old client: %+v", r)
	}
	if r := s.ExecReply(map[string]any{"cmd": "la_pins"}, false); r.Code != "BP0030" || r.Args["client_api"] != 0.0 {
		t.Fatalf("no client_api: %+v", r)
	}
	for _, cmd := range []string{"status", "ping", "ota_status", "blob_status"} {
		if r := s.ExecReply(map[string]any{"cmd": cmd}, false); r.Code == "BP0030" {
			t.Fatalf("%s refused for an old client: %+v", cmd, r)
		}
	}
	if r := s.ExecReply(map[string]any{"cmd": "frobnicate"}, false); r.Code != "BP0030" {
		t.Fatalf("unknown verb from an old client: %+v", r)
	}
	r = s.ExecReply(map[string]any{"cmd": "frobnicate", "client_api": 2}, false)
	if r.Code != "BP0002" || r.Error != "BP0002" || r.Args["cmd"] != "frobnicate" {
		t.Fatalf("unknown verb: %+v", r)
	}
	if r := s.ExecReply(map[string]any{"cmd": "la_pins", "client_api": 2}, false); r.Error != "" {
		t.Fatalf("la_pins: %+v", r)
	}
}
