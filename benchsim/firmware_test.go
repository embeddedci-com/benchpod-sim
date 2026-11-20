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
		exec(map[string]any{"cmd": "target_power", "efuse": float64(1), "enabled": true})
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

// The 4.0.0 names: rates in Hz (sample_rate_hz, adc_sample_rate_hz, la_sample_rate_hz), base64
// ADC samples by default, and no stream verb.
func TestCaptureNames(t *testing.T) {
	_, addr := startSim(t, WithFirmware("4.0.0", Hooks{}), WithLaVoltage(3300))
	fs := frames(t, addr, map[string]any{"cmd": "capture_dual", "adc_samples": 300, "adc_sample_rate_hz": 200000,
		"la_samples": 400, "la_sample_rate_hz": 1e6})
	if fs[0]["adc_sample_rate_hz"] != 200000.0 || fs[0]["la_sample_rate_hz"] != 1e6 || fs[0]["b64"] == nil {
		t.Fatalf("first frame: %v", fs[0])
	}
	var la map[string]any
	for _, f := range fs {
		if f["la"] == true {
			la = f
			break
		}
	}
	if la == nil || la["la_sample_rate_hz"] != 1e6 {
		t.Fatalf("no LA frame with its rate: %v", fs)
	}
	if fs := frames(t, addr, map[string]any{"cmd": "capture_read", "offset": 0, "enc": "dec"}); fs[0]["data"] == nil {
		t.Fatalf("capture_read enc dec: %v", fs[0])
	}
	if msg := refused(t, addr, map[string]any{"cmd": "stream", "samples": 100}); msg != "BP0002 cmd=stream" {
		t.Fatal(msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "capture", "samples": 40000}); msg != "BP0067 max=32768" {
		t.Fatal(msg)
	}
}

// The 4.0.0 field names of SPI, power_profile, dac_out, the loop and CAN.
func TestRenamedFields(t *testing.T) {
	_, addr := startSim(t, WithFirmware("4.0.0", Hooks{}), WithLaVoltage(3300))
	spi := ok(t, addr, map[string]any{"cmd": "spi_start", "sck": 10, "mosi": 11, "miso": 12, "cs": 13, "clock_hz": 1000000}).(map[string]any)
	if spi["clock_hz"] != 1e6 || spi["hz"] != nil {
		t.Fatalf("spi_start: %v", spi)
	}
	if st := ok(t, addr, map[string]any{"cmd": "spi_status"}).(map[string]any); st["clock_hz"] != 1e6 {
		t.Fatalf("spi_status: %v", st)
	}
	ok(t, addr, map[string]any{"cmd": "spi_stop"})
	out := ok(t, addr, map[string]any{"cmd": "dac_out", "path": "3v3", "v": 1.5}).(map[string]any)
	if mv := out["mv"].(float64); mv < 1495 || mv > 1505 || out["code16"] == nil || out["code"] != nil {
		t.Fatalf("dac_out: %v", out)
	}
	if p := ok(t, addr, map[string]any{"cmd": "dac_loop_probe"}).(map[string]any); p["code16"] == nil || p["v"] != nil {
		t.Fatalf("dac_loop_probe: %v", p)
	}
	fs := frames(t, addr, map[string]any{"cmd": "power_profile", "action": "start", "efuse": 1, "sample_rate_hz": 200})
	if fs[0]["data"].(map[string]any)["sample_rate_hz"] != 200.0 {
		t.Fatalf("power_profile start: %v", fs[0])
	}
	frames(t, addr, map[string]any{"cmd": "power_profile", "action": "stop"})
	if msg := refused(t, addr, map[string]any{"cmd": "power_profile", "action": "start", "sample_rate_hz": 1000}); msg != "sample_rate_hz must be 100..500" {
		t.Fatal(msg)
	}
	ok(t, addr, map[string]any{"cmd": "can_start", "bitrate": 500000})
	ok(t, addr, map[string]any{"cmd": "can_stop"})
	for _, old := range []string{"can_config", "can_disable", "la"} {
		if msg := refused(t, addr, map[string]any{"cmd": old}); msg != "BP0002 cmd="+old {
			t.Fatalf("%s: %s", old, msg)
		}
	}
}

// la_pull and la_step replace la; target_power, target_status and la_pull use booleans.
func TestLaPullAndBooleans(t *testing.T) {
	_, addr := startSim(t, WithFirmware("4.0.0", Hooks{}))
	if msg := refused(t, addr, map[string]any{"cmd": "la_pull"}); msg != "BP0001" {
		t.Fatal(msg)
	}
	ok(t, addr, map[string]any{"cmd": "la_voltage", "mv": 1800})
	p := ok(t, addr, map[string]any{"cmd": "la_pull", "la": 3, "pull": "up"}).(map[string]any)
	if p["state"] != "up" || p["pullups_available"] != true || p["pullup"] != nil || p["ohms"] != nil || p["pull"] != nil ||
		len(p["dirs"].([]any)) != 1 {
		t.Fatalf("la_pull set: %v", p)
	}
	if m := ok(t, addr, map[string]any{"cmd": "la_pull"}).(map[string]any); m["la_pullup_mask"] != 4.0 || m["pullups_available"] != true {
		t.Fatalf("la_pull mask: %v", m)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "la_pull", "la": 7, "pull": "up"}); msg != "BP0024 have=down la=7 want=up" {
		t.Fatal(msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "la_pull", "la": 9, "pull": "up"}); msg != "BP0023 la=9" {
		t.Fatal(msg)
	}
	if q := ok(t, addr, map[string]any{"cmd": "la_pull", "la": 3, "pullup": "off"}).(map[string]any); q["state"] != "up" {
		t.Fatalf("pullup is not read any more: %v", q)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "la_step", "la": 5}); msg != "missing steps" {
		t.Fatal(msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "target_power", "efuse": 1, "state": 1}); msg != "enabled must be true or false" {
		t.Fatal(msg)
	}
	tp := ok(t, addr, map[string]any{"cmd": "target_power", "efuse": 1, "enabled": true}).(map[string]any)
	if tp["enabled"] != true {
		t.Fatalf("target_power: %v", tp)
	}
	e1 := ok(t, addr, map[string]any{"cmd": "target_status"}).(map[string]any)["efuse1"].(map[string]any)
	if e1["enabled"] != true || e1["fault"] != false || e1["valid"] != true || e1["flt_pin"] != 0.0 {
		t.Fatalf("target_status: %v", e1)
	}
}

// generate takes only volts on a path or microamps (no 8-bit amplitude/offset), freq_hz, and
// replies null or {"cotrig":bool}.
func TestGenerateForms(t *testing.T) {
	for _, fw := range []bool{true, false} {
		var opts []Option
		if fw {
			opts = append(opts, WithFirmware("4.0.0", Hooks{}))
		}
		_, addr := startSim(t, opts...)
		msg := refused(t, addr, map[string]any{"cmd": "generate", "waveform": "sine", "freq_hz": 100, "amplitude": 100, "offset": 128})
		if msg != "amplitude_v (with path and offset_v) or amplitude_ua (with offset_ua) is required" {
			t.Fatalf("firmware %v: 8-bit form: %s", fw, msg)
		}
		req := map[string]any{"cmd": "generate", "waveform": "sine", "freq_hz": 100, "path": "3v3", "amplitude_v": 1, "offset_v": 1.65}
		if d := ok(t, addr, req); d != nil {
			t.Fatalf("firmware %v: generate: %v", fw, d)
		}
		req["on_capture"] = true
		if d, _ := ok(t, addr, req).(map[string]any); d == nil || d["cotrig"] == nil || d["bits"] != nil {
			t.Fatalf("firmware %v: generate on_capture: %v", fw, d)
		}
		ok(t, addr, map[string]any{"cmd": "dac_stop"})
	}
}

// Signatures are always required, the LAN cannot change lan_tls or usb_network, and SCPI is off
// until the cloud switches it on.
func TestPolicies(t *testing.T) {
	s, addr := startSim(t, WithFirmware("4.0.0", Hooks{}))
	sp := ok(t, addr, map[string]any{"cmd": "sig_policy", "set": "required"}).(map[string]any)
	if sp["policy"] != "required" || sp["lan_unsigned"] != nil {
		t.Fatalf("sig_policy: %v", sp)
	}
	for _, want := range []string{"audit", "permissive"} {
		r := s.ExecReply(map[string]any{"cmd": "sig_policy", "set": want, "client_api": 2}, true)
		if r.Code != "BP0074" || r.Args["setting"] != "sig_policy" || r.Args["want"] != want {
			t.Fatalf("sig_policy %s from the cloud: %+v", want, r)
		}
	}
	tls := ok(t, addr, map[string]any{"cmd": "lan_tls"}).(map[string]any)
	if tls["https_only"] != true || tls["scpi"] != false {
		t.Fatalf("lan_tls defaults: %v", tls)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "lan_tls", "scpi": true}); msg != "BP0042 setting=lan_tls" {
		t.Fatal(msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "usb_network", "enabled": false}); msg != "BP0043 setting=usb_network" {
		t.Fatal(msg)
	}
	if u := ok(t, addr, map[string]any{"cmd": "usb_network"}).(map[string]any); u["enabled"] != true {
		t.Fatalf("usb_network: %v", u)
	}
	st := ok(t, addr, map[string]any{"cmd": "status"}).(map[string]any)
	if usb := st["usb"].(map[string]any); usb["net"] != false || usb["enabled"] != true {
		t.Fatalf("status.usb: %v", usb)
	}
	scpi := func() string {
		c, err := net.Dial("tcp", addr)
		if err != nil {
			t.Fatal(err)
		}
		defer c.Close()
		_ = c.SetDeadline(time.Now().Add(5 * time.Second))
		_, _ = c.Write([]byte("*IDN?\n"))
		line, _ := bufio.NewReader(c).ReadString('\n')
		return line
	}
	if line := scpi(); !strings.HasPrefix(line, "SCPI is off on this BenchPod") {
		t.Fatalf("SCPI off: %q", line)
	}
	if r := s.ExecReply(map[string]any{"cmd": "lan_tls", "scpi": true, "client_api": 2}, true); r.Error != "" {
		t.Fatalf("lan_tls scpi from the cloud: %+v", r)
	}
	if line := scpi(); !strings.HasPrefix(line, "EmbeddedCI,BenchPod,0,4.0.0") {
		t.Fatalf("SCPI on: %q", line)
	}
}
