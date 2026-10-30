package benchsim

import (
	"bufio"
	"encoding/json"
	"math"
	"net"
	"regexp"
	"runtime"
	"strings"
	"testing"
	"time"
)

// startSim serves a fresh simulator on a loopback port.
func startSim(t *testing.T, opts ...Option) (*Sim, string) {
	t.Helper()
	sim := New(opts...)
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	go func() { _ = sim.Serve(ln) }()
	t.Cleanup(func() { _ = sim.Close() })
	return sim, ln.Addr().String()
}

// frames sends one command on a fresh connection and reads reply lines until "more" is false.
func frames(t *testing.T, addr string, req map[string]any) []map[string]any {
	t.Helper()
	c, err := net.Dial("tcp", addr)
	if err != nil {
		t.Fatal(err)
	}
	defer c.Close()
	_ = c.SetDeadline(time.Now().Add(20 * time.Second))
	b, _ := json.Marshal(req)
	if _, err := c.Write(append(b, '\n')); err != nil {
		t.Fatal(err)
	}
	br := bufio.NewReader(c)
	var out []map[string]any
	for {
		line, err := br.ReadBytes('\n')
		if err != nil {
			t.Fatalf("%v: %v (after %d frames)", req, err, len(out))
		}
		var m map[string]any
		if err := json.Unmarshal(line, &m); err != nil {
			t.Fatalf("bad reply %q: %v", line, err)
		}
		out = append(out, m)
		if more, _ := m["more"].(bool); !more {
			return out
		}
	}
}

// ok sends a one-reply command and returns its data, failing on an error reply.
func ok(t *testing.T, addr string, req map[string]any) any {
	t.Helper()
	r := frames(t, addr, req)[0]
	if r["status"] != "ok" {
		t.Fatalf("%v: %v", req, r)
	}
	return r["data"]
}

// refused sends a command that must fail and returns its message.
func refused(t *testing.T, addr string, req map[string]any) string {
	t.Helper()
	r := frames(t, addr, req)[0]
	if r["status"] != "error" {
		t.Fatalf("%v: want an error, got %v", req, r)
	}
	return r["message"].(string)
}

// console is a UART proxy session on the node's console.
type console struct {
	t   *testing.T
	c   net.Conn
	buf strings.Builder
}

func openConsole(t *testing.T, addr string) *console {
	t.Helper()
	c, err := net.Dial("tcp", addr)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = c.Close() })
	_, _ = c.Write([]byte(`{"cmd":"uart_proxy_start","rx":3,"tx":4,"baud":115200}` + "\n"))
	ack := make([]byte, 0, 64)
	one := make([]byte, 1)
	for {
		_ = c.SetReadDeadline(time.Now().Add(5 * time.Second))
		if _, err := c.Read(one); err != nil {
			t.Fatal(err)
		}
		if one[0] == '\n' {
			break
		}
		ack = append(ack, one[0])
	}
	if !strings.Contains(string(ack), "uart ready") {
		t.Fatalf("uart_proxy_start ack: %s", ack)
	}
	return &console{t: t, c: c}
}

func (k *console) expect(pattern string, timeout time.Duration) string {
	k.t.Helper()
	re := regexp.MustCompile(pattern)
	deadline := time.Now().Add(timeout)
	b := make([]byte, 1024)
	for !re.MatchString(k.buf.String()) {
		if time.Now().After(deadline) {
			k.t.Fatalf("timed out waiting for %q; console so far:\n%s", pattern, k.buf.String())
		}
		_ = k.c.SetReadDeadline(time.Now().Add(100 * time.Millisecond))
		n, _ := k.c.Read(b)
		k.buf.Write(b[:n])
	}
	text := k.buf.String()
	loc := re.FindStringIndex(text)
	k.buf.Reset()
	k.buf.WriteString(text[loc[1]:])
	return text[:loc[1]]
}

func (k *console) send(line string) {
	k.buf.Reset()
	if _, err := k.c.Write([]byte(line + "\r\n")); err != nil {
		k.t.Fatal(err)
	}
}

func TestStatusAndLaVoltage(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t)
	st := ok(t, addr, map[string]any{"cmd": "status"}).(map[string]any)
	if st["version"] != podFirmware || st["board"] != podBoard {
		t.Fatalf("status: %v", st)
	}
	if !strings.Contains(strings.Join(toStrings(st["caps"]), ","), "sensor_types") {
		t.Fatalf("caps: %v", st["caps"])
	}
	if ok(t, addr, map[string]any{"cmd": "ping"}) != "pong" {
		t.Fatal("ping")
	}
	if msg := refused(t, addr, map[string]any{"cmd": "la_capture", "samples": 64}); !strings.HasPrefix(msg, "la voltage not set") {
		t.Fatalf("la_capture before la_voltage: %s", msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "la_voltage", "mv": 2500}); !strings.Contains(msg, "1800 or 3300") {
		t.Fatal(msg)
	}
	lv := ok(t, addr, map[string]any{"cmd": "la_voltage", "mv": 3300}).(map[string]any)
	if lv["mv"] != 3300.0 {
		t.Fatalf("la_voltage: %v", lv)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "dap_start", "swclk": 11, "swdio": 12}); !strings.Contains(msg, "not available in the demo simulator") {
		t.Fatal(msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "frobnicate"}); msg != "unknown command: frobnicate" {
		t.Fatal(msg)
	}
}

func toStrings(v any) []string {
	var out []string
	for _, x := range v.([]any) {
		out = append(out, x.(string))
	}
	return out
}

func TestBootLogAndConsole(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300))
	k := openConsole(t, addr)
	start := time.Now()
	ok(t, addr, map[string]any{"cmd": "target_power", "efuse": 1, "state": 1})
	boot := k.expect(`APP_OK\r\n`, 4*time.Second)
	if d := time.Since(start); d < 900*time.Millisecond || d > 2*time.Second {
		t.Errorf("APP_OK after %v, want about 1 s", d)
	}
	for _, want := range []string{"PROCESS-NODE: stm32f446 + mcp2515", "reset=power-on",
		"CAN init ok bitrate=500000 osc=8000000 tq=8 sp=75% cnf=00 91 01", "ENV: no BMP280 at 0x76/0x77"} {
		if !strings.Contains(boot, want) {
			t.Errorf("boot log lacks %q:\n%s", want, boot)
		}
	}
	k.expect(`> $`, time.Second)
	k.send("info")
	info := k.expect(`INFO [^\r]*\r\n`, 2*time.Second)
	if !regexp.MustCompile(`fw=process-node version=0\.2\.0 .* boot_ms=2\d\d reset=power-on`).MatchString(info) {
		t.Errorf("info: %q", info)
	}
	k.send("aout 1500")
	k.expect(`AOUT mode=dc mv=1500 code=1863\r\n`, 2*time.Second)
	v := ok(t, addr, map[string]any{"cmd": "adc_read", "source": "ext"}).(map[string]any)
	if mv := v["mv"].(float64); math.Abs(mv-1500) > 5 {
		t.Errorf("adc_read of PA4 at 1500 mV: %v", v)
	}
	ok(t, addr, map[string]any{"cmd": "dac_out", "path": "3v3", "volts": 1.1})
	time.Sleep(50 * time.Millisecond)
	k.send("ain")
	ain := k.expect(`AIN [^\r]*\r\n`, 2*time.Second)
	m := regexp.MustCompile(`mv=(\d+)`).FindStringSubmatch(ain)
	if m == nil || math.Abs(atof(m[1])-1100) > 4 {
		t.Errorf("ain with the DAC at 1.1 V: %q", ain)
	}
	k.send("bogus")
	k.expect(`unknown command: bogus \(try help\)\r\n`, 2*time.Second)
	k.send("reset")
	k.expect(`RESET: rebooting via NVIC_SystemReset\(\)\r\n`, 2*time.Second)
	k.expect(`reset=software`, 2*time.Second)
	k.expect(`APP_OK`, 2*time.Second)

	// The console holds LA3/LA4: a GPIO claim on them is the firmware's pin conflict.
	if msg := refused(t, addr, map[string]any{"cmd": "gpio", "la": 3, "mode": "output"}); !strings.HasPrefix(msg, "pin conflict: LA3 is in use by uart_rx") {
		t.Fatal(msg)
	}
}

func atof(s string) float64 {
	var f float64
	_ = json.Unmarshal([]byte(s), &f)
	return f
}

func TestSensorAlarmAndPins(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300), WithDUTPowered())
	ok(t, addr, map[string]any{"cmd": "sensor_start", "type": "bmp280", "addr": "0x76", "sda": 2, "scl": 1})
	ok(t, addr, map[string]any{"cmd": "sensor_set", "temperature_c": 21.5})
	k := openConsole(t, addr)
	k.expect(`ENV: bmp280 found at 0x76\r\n`, 2*time.Second)
	time.Sleep(300 * time.Millisecond)
	k.send("alarm")
	a := k.expect(`ALARM [^\r]*\r\n`, 2*time.Second)
	if !strings.Contains(a, "env=ok env_c=21.5 press_pa=10132") {
		t.Fatalf("alarm: %q", a)
	}
	ok(t, addr, map[string]any{"cmd": "sensor_set", "temperature_c": 60})
	k.expect(`EVT alarm set=overtemp active=0x01\r\n`, 2*time.Second)
	lv := ok(t, addr, map[string]any{"cmd": "gpio"}).(map[string]any)
	if int(lv["levels"].(float64))&(1<<(dutAlarm-1)) == 0 {
		t.Fatalf("the alarm pin (LA5) is not high: %v", lv["levels"])
	}
	st := ok(t, addr, map[string]any{"cmd": "sensor_status"}).(map[string]any)
	if st["transactions"].(float64) < 4 {
		t.Fatalf("sensor_status: %v", st)
	}
	pins := ok(t, addr, map[string]any{"cmd": "la_pins"}).(map[string]any)["pins"].([]any)
	if f := pins[0].(map[string]any)["function"]; f != "i2c_scl" {
		t.Fatalf("LA1 function %v", f)
	}
	// Swapped bus channels: the node no longer finds its sensor.
	ok(t, addr, map[string]any{"cmd": "sensor_start", "type": "bmp280", "sda": 1, "scl": 2})
	k.expect(`EVT alarm set=env_lost`, 3*time.Second)
}

// decodeUART reads 8N1 bytes off one channel of RLE LA frames (the SDK's decoder, in brief).
func decodeUART(edges [][2]int, n, ch int, rate, baud float64) string {
	level := func(i int) int {
		w := 0
		for _, e := range edges {
			if e[0] > i {
				break
			}
			w = e[1]
		}
		return (w >> (ch - 1)) & 1
	}
	spb := rate / baud
	var out []byte
	for i := 1; i < n; i++ {
		if level(i-1) == 1 && level(i) == 0 {
			b := 0
			for bit := 0; bit < 8; bit++ {
				b |= level(i+int((1.5+float64(bit))*spb)) << bit
			}
			out = append(out, byte(b))
			i += int(9.5 * spb)
		}
	}
	return string(out)
}

func TestLaCaptureShowsTheBootLog(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300))
	ok(t, addr, map[string]any{"cmd": "target_power", "efuse": 1, "state": 1, "delay_ms": 300})
	fs := frames(t, addr, map[string]any{"cmd": "la_capture", "samples": 60000, "sample_rate_mhz": 2.0,
		"trigger": map[string]any{"la": 3, "edge": "falling"}, "trigger_timeout_ms": 5000})
	var edges [][2]int
	n := 0
	for _, f := range fs {
		if f["la"] != true || f["la_rate_hz"].(float64) != 2e6 {
			t.Fatalf("frame: %v", f)
		}
		for _, e := range f["la_edges"].([]any) {
			p := e.([]any)
			edges = append(edges, [2]int{int(p[0].(float64)), int(p[1].(float64))})
		}
		n = int(f["la_upto"].(float64))
	}
	if n != 60000 {
		t.Fatalf("la_upto %d", n)
	}
	if text := decodeUART(edges, n, dutTX, 2e6, dutBaud); !strings.Contains(text, "PROCESS-NODE: stm32f446") {
		t.Fatalf("decoded %q", text)
	}
}

func TestTriggerTimeout(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300))
	msg := refused(t, addr, map[string]any{"cmd": "la_capture", "samples": 1000,
		"trigger": map[string]any{"la": 9, "edge": "rising"}, "trigger_timeout_ms": 100})
	if !regexp.MustCompile(`trigger timeout: no rising edge on LA9`).MatchString(msg) {
		t.Fatal(msg)
	}
}

func TestPowerProfileAndAdcCapture(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithDUTPowered())
	fs := frames(t, addr, map[string]any{"cmd": "power_profile", "efuse": 1, "rate_hz": 500, "keep_samples": 200, "duration_ms": 500})
	stats := fs[len(fs)-1]["stats"].(map[string]any)
	if avg := stats["avg_ua"].(float64); avg < 40000 || avg > 50000 {
		t.Fatalf("idle current %v uA", avg)
	}
	if r := stats["rate_hz"].(float64); r < 360 || r > 370 {
		t.Fatalf("rate %v", r)
	}
	ok(t, addr, map[string]any{"cmd": "target_power", "efuse": 1, "state": 0})
	fs = frames(t, addr, map[string]any{"cmd": "power_profile", "efuse": 1, "rate_hz": 200, "keep_samples": 0, "duration_ms": 200})
	if avg := fs[len(fs)-1]["stats"].(map[string]any)["avg_ua"].(float64); avg != 0 {
		t.Fatalf("rail off draws %v uA", avg)
	}
	fs = frames(t, addr, map[string]any{"cmd": "capture", "samples": 5000, "enc": "b64"})
	if fs[0]["adc_rate_hz"].(float64) != adcMaxRateHz || fs[0]["b64"] == nil {
		t.Fatalf("capture frame: %v", fs[0])
	}
}

func TestBmp280RoundTrip(t *testing.T) {
	for _, c := range []float64{-20, 0, 24, 55.5, 85} {
		var img [256]byte
		buildBmpImage(&img, false, c, 101325, 0)
		cal := parseBmpCalib(img[0x88:0xA0])
		adcT := int32(uint32(img[0xFA])<<12 | uint32(img[0xFB])<<4 | uint32(img[0xFC])>>4)
		adcP := int32(uint32(img[0xF7])<<12 | uint32(img[0xF8])<<4 | uint32(img[0xF9])>>4)
		cdc, tFine := bmpTempCdc(cal, adcT)
		if math.Abs(float64(cdc)-c*100) > 1 {
			t.Errorf("%g degC reads back as %d cdC", c, cdc)
		}
		if p := bmpPressPa(cal, adcP, tFine); math.Abs(float64(p)-101325) > 2 {
			t.Errorf("101325 Pa reads back as %d", p)
		}
	}
}

func TestCloseEndsEveryConnection(t *testing.T) {
	before := runtime.NumGoroutine()
	sim, addr := startSim(t, WithLaVoltage(3300))
	openConsole(t, addr) // a long-lived UART session
	go func() {
		c, err := net.Dial("tcp", addr)
		if err == nil {
			_, _ = c.Write([]byte(`{"cmd":"power_profile","efuse":1,"duration_ms":60000}` + "\n"))
		}
	}()
	time.Sleep(200 * time.Millisecond)
	done := make(chan struct{})
	go func() { _ = sim.Close(); close(done) }()
	select {
	case <-done:
	case <-time.After(3 * time.Second):
		t.Fatal("Close did not return")
	}
	time.Sleep(100 * time.Millisecond)
	if after := runtime.NumGoroutine(); after > before+2 {
		t.Errorf("goroutines: %d before, %d after Close", before, after)
	}
	if len(sim.Log()) < 2 {
		t.Errorf("log: %v", sim.Log())
	}
}
