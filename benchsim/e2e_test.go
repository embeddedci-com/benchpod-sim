package benchsim

import (
	"bytes"
	"context"
	"encoding/json"
	"net"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/modelcontextprotocol/go-sdk/mcp"
)

// TestMCPEndToEnd drives the simulator through the real BenchPod MCP server, the way the website
// demo does: every tool of the demo's allow-list and the demo's stories. It needs the server
// binary:
//
//	BENCHPOD_MCP_COMMAND=.../embeddedci-python/venv/bin/embeddedci-mcp \
//	    go test -run TestMCPEndToEnd -v ./benchsim/
func TestMCPEndToEnd(t *testing.T) {
	bin := os.Getenv("BENCHPOD_MCP_COMMAND")
	if bin == "" {
		t.Skip("set BENCHPOD_MCP_COMMAND to the embeddedci-mcp binary to run the MCP end-to-end test")
	}
	sim := New()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	go func() { _ = sim.Serve(ln) }()
	t.Cleanup(func() { _ = sim.Close() })

	home := t.TempDir()
	wiring := filepath.Join(home, "process-node-wiring.json")
	if err := os.WriteFile(wiring, []byte(WiringProfileJSON), 0o644); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Minute)
	defer cancel()
	cmd := exec.Command(bin, "--connection", ln.Addr().String())
	cmd.Env = append(filteredEnv(), "HOME="+home, "BENCHPOD_TLS=off", "BENCHPOD_WIRING="+wiring)
	stderr := &lockedBuffer{}
	cmd.Stderr = stderr
	defer func() {
		if t.Failed() {
			t.Logf("MCP server log:\n%s", excerpt(stderr.String(), 8000))
		}
	}()
	client := mcp.NewClient(&mcp.Implementation{Name: "benchsim-e2e", Version: "1.0"}, nil)
	tools, err := client.Connect(ctx, &mcp.CommandTransport{Command: cmd}, nil)
	if err != nil {
		t.Fatal(err)
	}
	defer tools.Close()

	call := func(name string, args map[string]any) string {
		t.Helper()
		out, err := tools.CallTool(ctx, &mcp.CallToolParams{Name: name, Arguments: args})
		if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		res := resultText(out)
		if out.IsError {
			t.Fatalf("%s(%v) failed: %s", name, args, res)
		}
		t.Logf("%s(%s) -> %s", name, compact(args), excerpt(res, 600))
		return res
	}
	expect := func(text, pattern string) {
		t.Helper()
		if !regexp.MustCompile(pattern).MatchString(text) {
			t.Fatalf("want %q in:\n%s", pattern, text)
		}
	}

	// Connect as the host does, then the agent's first steps.
	expect(call("connect", nil), `"connected":\s*true`)
	call("set_la_voltage", map[string]any{"voltage": 3.3})
	expect(call("status", nil), `"la_voltage":\s*3\.3`)
	w := call("wiring", nil)
	expect(w, `"uart_rx":\s*3`)
	expect(w, `"wired_to":\s*"ALARM"`)
	call("power_status", nil)

	// Story 1: "power-cycle the board and show me the boot log" (no sensor emulated yet).
	boot := call("power_cycle_and_capture", map[string]any{"until_regex": "APP_OK"})
	expect(boot, `PROCESS-NODE: stm32f446 \+ mcp2515`)
	expect(boot, `INFO fw=process-node version=0\.2\.0`)
	expect(boot, `ENV: no BMP280 at 0x76/0x77`)
	expect(boot, `"matched":\s*true`)

	// Story 2: "why does it say sensor error?": no emulated sensor; a BME280 is the wrong part
	// for this firmware (chip id 0x60), a BMP280 works.
	expect(call("sensor_status", nil), `"instances":\s*\[\]`)
	expect(call("sensor_library", nil), `"bmp280"`)
	expect(call("sensor_show", map[string]any{"name": "bmp280"}), `temperature_c`)
	call("set_pull", map[string]any{"las": []any{1, 2}, "enabled": true})
	call("pull_status", nil)
	call("sensor_start", map[string]any{"definition": "bme280", "instance": "env"})
	expect(call("power_cycle_and_capture", map[string]any{"until_regex": "APP_OK"}), `chip id 0x60, not 0x58`)
	expect(call("sensor_start", map[string]any{"definition": "bmp280", "instance": "env", "values": map[string]any{"temperature_c": 24}}), `"slot":\s*"i2c0"`)
	boot = call("power_cycle_and_capture", map[string]any{"until_regex": "APP_OK"})
	expect(boot, `ENV: bmp280 ok addr=0x76`)
	expect(call("sensor_status", nil), `"writes":\s*[1-9]`)
	expect(call("sensor_regs", map[string]any{"instance": "env", "start": 0xD0, "length": 1}), `"bytes":\s*\[\s*88\s*\]`)

	// Story 3: the console: readings, then an over-temperature alarm from the emulated sensor.
	call("uart_open", nil)
	call("uart_write", map[string]any{"text": "alarm"})
	expect(call("uart_read", map[string]any{"until_regex": `ALARM .*press_pa=\d+ events=\d+`, "timeout": 3}), `env=ok env_c=24\.0`)
	call("sensor_set", map[string]any{"instance": "env", "values": map[string]any{"temperature_c": 55}})
	expect(call("uart_read", map[string]any{"until_regex": "EVT alarm set=overtemp", "timeout": 3}), `"matched":\s*true`)
	expect(call("gpio_read", map[string]any{"la": []any{"ALARM"}}), `"level":\s*1`)
	call("la_pins", nil)
	call("sensor_set", map[string]any{"instance": "env", "values": map[string]any{"temperature_c": 30}})
	expect(call("uart_read", map[string]any{"until_regex": "EVT alarm clear=overtemp", "timeout": 3}), `"matched":\s*true`)

	// Story 4: "set the room temperature to 30 degC and see if the thermostat turns the heater
	// on": PA1 is the process sensor (0.5 V + 20 mV/degC) driven by the pod's 3.3 V DAC, PA4 the
	// heater read back on the ADC SMA.
	call("dac_output", map[string]any{"path": "3v3", "volts": 0.5 + 0.02*30})
	time.Sleep(100 * time.Millisecond) // the node samples PA1 every 10 ms
	call("uart_write", map[string]any{"text": "ctl on 40"})
	expect(call("uart_read", map[string]any{"until_regex": `CTL mode=on[^\r]*\r\n`, "timeout": 3}), `pv_c=30\.0`)
	time.Sleep(3 * time.Second)
	expect(call("adc_read", map[string]any{"source": "ext"}), `"voltage":\s*[1-3]\.`)
	expect(call("capture_adc", map[string]any{"samples": 4096, "source": "ext"}), `"mean":\s*[1-3]\.`)
	call("uart_write", map[string]any{"text": "ctl off"})
	call("uart_read", map[string]any{"until_regex": "CTL mode=off", "timeout": 3})
	call("uart_close", nil)

	// Story 5: "capture the I2C bus".
	call("capture_la", map[string]any{"samples": 20000, "sample_rate_hz": 1e6, "trigger_la": "i2c_sda", "trigger_edge": "falling"})
	expect(call("decode_la", map[string]any{"protocol": "i2c", "sda": 2, "scl": 1}), `0x76`)
	// sensor_capture sends no sample rate, so the pod captures at its highest rate: 16384
	// samples at 24 MHz are 0.68 ms, shorter than one of the node's 0.9 ms reads, as on a real
	// pod. It answers; the decode needs a whole transfer.
	expect(call("sensor_capture", map[string]any{"instance": "env", "samples": 4096, "address": 0x76, "register": 0xF7}), `"transactions":`)

	// The boot log on the logic analyzer: power on 0.5 s out, trigger on the first start bit.
	call("power_off", nil)
	call("power_on", map[string]any{"delay": 0.5})
	call("capture_la", map[string]any{"samples": 100000, "sample_rate_hz": 2e6, "trigger_la": "uart_rx", "trigger_edge": "falling"})
	expect(call("decode_la", map[string]any{"protocol": "uart", "rx": 3, "baud": 115200}), `PROCESS-NODE`)

	// Story 6: "measure the board's current".
	time.Sleep(time.Second)
	expect(call("measure_power", map[string]any{"duration": 1.0, "points": 20}), `"avg_current":\s*0\.04`)

	// The rest of the allow-list.
	call("reset_target", nil)
	expect(call("capture_uart", map[string]any{"duration": 2.0, "until_regex": "APP_OK"}), `reset=pin`)
	call("generate", map[string]any{"waveform": "sine", "freq_hz": 50, "amplitude": 0.5, "offset": 1.5})
	call("dac_stop", nil)
	expect(call("sensor_stop", nil), `"env"`)
	call("power_off", nil)

	if len(sim.Log()) == 0 {
		t.Fatal("the simulator logged no commands")
	}
}

// filteredEnv is the test's environment without anything that would point the MCP server at a
// real pod or account.
func filteredEnv() []string {
	var out []string
	for _, kv := range os.Environ() {
		k, _, _ := strings.Cut(kv, "=")
		if strings.HasPrefix(k, "BENCHPOD_") || strings.HasPrefix(k, "EMBEDDEDCI_") || k == "HOME" {
			continue
		}
		out = append(out, kv)
	}
	return out
}

func compact(v map[string]any) string {
	if len(v) == 0 {
		return ""
	}
	b, _ := json.Marshal(v)
	return string(b)
}

func excerpt(s string, n int) string {
	s = strings.Join(strings.Fields(s), " ")
	if len(s) <= n {
		return s
	}
	return s[:n] + " ..."
}

// lockedBuffer collects the MCP server's stderr (written from the exec goroutine).
type lockedBuffer struct {
	mu  sync.Mutex
	buf bytes.Buffer
}

func (b *lockedBuffer) Write(p []byte) (int, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if b.buf.Len() < 1<<20 {
		b.buf.Write(p)
	}
	return len(p), nil
}

func (b *lockedBuffer) String() string {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.buf.String()
}

// resultText is what the model reads: the text parts, or the structured result.
func resultText(res *mcp.CallToolResult) string {
	var parts []string
	for _, c := range res.Content {
		if t, ok := c.(*mcp.TextContent); ok {
			parts = append(parts, t.Text)
		}
	}
	if len(parts) == 0 && res.StructuredContent != nil {
		b, _ := json.Marshal(res.StructuredContent)
		return string(b)
	}
	return strings.Join(parts, "\n")
}
