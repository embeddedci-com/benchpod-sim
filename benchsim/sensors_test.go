package benchsim

import (
	"encoding/base64"
	"os"
	"strings"
	"testing"
	"time"
)

// testDefinition is a library definition as JSON (testdata/sensors: benchpod-sensors v0.1.0,
// converted from YAML with the key order kept, as clients send it).
func testDefinition(t *testing.T, name string) []byte {
	t.Helper()
	b, err := os.ReadFile("testdata/sensors/" + name + ".json")
	if err != nil {
		t.Fatal(err)
	}
	return b
}

// upload sends a definition or a stimulus in 768-byte chunks, as the SDK does, and returns the
// last chunk's reply.
func upload(t *testing.T, addr string, base map[string]any, data []byte) map[string]any {
	t.Helper()
	var last map[string]any
	for off := 0; off < len(data); off += 768 {
		end := min(off+768, len(data))
		req := map[string]any{"offset": off, "total": len(data), "data_b64": base64.StdEncoding.EncodeToString(data[off:end])}
		for k, v := range base {
			req[k] = v
		}
		last = ok(t, addr, req).(map[string]any)
	}
	return last
}

func defineSensor(t *testing.T, addr, instance, name string) map[string]any {
	t.Helper()
	return upload(t, addr, map[string]any{"cmd": "sensor_define", "instance": instance}, testDefinition(t, name))
}

func TestSensorDefine(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300))
	def := testDefinition(t, "bme280")
	first := ok(t, addr, map[string]any{"cmd": "sensor_define", "instance": "env", "offset": 0, "total": len(def),
		"data_b64": base64.StdEncoding.EncodeToString(def[:768])}).(map[string]any)
	if first["received"] != 768.0 {
		t.Fatalf("first chunk: %v", first)
	}
	msg := refused(t, addr, map[string]any{"cmd": "sensor_define", "instance": "env", "offset": 1000, "total": len(def), "data_b64": "AA"})
	if msg != "chunk at offset 1000, expected 768 (start again at offset 0)" {
		t.Fatalf("out of order: %q", msg)
	}
	rep := defineSensor(t, addr, "env", "bme280")
	i2c := rep["i2c"].(map[string]any)
	if rep["kind"] != "i2c" || rep["name"] != "bme280" || i2c["address"] != "0x76" || i2c["alt_addresses"].([]any)[0] != "0x77" ||
		len(rep["params"].([]any)) != 3 || rep["program_bytes"].(float64) <= 0 {
		t.Fatalf("define: %v", rep)
	}
	gps := defineSensor(t, addr, "gps", "gps_nmea")
	if u := gps["uart"].(map[string]any); u["baud"] != 9600.0 || u["format"] != "8N1" {
		t.Fatalf("uart define: %v", gps)
	}
	bad := []byte(`{"format":1,"kind":"i2c","name":"x","params":[],"i2c":{"address":118},"registers":[{"at":0,"u8":"nope + 1"}]}`)
	msg = refused(t, addr, map[string]any{"cmd": "sensor_define", "instance": "x", "offset": 0, "total": len(bad),
		"data_b64": base64.StdEncoding.EncodeToString(bad)})
	if !strings.HasPrefix(msg, "invalid definition: registers[0]") {
		t.Fatalf("broken definition: %q", msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_define", "instance": "Env", "offset": 0, "total": 2, "data_b64": "e30"}); msg != "instance must be 1..16 characters of a-z, 0-9 and _" {
		t.Fatalf("bad name: %q", msg)
	}
	dry := upload(t, addr, map[string]any{"cmd": "sensor_define", "instance": "probe", "dry_run": true}, testDefinition(t, "lis3dh"))
	if dry["spi"].(map[string]any)["mode"] != 3.0 {
		t.Fatalf("dry run: %v", dry)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_start", "instance": "probe", "pins": map[string]any{}}); msg != "unknown sensor instance 'probe'" {
		t.Fatalf("a dry run kept the definition: %q", msg)
	}
	st := ok(t, addr, map[string]any{"cmd": "sensor_status"}).(map[string]any)
	if n := len(st["instances"].([]any)); n != 2 || st["image"] != "loop" {
		t.Fatalf("status: %v", st)
	}
	stopped := ok(t, addr, map[string]any{"cmd": "sensor_stop", "all": true}).(map[string]any)["stopped"].([]any)
	if len(stopped) != 2 {
		t.Fatalf("stop all: %v", stopped)
	}
}

func TestSensorStartChecks(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300))
	defineSensor(t, addr, "env", "bme280")
	defineSensor(t, addr, "env2", "bme280")
	defineSensor(t, addr, "temp", "bmp280")
	for req, want := range map[string]map[string]any{
		"an I2C sensor needs pins sda and scl":               {"pins": map[string]any{"sda": 2}},
		"address 0x40 is not one of this sensor's addresses": {"pins": map[string]any{"sda": 2, "scl": 1}, "address": "0x40"},
		"humidity_pct must be 0..100 %":                      {"pins": map[string]any{"sda": 2, "scl": 1}, "values": map[string]any{"humidity_pct": 120}},
		"env has no param 'x'":                               {"pins": map[string]any{"sda": 2, "scl": 1}, "values": map[string]any{"x": 1}},
	} {
		want["cmd"], want["instance"] = "sensor_start", "env"
		if msg := refused(t, addr, want); msg != req {
			t.Errorf("%v: %q, want %q", want, msg, req)
		}
	}
	ok(t, addr, map[string]any{"cmd": "sensor_start", "instance": "env", "pins": map[string]any{"sda": 2, "scl": 1}})
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_start", "instance": "env", "pins": map[string]any{"sda": 2, "scl": 1}}); msg != "env is running: sensor_stop it first" {
		t.Fatalf("running: %q", msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_start", "instance": "env2", "pins": map[string]any{"sda": 2, "scl": 1}}); msg != "address 0x76 is used by sensor:env on this bus" {
		t.Fatalf("same address: %q", msg)
	}
	// Two sensors share the bus at different addresses; another pair is a conflict.
	st := ok(t, addr, map[string]any{"cmd": "sensor_start", "instance": "env2", "address": 0x77, "pins": map[string]any{"sda": 2, "scl": 1}}).(map[string]any)
	if st["slot"] != "i2c1" || st["image_loaded"] != false {
		t.Fatalf("second sensor: %v", st)
	}
	pins := ok(t, addr, map[string]any{"cmd": "la_pins"}).(map[string]any)["pins"].([]any)
	if o := pins[1].(map[string]any)["owner"]; o != "sensor:env+1" {
		t.Fatalf("shared line owner: %v", o)
	}
	msg := refused(t, addr, map[string]any{"cmd": "sensor_start", "instance": "temp", "pins": map[string]any{"sda": 2, "scl": 3}})
	if msg != `pin conflict: LA2 is in use by sensor:env (i2c_sda); stop the sensor emulation first ({"cmd":"sensor_stop","instance":"env"})` {
		t.Fatalf("conflict: %q", msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "gpio", "la": 1, "mode": "output"}); !strings.Contains(msg, "sensor:env (i2c_scl)") {
		t.Fatalf("gpio on a sensor pin: %q", msg)
	}
	ok(t, addr, map[string]any{"cmd": "sensor_stop", "instance": "env"})
	pins = ok(t, addr, map[string]any{"cmd": "la_pins"}).(map[string]any)["pins"].([]any)
	if p := pins[1].(map[string]any); p["function"] != "i2c_sda" || p["owner"] != "sensor:env2" {
		t.Fatalf("after one stop: %v", p)
	}
	ok(t, addr, map[string]any{"cmd": "sensor_stop", "all": true})
	pins = ok(t, addr, map[string]any{"cmd": "la_pins"}).(map[string]any)["pins"].([]any)
	if p := pins[1].(map[string]any); p["function"] != "none" || p["owner"] != nil {
		t.Fatalf("after stop all: %v", p)
	}
}

func TestSensorRegsAndFaults(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300), WithDUTPowered())
	defineSensor(t, addr, "env", "bmp280")
	ok(t, addr, map[string]any{"cmd": "sensor_start", "instance": "env", "pins": map[string]any{"sda": 2, "scl": 1}})
	regs := ok(t, addr, map[string]any{"cmd": "sensor_regs", "instance": "env", "start": "0xD0", "len": 1}).(map[string]any)
	if regs["start"] != "0xD0" || regs["len"] != 1.0 || regs["data_b64"] != "WA==" { // chip id 0x58
		t.Fatalf("sensor_regs: %v", regs)
	}
	k := openConsole(t, addr)
	k.expect(`ENV: bmp280 found at 0x76\r\n`, 2*time.Second)
	// The node's own writes are in the image: normal mode in ctrl_meas.
	regs = ok(t, addr, map[string]any{"cmd": "sensor_regs", "instance": "env", "start": 0xF4, "len": 1}).(map[string]any)
	if regs["data_b64"] != "Jw==" {
		t.Fatalf("ctrl_meas: %v", regs)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_fault", "instance": "env", "type": "garbage"}); msg != "fault garbage does not apply to an i2c sensor" {
		t.Fatalf("garbage on i2c: %q", msg)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_fault", "instance": "env", "type": "bus_stuck", "duration_s": 3}); msg != "bus_stuck needs a duration_s of at most 1 s" {
		t.Fatalf("long bus_stuck: %q", msg)
	}
	f := ok(t, addr, map[string]any{"cmd": "sensor_fault", "instance": "env", "type": "no_response"}).(map[string]any)
	if f["fault"] != "no_response" {
		t.Fatalf("fault: %v", f)
	}
	k.expect(`EVT alarm set=env_lost`, 3*time.Second)
	st := ok(t, addr, map[string]any{"cmd": "sensor_status", "instance": "env"}).(map[string]any)
	faults := st["instances"].([]any)[0].(map[string]any)["faults"].([]any)
	if len(faults) != 1 || faults[0].(map[string]any)["left_s"] != nil {
		t.Fatalf("faults: %v", faults)
	}
	cl := ok(t, addr, map[string]any{"cmd": "sensor_fault_clear", "instance": "env"}).(map[string]any)
	if len(cl["faults"].([]any)) != 0 {
		t.Fatalf("clear: %v", cl)
	}
	k.expect(`ENV: bmp280 found at 0x76\r\n`, 3*time.Second)
	// A frozen reading: sensor_set no longer reaches the node.
	ok(t, addr, map[string]any{"cmd": "sensor_fault", "instance": "env", "type": "stuck"})
	ok(t, addr, map[string]any{"cmd": "sensor_set", "instance": "env", "values": map[string]any{"temperature_c": 70}})
	time.Sleep(700 * time.Millisecond)
	k.send("alarm")
	if a := k.expect(`ALARM [^\r]*\r\n`, 2*time.Second); !strings.Contains(a, "env_c=25.0") {
		t.Fatalf("stuck reading: %q", a)
	}
}

func TestSensorStimulus(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300), WithDUTPowered())
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_stimulus_start"}); !strings.HasPrefix(msg, "no stimulus loaded") {
		t.Fatalf("no stimulus: %q", msg)
	}
	rep := upload(t, addr, map[string]any{"cmd": "sensor_stimulus"}, testDefinition(t, "temperature_ramp_fault"))
	if rep["faults"] != 2.0 || rep["duration_s"] != 150.0 || rep["signals"].([]any)[0] != "temperature_c" {
		t.Fatalf("stimulus: %v", rep)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_stimulus_start"}); !strings.HasPrefix(msg, "signal temperature_c: 0 sensors run") {
		t.Fatalf("nothing running: %q", msg)
	}
	defineSensor(t, addr, "env", "bmp280")
	ok(t, addr, map[string]any{"cmd": "sensor_start", "instance": "env", "pins": map[string]any{"sda": 2, "scl": 1}})
	ok(t, addr, map[string]any{"cmd": "sensor_stimulus_start", "at_s": 69})
	time.Sleep(100 * time.Millisecond)
	st := ok(t, addr, map[string]any{"cmd": "sensor_status"}).(map[string]any)
	inst := st["instances"].([]any)[0].(map[string]any)
	if v := inst["values"].(map[string]any)["temperature_c"].(float64); v < 59 || v > 60 {
		t.Fatalf("temperature at 69 s: %v", v)
	}
	if s := st["stimulus"].(map[string]any); s["running"] != true || s["t_s"].(float64) < 69 {
		t.Fatalf("stimulus status: %v", s)
	}
	// 120 s: the scheduled no_response fault.
	ok(t, addr, map[string]any{"cmd": "sensor_stimulus_start", "at_s": 120.2})
	time.Sleep(100 * time.Millisecond)
	inst = ok(t, addr, map[string]any{"cmd": "sensor_status"}).(map[string]any)["instances"].([]any)[0].(map[string]any)
	faults := inst["faults"].([]any)
	if len(faults) != 1 || faults[0].(map[string]any)["type"] != "no_response" || faults[0].(map[string]any)["stimulus"] != true {
		t.Fatalf("stimulus fault: %v", faults)
	}
	ok(t, addr, map[string]any{"cmd": "sensor_stimulus_stop"})
	inst = ok(t, addr, map[string]any{"cmd": "sensor_status"}).(map[string]any)["instances"].([]any)[0].(map[string]any)
	if len(inst["faults"].([]any)) != 0 {
		t.Fatalf("faults after stop: %v", inst["faults"])
	}
}

func TestSensorUARTAndAnalog(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithLaVoltage(3300))
	defineSensor(t, addr, "gps", "gps_nmea")
	st := ok(t, addr, map[string]any{"cmd": "sensor_start", "instance": "gps", "pins": map[string]any{"tx": 9}, "baud": 115200}).(map[string]any)
	if st["slot"] != "uart0" || st["baud"] != 115200.0 {
		t.Fatalf("uart start: %v", st)
	}
	time.Sleep(1200 * time.Millisecond)
	inst := ok(t, addr, map[string]any{"cmd": "sensor_status"}).(map[string]any)["instances"].([]any)[0].(map[string]any)
	if bus := inst["bus"].(map[string]any); bus["messages"].(float64) < 1 || bus["sent_bytes"].(float64) < 100 {
		t.Fatalf("uart bus: %v", bus)
	}
	if msg := refused(t, addr, map[string]any{"cmd": "sensor_la", "instance": "gps"}); msg != "sensor_la captures I2C sensors only; use la_capture for the pins of gps" {
		t.Fatalf("sensor_la of a uart sensor: %q", msg)
	}
	// An NTC on the DAC: the output follows the temperature, and dac_out is refused meanwhile.
	defineSensor(t, addr, "ntc", "ntc_10k_divider")
	st = ok(t, addr, map[string]any{"cmd": "sensor_start", "instance": "ntc", "pins": map[string]any{"output": "dac"}}).(map[string]any)
	if st["slot"] != "analog0" {
		t.Fatalf("analog start: %v", st)
	}
	out := func() float64 {
		for _, x := range ok(t, addr, map[string]any{"cmd": "sensor_status"}).(map[string]any)["instances"].([]any) {
			if e := x.(map[string]any); e["instance"] == "ntc" {
				return e["output"].(float64)
			}
		}
		t.Fatal("no ntc")
		return 0
	}
	if v := out(); v < 1.64 || v > 1.66 {
		t.Fatalf("25 degC: %v V", v)
	}
	ok(t, addr, map[string]any{"cmd": "sensor_set", "instance": "ntc", "values": map[string]any{"temperature_c": 60}})
	if v := out(); v > 1.2 {
		t.Fatalf("60 degC: %v V", v)
	}
	msg := refused(t, addr, map[string]any{"cmd": "dac_out", "path": "3v3", "volts": 1})
	if msg != `the dac output is played by sensor:ntc; stop it first ({"cmd":"sensor_stop","instance":"ntc"})` {
		t.Fatalf("dac_out: %q", msg)
	}
	ok(t, addr, map[string]any{"cmd": "dac_stop"})
	ok(t, addr, map[string]any{"cmd": "dac_out", "path": "3v3", "volts": 1})
}

func TestSensorEmulationImage(t *testing.T) {
	t.Parallel()
	_, addr := startSim(t, WithFirmware("3.10.3", Hooks{}), WithLaVoltage(3300))
	if img := ok(t, addr, map[string]any{"cmd": "fpga_image"}).(map[string]any); img["name"] != "loop" {
		t.Fatalf("fpga_image: %v", img)
	}
	defineSensor(t, addr, "env", "bmp280")
	st := ok(t, addr, map[string]any{"cmd": "sensor_start", "instance": "env", "pins": map[string]any{"sda": 2, "scl": 1}}).(map[string]any)
	if st["image_loaded"] != true || len(st["unavailable"].([]any)) != 2 {
		t.Fatalf("start: %v", st)
	}
	msg := refused(t, addr, map[string]any{"cmd": "fpga_image", "image": "loop"})
	if !strings.HasPrefix(msg, "image switch refused: the emulated sensors env need the emulation image") {
		t.Fatalf("switch away: %q", msg)
	}
	msg = refused(t, addr, map[string]any{"cmd": "dac_loop_probe"})
	if !strings.Contains(msg, `not in the emulation image`) {
		t.Fatalf("loop in the emulation image: %q", msg)
	}
	ok(t, addr, map[string]any{"cmd": "sensor_stop", "all": true})
	if img := ok(t, addr, map[string]any{"cmd": "fpga_image", "image": 0}).(map[string]any); img["name"] != "loop" || img["features"] != 1.0 {
		t.Fatalf("back to loop: %v", img)
	}
}
