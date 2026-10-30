package benchsim

import (
	"math"
	"strings"
)

// The pod's emulated I2C sensors (sensor_sim.c and the per-model files): an I2C target on two
// LA channels that the DUT's controller reads like the real part. Register-image parts (BMP280,
// BME280, MPU-6050) take a write as "register pointer, then data"; the SHT4x takes commands.

type sensorParam struct {
	key, unit     string
	min, max, def float64
}

type sensorModel struct {
	name, label   string
	addr, altAddr byte
	params        []sensorParam
}

var tpParams = []sensorParam{
	{"temperature_c", "C", -100, 150, 25},
	{"pressure_pa", "Pa", 1000, 200000, 101325},
}

var sensorModels = []*sensorModel{
	{name: "bmp280", label: "Bosch BMP280: temperature and pressure", addr: 0x76, altAddr: 0x77, params: tpParams},
	{name: "bme280", label: "Bosch BME280: temperature, pressure and humidity", addr: 0x76, altAddr: 0x77,
		params: append(append([]sensorParam(nil), tpParams...), sensorParam{"humidity_pct", "%", 0, 100, 40})},
	{name: "sht4x", label: "Sensirion SHT4x: temperature and humidity", addr: 0x44, altAddr: 0x45,
		params: []sensorParam{{"temperature_c", "C", -40, 125, 25}, {"humidity_pct", "%", 0, 100, 40}}},
	{name: "mpu6050", label: "InvenSense MPU-6050: accelerometer and gyroscope", addr: 0x68, altAddr: 0x69,
		params: []sensorParam{
			{"accel_x_g", "g", -16, 16, 0}, {"accel_y_g", "g", -16, 16, 0}, {"accel_z_g", "g", -16, 16, 1},
			{"gyro_x_dps", "dps", -2000, 2000, 0}, {"gyro_y_dps", "dps", -2000, 2000, 0},
			{"gyro_z_dps", "dps", -2000, 2000, 0}, {"temperature_c", "C", -40, 85, 25},
		}},
}

func findModel(name string) *sensorModel {
	for _, m := range sensorModels {
		if m.name == strings.ToLower(name) {
			return m
		}
	}
	return nil
}

type emuSensor struct {
	model    *sensorModel
	addr     byte
	sda, scl int
	values   []float64
	img      [256]byte
	ptr      byte
	resp     []byte // SHT4x: the reply to its last command
	xfers    int
	writes   int
	lastReg  byte
	lastVal  byte
}

func newEmuSensor(m *sensorModel, addr byte, sda, scl int) *emuSensor {
	e := &emuSensor{model: m, addr: addr, sda: sda, scl: scl}
	for _, p := range m.params {
		e.values = append(e.values, p.def)
	}
	e.rebuild(true)
	return e
}

func (e *emuSensor) value(key string) float64 {
	for i, p := range e.model.params {
		if p.key == key {
			return e.values[i]
		}
	}
	return 0
}

// rebuild reloads the register image from the readings. power-on also resets the
// configuration registers (an MPU-6050 wakes up asleep).
func (e *emuSensor) rebuild(powerOn bool) {
	switch e.model.name {
	case "bmp280", "bme280":
		live := e.img
		buildBmpImage(&e.img, e.model.name == "bme280", e.value("temperature_c"), e.value("pressure_pa"), e.value("humidity_pct"))
		if !powerOn { // keep what the DUT configured
			for _, r := range []int{0xF2, 0xF4, 0xF5} {
				e.img[r] = live[r]
			}
		}
	case "mpu6050":
		cfg := [3]byte{e.img[0x1B], e.img[0x1C], e.img[0x6B]}
		if powerOn {
			cfg = [3]byte{0, 0, 0x40} // SLEEP set after power-up
		}
		e.img = [256]byte{}
		e.img[0x75] = 0x68
		e.img[0x1B], e.img[0x1C], e.img[0x6B] = cfg[0], cfg[1], cfg[2]
		e.mpuData()
	}
}

func (e *emuSensor) mpuData() {
	put := func(a int, v float64) {
		n := int16(math.Max(-32768, math.Min(32767, math.Round(v))))
		e.img[a], e.img[a+1] = byte(uint16(n)>>8), byte(n)
	}
	if e.img[0x6B]&0x40 != 0 { // asleep: the data registers hold zeros
		for a := 0x3B; a <= 0x48; a++ {
			e.img[a] = 0
		}
		return
	}
	accelLSB := []float64{16384, 8192, 4096, 2048}[(e.img[0x1C]>>3)&3]
	gyroLSB := []float64{131, 65.5, 32.8, 16.4}[(e.img[0x1B]>>3)&3]
	put(0x3B, e.value("accel_x_g")*accelLSB)
	put(0x3D, e.value("accel_y_g")*accelLSB)
	put(0x3F, e.value("accel_z_g")*accelLSB)
	put(0x41, (e.value("temperature_c")-36.53)*340)
	put(0x43, e.value("gyro_x_dps")*gyroLSB)
	put(0x45, e.value("gyro_y_dps")*gyroLSB)
	put(0x47, e.value("gyro_z_dps")*gyroLSB)
}

// write is the controller writing data to the part; it ACKs every byte.
func (e *emuSensor) write(data []byte) {
	e.xfers++
	if len(data) == 0 {
		return
	}
	if e.model.name == "sht4x" {
		e.shtCommand(data[0])
		return
	}
	e.ptr = data[0]
	for _, b := range data[1:] {
		reg := e.ptr
		e.writes++
		e.lastReg, e.lastVal = reg, b
		e.img[reg] = b
		e.ptr++
		if e.model.name == "mpu6050" {
			if reg == 0x6B && b&0x80 != 0 { // DEVICE_RESET clears itself
				e.rebuild(true)
			} else {
				e.mpuData()
			}
		}
	}
}

// read is the controller reading n bytes from the part.
func (e *emuSensor) read(n int) []byte {
	e.xfers++
	out := make([]byte, n)
	if e.model.name == "sht4x" {
		copy(out, e.resp)
		for i := len(e.resp); i < n; i++ {
			out[i] = 0xFF
		}
		return out
	}
	for i := range out {
		out[i] = e.img[e.ptr]
		e.ptr++
	}
	return out
}

func crc8Sensirion(b []byte) byte {
	crc := byte(0xFF)
	for _, x := range b {
		crc ^= x
		for i := 0; i < 8; i++ {
			if crc&0x80 != 0 {
				crc = crc<<1 ^ 0x31
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

func (e *emuSensor) shtCommand(cmd byte) {
	word := func(v uint16) []byte {
		b := []byte{byte(v >> 8), byte(v)}
		return append(b, crc8Sensirion(b))
	}
	switch cmd {
	case 0xFD, 0xF6, 0xE0: // measure, high / medium / low precision
		st := (e.value("temperature_c") + 45) / 175 * 65535
		srh := (e.value("humidity_pct") + 6) / 125 * 65535
		e.resp = append(word(uint16(math.Max(0, math.Min(65535, math.Round(st))))),
			word(uint16(math.Max(0, math.Min(65535, math.Round(srh)))))...)
	case 0x89: // serial number
		e.resp = append(word(0x1234), word(0x5678)...)
	default: // soft reset (0x94) and the heater commands answer nothing
		e.resp = nil
	}
}

func (e *emuSensor) valuesJSON() map[string]any {
	out := map[string]any{}
	for i, p := range e.model.params {
		out[p.key] = math.Round(e.values[i]*1000) / 1000
	}
	return out
}

// sensorAt is the emulated part answering at addr on the node's sensor bus: the right channels and address,
// with nothing holding the bus low.
func (s *Sim) sensorAt(addr byte) *emuSensor {
	e := s.pod.sensor
	if e == nil || e.addr != addr || e.sda != dutSDA || e.scl != dutSCL || s.busHeldLow() {
		return nil
	}
	return e
}

func (s *Sim) cmdSensorStart(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.requireLaVoltage(); err != nil {
		return nil, err
	}
	if !r.has("type") {
		return nil, fwError("missing type")
	}
	if !r.has("sda") || !r.has("scl") {
		return nil, fwError("missing sda/scl")
	}
	sda, scl := r.int("sda", 0), r.int("scl", 0)
	if err := s.checkClaim("i2c_sda", []int{sda}, "i2c_sda", "i2c_scl"); err != nil {
		return nil, err
	}
	if err := s.checkClaim("i2c_scl", []int{scl}, "i2c_sda", "i2c_scl"); err != nil {
		return nil, err
	}
	m := findModel(r.str("type"))
	if m == nil {
		return nil, fwError("unknown sensor type")
	}
	if sda == scl || sda < 1 || sda > 14 || scl < 1 || scl > 14 {
		return nil, fwError("sensor start failed (bad channel?)")
	}
	addr := m.addr
	if a, ok := r.num("addr"); ok && a != 0 {
		addr = byte(a)
	}
	now := s.now()
	s.advance(now)
	s.releaseFn("i2c_sda", "i2c_scl")
	s.pod.pins[sda] = laPin{fn: "i2c_sda"}
	s.pod.pins[scl] = laPin{fn: "i2c_scl"}
	s.pod.sensor = newEmuSensor(m, addr, sda, scl)
	return map[string]any{"type": m.name, "addr": int(addr), "sda": sda, "scl": scl}, nil
}

func (s *Sim) cmdSensorSet(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e := s.pod.sensor
	if e == nil {
		return nil, fwError("no sensor active")
	}
	vals := map[int]float64{}
	for i, p := range e.model.params {
		if !r.has(p.key) {
			continue
		}
		v, ok := r.num(p.key)
		if !ok || v < p.min || v > p.max {
			return nil, errf("%s must be a number from %d to %d (%s)", p.key, int(math.Round(p.min)), int(math.Round(p.max)), p.unit)
		}
		vals[i] = v
	}
	if len(vals) == 0 {
		return nil, errf("no %s parameter given (sensor_types lists them)", e.model.name)
	}
	s.advance(s.now())
	for i, v := range vals {
		e.values[i] = v
	}
	e.rebuild(false)
	return map[string]any{"type": e.model.name, "values": e.valuesJSON()}, nil
}

func (s *Sim) cmdSensorStop(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.advance(s.now())
	s.pod.sensor = nil
	s.releaseFn("i2c_sda", "i2c_scl")
	return nil, nil
}

func (s *Sim) cmdSensorStatus(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.advance(s.now())
	e := s.pod.sensor
	if e == nil {
		return map[string]any{"active": false}, nil
	}
	return map[string]any{
		"active": true, "type": e.model.name, "addr": int(e.addr), "sda": e.sda, "scl": e.scl,
		"transactions": e.xfers, "writes": e.writes, "last_reg": int(e.lastReg), "last_val": int(e.lastVal),
		"values": e.valuesJSON(),
	}, nil
}

func (s *Sim) cmdSensorTypes(request) (any, error) {
	types := []any{}
	for _, m := range sensorModels {
		params := []any{}
		for _, p := range m.params {
			params = append(params, map[string]any{"key": p.key, "unit": p.unit, "min": p.min, "max": p.max, "default": p.def})
		}
		types = append(types, map[string]any{"type": m.name, "label": m.label, "addr": int(m.addr),
			"alt_addr": int(m.altAddr), "params": params})
	}
	return map[string]any{"types": types}, nil
}

func (s *Sim) cmdSensorRegs(c *conn, r request) error {
	s.mu.Lock()
	e := s.pod.sensor
	if e == nil {
		s.mu.Unlock()
		err := fwError("no sensor active")
		_ = c.send(errReply(err.Error()))
		return err
	}
	start := r.int("start", 0)
	n := r.int("len", 256)
	if n <= 0 || start < 0 || start+n > 256 {
		s.mu.Unlock()
		err := fwError("start/len out of range")
		_ = c.send(errReply(err.Error()))
		return err
	}
	data := make([]int, n)
	for i := range data {
		data[i] = int(e.img[start+i])
	}
	s.mu.Unlock()
	return sendSamples(c, data, 256)
}

// sendSamples writes a sample list the way bulk_begin does: chunks of "data" with "more".
func sendSamples(c *conn, data []int, per int) error {
	if len(data) == 0 {
		return c.send(map[string]any{"status": "ok", "data": []int{}, "more": false})
	}
	for i := 0; i < len(data); i += per {
		j := min(i+per, len(data))
		if err := c.send(map[string]any{"status": "ok", "data": data[i:j], "more": j < len(data)}); err != nil {
			return nil
		}
	}
	return nil
}
