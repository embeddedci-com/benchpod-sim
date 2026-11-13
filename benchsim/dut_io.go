package benchsim

import (
	"fmt"
	"math"
)

// The node's tasks: analog in (analog.c, proc.h), analog out, the thermostat (thermostat.h),
// the environment sensor (env.c) and the alarms (alarm.h), each a port of the C it names.

const (
	vddaMv          = 3297 // VDDA as the node measures it through VREFINT on this board
	ainPeriodNs     = 10 * msNs
	thermoPeriodMs  = 20
	envPeriodNs     = 200 * msNs
	envProbeNs      = 1000 * msNs
	envFailsToLose  = 3
	aoutMinMv       = 200
	aoutHeadroomMv  = 200
	aoutSineMaxHz   = 1000
	thermoOutMinMv  = 200
	thermoOutMaxMv  = 3000
	alarmOvertemp   = 0x01
	alarmPvFault    = 0x02
	alarmEnvLost    = 0x04
	alarmLimitMinDc = -400
	alarmLimitMaxDc = 850
)

// -- analog in -------------------------------------------------------------------------------

type ainState struct {
	raw, mv, filtMv, vddaMv, samples uint32
}

func rawToMv(raw, vdda uint32) uint32 {
	if raw > 4095 {
		raw = 4095
	}
	return (raw*vdda + 4095/2) / 4095
}

func mvToDc(mv uint32) int32 { return (int32(mv) - 500) / 2 }

// sampleAin is one 16x-oversampled conversion of PA1. The pod's 3.3 V DAC output drives PA1
// through 5-10 kOhm; the node's 60 us sample time settles through it.
func (s *Sim) sampleAin(t int64) {
	d := &s.dut
	v := s.pa1Volts(t)
	raw := math.Round(v*1000*4095/vddaMv + s.noise(1.2))
	raw = math.Max(0, math.Min(4095, raw))
	d.ain.raw = uint32(raw)
	d.ain.vddaMv = vddaMv
	d.ain.mv = rawToMv(d.ain.raw, vddaMv)
}

func (s *Sim) ainInit(t int64) {
	d := &s.dut
	d.ain = ainState{}
	s.sampleAin(t)
	d.filt = d.ain.mv << 4
	d.ain.filtMv = d.ain.mv
}

func (s *Sim) ainTick(t int64) {
	d := &s.dut
	d.ainNext += ainPeriodNs
	if d.ainNext <= t {
		d.ainNext = t + ainPeriodNs
	}
	s.sampleAin(t)
	st := int32(d.filt)
	st += (int32(d.ain.mv<<4) - st) >> 5
	d.filt = uint32(st)
	d.ain.filtMv = (d.filt + 8) >> 4
	d.ain.samples++
	if d.strobe == 1 { // PA8 high for the ~1 ms the 16 conversions take
		s.sig.addPulse(t, t+msNs, dutStrobe)
	}
}

func (s *Sim) ainReadNow(t int64) ainState {
	s.sampleAin(t)
	return s.dut.ain
}

// -- analog out ------------------------------------------------------------------------------

type aoutMode int

const (
	aoutOff aoutMode = iota
	aoutDC
	aoutSine
)

type aoutState struct {
	mode                aoutMode
	mv, ampMv, hz, code uint32
	ampCode             uint32
	t0                  int64
}

type aoutEntry struct {
	t int64
	a aoutState
}

func mvToCode(mv uint32) uint32 { return min((mv*4095+vddaMv/2)/vddaMv, 4095) }

func aoutInRange(lo, hi uint32) bool { return lo >= aoutMinMv && hi+aoutHeadroomMv <= vddaMv }

func (s *Sim) setAout(t int64, a aoutState) {
	d := &s.dut
	a.t0 = t
	d.aout = a
	d.aoutLog = append(d.aoutLog, aoutEntry{t, a})
	cut := 0
	for cut < len(d.aoutLog)-1 && d.aoutLog[cut+1].t < t-historyNs {
		cut++
	}
	if len(d.aoutLog)-cut > 4096 {
		cut = len(d.aoutLog) - 4096
	}
	if cut > 0 {
		d.aoutLog = append([]aoutEntry(nil), d.aoutLog[cut:]...)
	}
	s.pwrMark(t)
}

func (s *Sim) aoutDC(t int64, mv uint32) bool {
	if !aoutInRange(mv, mv) {
		return false
	}
	s.setAout(t, aoutState{mode: aoutDC, mv: mv, code: mvToCode(mv)})
	return true
}

func (s *Sim) aoutSine(t int64, hz, amp, off uint32) bool {
	if hz == 0 || hz > aoutSineMaxHz || amp == 0 || amp > off || !aoutInRange(off-amp, off+amp) {
		return false
	}
	s.setAout(t, aoutState{mode: aoutSine, mv: off, ampMv: amp, hz: hz, code: mvToCode(off), ampCode: mvToCode(amp)})
	return true
}

func (s *Sim) aoutOff(t int64) { s.setAout(t, aoutState{}) }

// pa4Volts is the node's analog output (PA4) at t, which is wired to the pod's ADC SMA. Off is
// high-Z: the pod's input reads about 0 V.
func (s *Sim) pa4Volts(t int64) float64 {
	a := aoutState{}
	for _, e := range s.dut.aoutLog {
		if e.t > t {
			break
		}
		a = e.a
	}
	switch a.mode {
	case aoutDC:
		return float64(a.code) * vddaMv / 4095 / 1000
	case aoutSine:
		ph := 2 * math.Pi * float64(a.hz) * float64(t-a.t0) / 1e9
		code := float64(a.code) + float64(a.ampCode)*math.Sin(ph)
		return code * vddaMv / 4095 / 1000
	}
	return 0
}

// -- thermostat ------------------------------------------------------------------------------

type thermoState struct {
	on     bool
	spDc   int32
	pvDc   int32
	outMv  int32
	steps  uint32
	inBand uint32
	integ  int32 // PI integrator, output units Q8
}

const (
	thermoKpQ8   = 768
	thermoKiQ8   = 51
	thermoBandDc = 10
)

func (th *thermoState) start(sp, currentOut int32) bool {
	if sp < 0 || sp > 1000 {
		return false
	}
	if !th.on {
		if currentOut < thermoOutMinMv {
			currentOut = thermoOutMinMv
		}
		th.integ = currentOut * 256
		th.outMv = currentOut
		th.steps = 0
	}
	th.spDc = sp
	th.inBand = 0
	th.on = true
	return true
}

func (th *thermoState) step(pvMv uint32) int32 {
	th.pvDc = mvToDc(pvMv)
	err := th.spDc - th.pvDc
	p := int32(thermoKpQ8) * err
	integ := th.integ + int32(thermoKiQ8)*err
	out := (p + integ) / 256
	switch {
	case out > thermoOutMaxMv:
		out = thermoOutMaxMv
		if err < 0 {
			th.integ = integ
		}
	case out < thermoOutMinMv:
		out = thermoOutMinMv
		if err > 0 {
			th.integ = integ
		}
	default:
		th.integ = integ
	}
	th.integ = max(min(th.integ, thermoOutMaxMv*256), thermoOutMinMv*256)
	th.outMv = out
	th.steps++
	if err <= thermoBandDc && err >= -thermoBandDc {
		th.inBand++
	} else {
		th.inBand = 0
	}
	return out
}

func (s *Sim) thermoTick(t int64) {
	d := &s.dut
	d.thNext += thermoPeriodMs * msNs
	if d.thNext <= t {
		d.thNext = t + thermoPeriodMs*msNs
	}
	out := d.thermo.step(d.ain.mv)
	if uint32(out) != d.aout.mv || d.aout.mode != aoutDC {
		s.aoutDC(t, uint32(out))
	}
	if d.strobe == 2 {
		d.strobePin = !d.strobePin
		s.touchBase(t)
	}
}

// -- environment sensor (BMP280 on I2C1) -----------------------------------------------------

type envStatus int

const (
	envAbsent envStatus = iota
	envOK
	envLost
)

type envState struct {
	status  envStatus
	addr    byte
	tempDc  int32
	pressPa uint32
	reads   uint32
	fails   uint32
	seq     uint32
	// sim: what the last failed probe saw, for the boot line.
	probeID byte
}

// i2cRaw runs the node's HAL transfers on its sensor bus at t and records them on the wire.
// segs are tried in order; a NACKed address ends the transfer (the HAL sends STOP). It returns
// the bytes read, whether every part was ACKed, and when the bus is free again.
func (s *Sim) i2cTransfer(t int64, segs []i2cSeg) (data []byte, ok bool, end int64) {
	if s.busHeldLow(t) {
		// A line held low: the controller cannot even send START; HAL times out after 10 ms.
		return nil, false, t + 10*msNs
	}
	dev := s.sensorAt(segs[0].addr, t)
	ok = true
	for i := range segs {
		sg := &segs[i]
		sg.ack = make([]bool, 1+len(sg.data))
		if dev == nil {
			sg.ack[0] = false
			segs = segs[:i+1]
			ok = false
			break
		}
		sg.ack[0] = true
		if sg.read {
			sg.data = s.i2cRead(dev, t, len(sg.data))
			for k := range sg.data {
				sg.ack[k+1] = k < len(sg.data)-1 // the controller NACKs the last byte
			}
			data = append(data, sg.data...)
		} else {
			s.i2cWrite(dev, t, sg.data)
			for k := range sg.data {
				sg.ack[k+1] = true
			}
		}
	}
	_, end = s.sig.addI2C(t, segs)
	return data, ok, end
}

func (s *Sim) memRead(t int64, addr, reg byte, n int) ([]byte, bool, int64) {
	return s.i2cTransfer(t, []i2cSeg{{addr: addr, data: []byte{reg}}, {addr: addr, read: true, data: make([]byte, n)}})
}

func (s *Sim) memWrite(t int64, addr, reg, v byte) (bool, int64) {
	_, ok, end := s.i2cTransfer(t, []i2cSeg{{addr: addr, data: []byte{reg, v}}})
	return ok, end
}

// i2cRaw is env_i2c_raw: write, wait, read, each its own START..STOP.
func (s *Sim) i2cRaw(t int64, addr byte, w []byte, nr int, delay int64) ([]byte, bool) {
	at := t
	if len(w) > 0 {
		_, ok, end := s.i2cTransfer(at, []i2cSeg{{addr: addr, data: append([]byte(nil), w...)}})
		if !ok {
			return nil, false
		}
		at = end
	}
	at += delay
	if nr > 0 {
		r, ok, _ := s.i2cTransfer(at, []i2cSeg{{addr: addr, read: true, data: make([]byte, nr)}})
		return r, ok
	}
	return nil, true
}

// probe looks for the BMP280 at 0x76 then 0x77 the way env.c does: chip id, soft reset, the
// trim, then normal mode.
func (s *Sim) probe(t int64) (bool, int64) {
	d := &s.dut
	at := t
	d.env.probeID = 0
	for _, addr := range []byte{0x76, 0x77} {
		id, ok, end := s.memRead(at, addr, 0xD0, 1)
		at = end
		if !ok {
			continue
		}
		if id[0] != 0x58 {
			if d.env.probeID == 0 {
				d.env.probeID = id[0]
			}
			continue
		}
		if ok, end = s.memWrite(at, addr, 0xE0, 0xB6); !ok {
			at = end
			continue
		}
		at = end + 3*msNs
		cal, ok, end := s.memRead(at, addr, 0x88, 24)
		at = end
		if !ok {
			continue
		}
		if ok, end = s.memWrite(at, addr, 0xF5, 0x00); !ok {
			at = end
			continue
		}
		at = end
		if ok, end = s.memWrite(at, addr, 0xF4, 0x27); !ok {
			at = end
			continue
		}
		at = end
		d.calib = parseBmpCalib(cal)
		d.env.addr = addr
		return true, at
	}
	return false, at
}

func (s *Sim) readSample(t int64) (bool, int64) {
	d := &s.dut
	b, ok, end := s.memRead(t, d.env.addr, 0xF7, 6)
	if !ok {
		return false, end
	}
	adcP := int32(uint32(b[0])<<12 | uint32(b[1])<<4 | uint32(b[2])>>4)
	adcT := int32(uint32(b[3])<<12 | uint32(b[4])<<4 | uint32(b[5])>>4)
	if adcT == 0x80000 {
		return false, end
	}
	cdc, tFine := bmpTempCdc(d.calib, adcT)
	d.env.tempDc = cdc / 10
	d.env.pressPa = bmpPressPa(d.calib, adcP, tFine)
	return true, end
}

// envInit is env_init: set up I2C1 (the node's own pull-ups now hold the bus high) and probe.
// It returns when the bus is free again.
func (s *Sim) envInit(t int64) int64 {
	d := &s.dut
	d.i2cInit = true
	s.touchBase(t)
	d.env = envState{}
	d.failRun = 0
	ok, end := s.probe(t)
	if ok {
		d.env.status = envOK
	}
	return end
}

func (s *Sim) strobeEnv(start, end int64) {
	if s.dut.strobe == 3 {
		s.sig.addPulse(start, end, dutStrobe)
	}
}

func (s *Sim) envTick(t int64) {
	d := &s.dut
	if d.env.status != envOK {
		d.envNext = t + envProbeNs
		ok, end := s.probe(t)
		s.strobeEnv(t, end)
		if ok {
			d.env.status = envOK
			d.failRun = 0
			d.envNext = t + 20*msNs
			s.print(t, fmt.Sprintf("ENV: bmp280 found at 0x%02x\r\n", d.env.addr))
		}
		return
	}
	d.envNext = t + envPeriodNs
	ok, end := s.readSample(t)
	s.strobeEnv(t, end)
	if ok {
		d.failRun = 0
		d.env.reads++
		d.env.seq++
		return
	}
	d.env.fails++
	d.failRun++
	if d.failRun >= envFailsToLose {
		d.env.status = envLost
		d.env.seq++
	}
}

// printEnvBootLine is the one line the simulator adds to the node's boot log: what env_init
// found on the sensor bus. The real firmware probes silently and shows the result only in
// `alarm` (env=ok|absent); a boot log that says so makes the demo's "why is there no sensor
// reading?" question answerable from the log.
func (s *Sim) printEnvBootLine(t int64) {
	d := &s.dut
	switch {
	case d.env.status == envOK:
		s.print(t, fmt.Sprintf("ENV: bmp280 ok addr=0x%02x\r\n", d.env.addr))
	case d.env.probeID != 0:
		s.print(t, fmt.Sprintf("ENV: no BMP280: the device at 0x76/0x77 has chip id 0x%02x, not 0x58; retrying every 1 s\r\n", d.env.probeID))
	default:
		s.print(t, "ENV: no BMP280 at 0x76/0x77 (I2C1 PB8/PB9 NACK); retrying every 1 s\r\n")
	}
}

// -- alarms ----------------------------------------------------------------------------------

type alarmState struct {
	active  uint8
	limitDc int32
	cnt     [3]uint8
	events  uint32
}

func newAlarm() alarmState { return alarmState{limitDc: 500} }

func alarmName(bit uint8) string {
	switch bit {
	case alarmOvertemp:
		return "overtemp"
	case alarmPvFault:
		return "pv_fault"
	case alarmEnvLost:
		return "env_lost"
	}
	return "?"
}

type alarmIn struct {
	envOK, envLost, regulating bool
	envDc                      int32
	pvMv                       uint32
}

func (a *alarmState) eval(in alarmIn) uint8 {
	var want uint8
	if in.envOK {
		thr := a.limitDc
		if a.active&alarmOvertemp != 0 {
			thr -= 20
		}
		if in.envDc > thr {
			want |= alarmOvertemp
		}
	} else {
		want |= a.active & alarmOvertemp
	}
	if in.regulating && (in.pvMv < 300 || in.pvMv > 3100) {
		want |= alarmPvFault
	}
	if in.envLost {
		want |= alarmEnvLost
	}
	var changed uint8
	for i := 0; i < 3; i++ {
		bit := uint8(1) << i
		if want&bit != a.active&bit {
			a.cnt[i]++
			if a.cnt[i] >= 2 {
				a.active ^= bit
				changed |= bit
				a.cnt[i] = 0
			}
		} else {
			a.cnt[i] = 0
		}
	}
	if changed != 0 {
		a.events++
	}
	return changed
}

// alarmTick is main.c alarm_tick: evaluate once per new env reading, or on a timer of the same
// period while the sensor is not delivering.
func (s *Sim) alarmTick(t int64) {
	d := &s.dut
	e := &d.env
	switch {
	case e.seq != d.alEnvSeq:
		d.alEnvSeq = e.seq
		d.alNext = t + envPeriodNs
	case e.status != envOK && d.alNext <= t:
		d.alNext = t + envPeriodNs
	default:
		return
	}
	changed := d.alarm.eval(alarmIn{
		envOK: e.status == envOK && e.reads > 0, envLost: e.status == envLost, envDc: e.tempDc,
		regulating: d.thermo.on, pvMv: d.ain.mv,
	})
	if changed == 0 {
		return
	}
	d.alarmPin = d.alarm.active != 0
	s.touchBase(t)
	trip := d.alarm.active & (alarmOvertemp | alarmPvFault)
	if d.thermo.on && trip != 0 {
		d.thermo.on = false
		d.thermo.inBand = 0
		s.aoutOff(t)
		if trip&alarmOvertemp != 0 {
			d.trip = "overtemp"
		} else {
			d.trip = "pv_fault"
		}
	}
	s.pwrMark(t)
	if d.can.initOK {
		s.canSend(t) // frame 0x0A0 [active, changed, temp lo, temp hi]
	}
	for i := 0; i < 3; i++ {
		bit := uint8(1) << i
		if changed&bit == 0 {
			continue
		}
		what := "clear"
		if d.alarm.active&bit != 0 {
			what = "set"
		}
		s.print(t, fmt.Sprintf("\r\nEVT alarm %s=%s active=0x%02X\r\n", what, alarmName(bit), d.alarm.active))
	}
	if d.trip != "none" && !d.thermo.on && changed&trip != 0 {
		s.print(t, fmt.Sprintf("EVT ctl trip=%s heater=off\r\n", d.trip))
	}
}
