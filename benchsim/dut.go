package benchsim

import "math"

// The DUT: a NUCLEO-F446RE running examples/process-node-stm32 (firmware "process-node"
// 0.2.0), wired to the pod as that example's README describes. Its firmware is a
// cooperative main loop over periodic tasks; the model replays that loop as discrete events in
// simulation time (each task at the millisecond its timer is due), so readings, alarms and the
// console come out in the order and at the moments the real node produces them.

// The bench wiring (README "Wiring"): DUT signal -> pod LA channel.
const (
	dutSCL    = 1  // PB8, I2C1 SCL (the emulated BMP280)
	dutSDA    = 2  // PB9, I2C1 SDA
	dutTX     = 3  // PA9, USART1 TX: the pod samples it (uart_rx)
	dutRX     = 4  // PA10, USART1 RX: the pod drives it (uart_tx)
	dutAlarm  = 5  // PB0, high while any alarm is active
	dutStrobe = 6  // PA8, task strobe
	dutSWCLK  = 11 // SWD
	dutSWDIO  = 12
)

const (
	dutBaud   = 115200
	dutByteNs = int64(1e9*10) / dutBaud
	msNs      = int64(1e6)
	// stlinkHoldNs is how long the board's ST-LINK keeps the F446 in reset after its rail comes
	// up. A real NUCLEO holds it about 2.2 s; the demo shortens that so a power cycle shows the
	// boot log about a second later, which keeps the default 4 s capture window comfortable.
	stlinkHoldNs = 800 * msNs
	// attachWindowNs is the node's FLASH_ATTACH_WINDOW_MS: it waits before touching peripherals.
	attachWindowNs = 200 * msNs
	// ffKeepNs: after a long idle gap the model skips ahead and replays only the last minute.
	ffKeepNs = int64(60e9)

	fwName    = "process-node"
	fwVersion = "0.2.0"
	fwBuild   = "Oct 28 2026 14:02:11"
)

type dutPhase int

const (
	phaseOff   dutPhase = iota // rail off
	phaseReset                 // powered, held in reset (ST-LINK, the pod's NRST, a reset in progress)
	phaseBoot                  // running main() up to the end of the boot attach window
	phaseRun                   // in the main loop
	phaseStall                 // `wdt stall`: spinning until the watchdog bites
)

type rxByte struct {
	at int64
	b  byte
}

type dut struct {
	railOn     bool
	phase      dutPhase
	releaseAt  int64 // phaseReset: when reset is released (0 with the pod holding NRST)
	mainAt     int64 // reset release: HAL tick 0
	bootAt     int64 // end of the attach window: peripherals come up
	cause      string
	nextCause  string
	bootMs     int64
	wdtAt      int64 // phaseStall: when the MCU resets (the IWDG bites, or a software reset)
	stallCause string

	usartInit, i2cInit, outputsInit bool
	alarmPin, strobePin             bool

	rx     []rxByte
	cmdBuf []byte
	lastCR bool

	ain      ainState
	ainNext  int64
	filt     uint32
	aout     aoutState
	thermo   thermoState
	thNext   int64
	trip     string
	env      envState
	envNext  int64
	failRun  int
	calib    bmpCalib
	alarm    alarmState
	alNext   int64
	alEnvSeq uint32
	strobe   int // 0 off, 1 ain, 2 ctl, 3 env
	can      canState

	pwr     []pwrEntry
	aoutLog []aoutEntry
}

func (d *dut) init() {
	d.cause = "unknown"
	d.trip = "none"
	d.can = canState{osc: 8000000, bitrate: 500000, print: true}
}

func (d *dut) tick(t int64) int64 { return (t - d.mainAt) / msNs }

// dutRailOn: the 5 V rail came up at t.
func (s *Sim) dutRailOn(t int64) {
	d := &s.dut
	d.railOn = true
	s.resetState(t)
	d.phase = phaseReset
	d.nextCause = "power-on"
	d.releaseAt = t + stlinkHoldNs
	s.pwrMark(t)
	s.touchBase(t)
}

func (s *Sim) dutRailOff(t int64) {
	d := &s.dut
	d.railOn = false
	d.phase = phaseOff
	s.resetState(t)
	s.pwrMark(t)
	s.touchBase(t)
}

func (s *Sim) dutResetAssert(t int64) {
	d := &s.dut
	if !d.railOn {
		return
	}
	if d.phase != phaseReset || d.nextCause != "power-on" {
		d.nextCause = "pin"
	}
	s.enterReset(t)
	d.releaseAt = 0
}

func (s *Sim) dutResetRelease(t int64) {
	d := &s.dut
	if !d.railOn || d.phase != phaseReset {
		return
	}
	if d.releaseAt == 0 || d.releaseAt < t {
		d.releaseAt = t
	}
}

// enterReset stops the MCU: every pin goes high-Z, RAM state is gone.
func (s *Sim) enterReset(t int64) {
	s.resetState(t)
	s.dut.phase = phaseReset
	s.pwrMark(t)
	s.touchBase(t)
}

func (s *Sim) resetState(t int64) {
	d := &s.dut
	d.usartInit, d.i2cInit, d.outputsInit = false, false, false
	d.alarmPin, d.strobePin = false, false
	d.rx, d.cmdBuf, d.lastCR = nil, nil, false
	d.thermo = thermoState{}
	d.strobe = 0
	d.trip = "none"
	d.can = canState{osc: 8000000, bitrate: 500000, print: true}
	s.setAout(t, aoutState{})
}

// nextEvent is when the node next has something to do; MaxInt64 when it waits on nothing.
func (s *Sim) dutNext() int64 {
	d := &s.dut
	never := int64(math.MaxInt64)
	switch d.phase {
	case phaseReset:
		if s.pod.nrst || d.releaseAt == 0 {
			return never
		}
		return d.releaseAt
	case phaseBoot:
		return d.bootAt
	case phaseStall:
		return d.wdtAt
	case phaseRun:
		n := d.ainNext
		if len(d.rx) > 0 && d.rx[0].at < n {
			n = d.rx[0].at
		}
		if d.thermo.on && d.thNext < n {
			n = d.thNext
		}
		if d.envNext < n {
			n = d.envNext
		}
		if d.env.status != envOK && d.alNext < n {
			n = d.alNext
		}
		if d.can.periodic.on && d.can.periodic.next < n {
			n = d.can.periodic.next
		}
		return n
	}
	return never
}

// dutAdvance runs the node up to t. Callers hold s.mu.
func (s *Sim) dutAdvance(t int64) {
	d := &s.dut
	for i := 0; ; i++ {
		next := s.dutNext()
		if next > t {
			return
		}
		if d.phase == phaseRun && len(d.rx) == 0 && next < t-ffKeepNs {
			s.fastForward(t - ffKeepNs - next)
			continue
		}
		s.dutStep(next)
		if i > 10_000_000 { // a model bug must not hang the pod
			return
		}
	}
}

// fastForward skips a long idle stretch: the timers move on and the counters count what the
// skipped passes would have done. The node's outputs do not change while nobody talks to it.
func (s *Sim) fastForward(delta int64) {
	d := &s.dut
	d.ain.samples += uint32(delta / (10 * msNs))
	d.ainNext += delta
	if d.thermo.on {
		d.thermo.steps += uint32(delta / (20 * msNs))
		d.thNext += delta
	}
	if d.env.status == envOK {
		n := uint32(delta / (200 * msNs))
		d.env.reads += n
		d.env.seq += n
		d.alEnvSeq = d.env.seq
		if e := s.sensorAt(d.env.addr); e != nil {
			e.xfers += 2 * int(n)
		}
	}
	d.envNext += delta
	d.alNext += delta
	if d.can.periodic.on {
		d.can.periodic.next += delta
	}
}

func (s *Sim) dutStep(t int64) {
	d := &s.dut
	switch d.phase {
	case phaseReset:
		d.phase = phaseBoot
		d.mainAt = t
		d.bootAt = t + attachWindowNs
		d.cause = d.nextCause
		if d.cause == "" {
			d.cause = "pin"
		}
		d.nextCause = "pin"
		s.pwrMark(t)
	case phaseBoot:
		s.boot(t)
	case phaseStall:
		d.nextCause = d.stallCause
		s.enterReset(t)
		d.releaseAt = t + msNs
	case phaseRun:
		s.mainLoopPass(t)
	}
}

// boot is main() from the end of the attach window to the prompt.
func (s *Sim) boot(t int64) {
	d := &s.dut
	d.usartInit = true
	s.touchBase(t)
	t += 50 * 1000 // setvbuf and the first printf: the line idles high before the first frame
	s.print(t, "\r\nPROCESS-NODE: stm32f446 + mcp2515\r\n")
	s.printInfo(t)
	s.print(t, "PROCESS-NODE: uart=USART1(115200), spi=SPI1 PA5/PA6/PA7, cs=PB6, int=PC7\r\n")
	s.canInit(t, d.can.bitrate)
	s.ainInit(t)
	s.setAout(t, aoutState{})
	d.thermo = thermoState{}
	end := s.envInit(t)
	d.alarm = newAlarm()
	d.alNext, d.alEnvSeq = 0, 0
	d.outputsInit = true
	d.alarmPin, d.strobePin = false, false
	s.touchBase(end)
	d.bootMs = (max(s.sig.dutTx.end, end) - d.mainAt) / msNs
	s.printEnvBootLine(t)
	s.print(t, "APP_OK\r\n")
	s.print(t, "type a command and press Enter (e.g. help)\r\n")
	s.print(t, "> ")
	d.phase = phaseRun
	d.ainNext = t + 10*msNs
	d.envNext = end
	s.pwrMark(t)
}

// mainLoopPass is one pass of the node's while(1) at t, in the firmware's order.
func (s *Sim) mainLoopPass(t int64) {
	d := &s.dut
	for len(d.rx) > 0 && d.rx[0].at <= t && d.phase == phaseRun {
		b := d.rx[0].b
		d.rx = d.rx[1:]
		s.rxByte(t, b)
	}
	if d.phase != phaseRun {
		return
	}
	if d.ainNext <= t {
		s.ainTick(t)
	}
	if d.thermo.on && d.thNext <= t {
		s.thermoTick(t)
	}
	if d.envNext <= t {
		s.envTick(t)
	}
	s.alarmTick(t)
	if d.can.periodic.on && d.can.periodic.next <= t {
		s.canPeriodicTick(t)
	}
}

// print sends console output: USART1 at 115200 8N1, after whatever is still going out.
func (s *Sim) print(t int64, text string) {
	if !s.dut.usartInit {
		return
	}
	s.sig.dutTx.write(t, dutTX, dutByteNs, []byte(text))
}

// podUartWrite is the pod's UART proxy sending data on channel ch at baud: the frames go on the
// wire, and the node receives them when that wire is its RX at its baud.
func (s *Sim) podUartWrite(t int64, ch, baud int, data []byte) {
	byteNs := int64(1e9 * 10 / float64(baud))
	start := s.sig.podTx.write(t, ch, byteNs, data)
	d := &s.dut
	if ch != dutRX || !d.usartInit || math.Abs(float64(baud)-dutBaud)/dutBaud > 0.03 {
		return
	}
	for k, b := range data {
		if len(d.rx) >= 255 { // the node's 256-byte RX ring is full: the byte is dropped
			break
		}
		d.rx = append(d.rx, rxByte{at: start + int64(k+1)*byteNs, b: b})
	}
}

type pwrEntry struct {
	t  int64
	ma float64
}

// loadMA is the whole NUCLEO board's draw from the 5 V rail: the ST-LINK, the F446 at 16 MHz on
// HSI sleeping in __WFI between passes, and the MCP2515/TJA1050 module.
func (s *Sim) loadMA() float64 {
	d := &s.dut
	switch d.phase {
	case phaseOff:
		return 0
	case phaseReset:
		return 31.6
	case phaseBoot:
		return 39.8
	case phaseStall:
		return 49.5 // spinning without __WFI
	}
	ma := 44.2
	if d.aout.mode != aoutOff {
		ma += 0.4
	}
	if d.thermo.on {
		ma += 1.3
	}
	if d.alarmPin {
		ma += 0.2
	}
	return ma
}

func (s *Sim) pwrMark(t int64) {
	d := &s.dut
	ma := s.loadMA()
	if n := len(d.pwr); n > 0 && d.pwr[n-1].ma == ma {
		return
	}
	d.pwr = append(d.pwr, pwrEntry{t, ma})
	// Keep ten minutes (the longest power profile) and at most 4096 changes.
	cut := 0
	for cut < len(d.pwr)-1 && d.pwr[cut+1].t < t-int64(610e9) {
		cut++
	}
	if len(d.pwr)-cut > 4096 {
		cut = len(d.pwr) - 4096
	}
	if cut > 0 {
		d.pwr = append([]pwrEntry(nil), d.pwr[cut:]...)
	}
}

func (s *Sim) loadAt(t int64) float64 {
	d := &s.dut
	ma := 0.0
	for _, e := range d.pwr {
		if e.t > t {
			break
		}
		ma = e.ma
	}
	return ma
}

// railSample is one INA238 reading of the internal rail at t: microamps and millivolts.
func (s *Sim) railSample(t int64) (int64, int64) {
	if !s.pod.efuse[1].enabled {
		return 0, 0
	}
	return s.railFromMA(s.loadAt(t))
}

// railFromMA is one noisy INA238 reading for a board drawing ma.
func (s *Sim) railFromMA(ma float64) (int64, int64) {
	if ma <= 0 {
		return 0, 0
	}
	ua := int64(math.Round(ma*1000 + s.noise(700)))
	if ua < 0 {
		ua = 0
	}
	mv := int64(math.Round(5012 - ma*0.09 + s.noise(3)))
	return ua, mv
}
