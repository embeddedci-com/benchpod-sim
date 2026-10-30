package benchsim

import (
	"fmt"
	"strconv"
	"strings"
)

// The node's UART console (main.c uart_process_rx_byte / process_command), reply for reply.

const cmdBufSize = 96

func (s *Sim) rxByte(t int64, b byte) {
	d := &s.dut
	switch {
	case b == '\r' || b == '\n':
		if b == '\n' && d.lastCR {
			d.lastCR = false
			return
		}
		d.lastCR = b == '\r'
		s.print(t, "\r\n")
		if len(d.cmdBuf) > 0 {
			cmd := string(d.cmdBuf)
			d.cmdBuf = d.cmdBuf[:0]
			s.processCommand(t, cmd)
			if d.phase == phaseRun {
				s.print(t, "> ")
			}
		} else {
			s.print(t, "> ")
		}
	case b == '\b' || b == 0x7F:
		if len(d.cmdBuf) > 0 {
			d.cmdBuf = d.cmdBuf[:len(d.cmdBuf)-1]
			s.print(t, "\b \b")
		}
	case b >= 32 && b <= 126:
		d.lastCR = false
		if len(d.cmdBuf) < cmdBufSize-1 {
			d.cmdBuf = append(d.cmdBuf, b)
			s.print(t, string(b))
		}
	}
}

// fields splits a command's arguments on spaces, like the firmware's strtok(" ").
func fields(s string) []string { return strings.Fields(s) }

func (s *Sim) processCommand(t int64, cmd string) {
	d := &s.dut
	switch {
	case cmd == "help":
		s.print(t, helpText)
	case cmd == "status":
		s.canPrintStatus(t)
	case cmd == "info":
		s.printInfo(t)
	case cmd == "strobe" || strings.HasPrefix(cmd, "strobe "):
		s.handleStrobe(t, cmd[6:])
	case cmd == "alarm" || strings.HasPrefix(cmd, "alarm "):
		s.handleAlarm(t, cmd[5:])
	case cmd == "ctl" || strings.HasPrefix(cmd, "ctl "):
		s.handleCtl(t, cmd[3:])
	case cmd == "aout" || strings.HasPrefix(cmd, "aout "):
		s.handleAout(t, cmd[4:])
	case cmd == "ain":
		a := s.ainReadNow(t)
		s.print(t, fmt.Sprintf("AIN mv=%d raw=%d filt_mv=%d vdda_mv=%d samples=%d\r\n", a.mv, a.raw, a.filtMv, a.vddaMv, a.samples))
	case cmd == "wdt stall":
		s.print(t, "WDT stall: waiting for the watchdog reset\r\n")
		d.phase = phaseStall
		d.stallCause = "iwdg"
		d.wdtAt = t + 2000*msNs // IWDG: LSI 32 kHz / 64, reload 1000
		s.pwrMark(t)
	case strings.HasPrefix(cmd, "can "):
		s.handleCan(t, cmd[4:])
	case strings.HasPrefix(cmd, "i2c "):
		s.handleI2C(t, cmd[4:])
	case cmd == "reset" || cmd == "reboot":
		s.print(t, "RESET: rebooting via NVIC_SystemReset()\r\n")
		// NVIC_SystemReset after the last byte has left the shift register.
		d.phase = phaseStall
		d.stallCause = "software"
		d.wdtAt = max(t, s.sig.dutTx.end)
	default:
		s.print(t, fmt.Sprintf("unknown command: %s (try help)\r\n", cmd))
	}
}

func (s *Sim) printInfo(t int64) {
	d := &s.dut
	s.print(t, fmt.Sprintf("INFO fw=%s version=%s build=\"%s\" uptime_ms=%d boot_ms=%d reset=%s\r\n",
		fwName, fwVersion, fwBuild, d.tick(t), d.bootMsShown(), d.cause))
}

// bootMsShown is g_boot_ms: 0 until the boot reached APP_OK.
func (d *dut) bootMsShown() int64 {
	if d.phase == phaseBoot {
		return 0
	}
	return d.bootMs
}

const helpText = "commands:\r\n" +
	"  info                      firmware, version, uptime, reset cause\r\n" +
	"  ain                       analog input PA1: mV, raw, filtered, VDDA\r\n" +
	"  aout [<mV> | sine <Hz> <amp-mV> <offset-mV> | off]   analog output PA4\r\n" +
	"  ctl [on <degC> | off]     thermostat: PA1 sensor (0.5 V + 20 mV/degC) -> PI -> PA4\r\n" +
	"  alarm [limit <degC>]      alarms, env sensor (BMP280), over-temperature limit\r\n" +
	"  strobe [off|ain|ctl|env]  PA8 marks that task: high while ain/env runs, toggles per ctl step\r\n" +
	"  i2c <addr> [w <b..>] [d <ms>] [r <n>]   raw transfer on the env bus (hex bytes)\r\n" +
	"  status | can status       counters, mode, error state\r\n" +
	"  can init [bitrate]        reset + configure the MCP2515 (default 500000)\r\n" +
	"  can osc <MHz>             module crystal (8 or 16), then re-init\r\n" +
	"  can mode normal|listen|loopback\r\n" +
	"  can send <id> [b0..b7]    hex; id > 7FF sends an extended frame\r\n" +
	"  can sendx <id> [b0..b7]   force an extended id\r\n" +
	"  can burst <n> <id>        n back-to-back frames, data = 16-bit counter\r\n" +
	"  can periodic <id> <ms> [count] | can periodic off\r\n" +
	"  can echo on|off           reply to every frame with id+1, same data\r\n" +
	"  can print on|off          print received frames (default on)\r\n" +
	"  can selftest              loop a frame inside the MCP2515\r\n" +
	"  can clear                 zero the counters\r\n" +
	"  can regs                  dump MCP2515 registers\r\n" +
	"  reset                     reboot the MCU\r\n" +
	"  wdt stall                 hang until the watchdog resets the MCU\r\n"

var strobeNames = []string{"off", "ain", "ctl", "env"}

func (s *Sim) handleStrobe(t int64, args string) {
	d := &s.dut
	if f := fields(args); len(f) > 0 {
		i := indexOf(strobeNames, f[0])
		if i < 0 {
			s.print(t, "STROBE error: usage strobe [off|ain|ctl|env]\r\n")
			return
		}
		d.strobe = i
		d.strobePin = false
		s.touchBase(t)
	}
	s.print(t, fmt.Sprintf("STROBE mode=%s\r\n", strobeNames[d.strobe]))
}

func indexOf(list []string, v string) int {
	for i, x := range list {
		if x == v {
			return i
		}
	}
	return -1
}

// parseU32 is the node's parse_u32: decimal digits only.
func parseU32(s string) (uint32, bool) {
	if s == "" {
		return 0, false
	}
	v, err := strconv.ParseUint(s, 10, 32)
	return uint32(v), err == nil
}

// parseDc is the node's parse_dc: "60", "60.5", "-5.5" in deci-degC, at most one decimal.
func parseDc(s string) (int32, bool) {
	if s == "" {
		return 0, false
	}
	neg := s[0] == '-'
	if neg {
		s = s[1:]
	}
	whole, frac, dot := strings.Cut(s, ".")
	if whole == "" {
		return 0, false
	}
	w, err := strconv.ParseUint(whole, 10, 32)
	if err != nil || w > 1000 {
		return 0, false
	}
	tenth := uint64(0)
	if dot {
		if len(frac) != 1 || frac[0] < '0' || frac[0] > '9' {
			return 0, false
		}
		tenth = uint64(frac[0] - '0')
	}
	v := int32(w*10 + tenth)
	if neg {
		v = -v
	}
	return v, true
}

func fmtDc(dc int32) string {
	a := dc
	sign := ""
	if a < 0 {
		a, sign = -a, "-"
	}
	return fmt.Sprintf("%s%d.%d", sign, a/10, a%10)
}

func (s *Sim) printAout(t int64) {
	a := s.dut.aout
	switch a.mode {
	case aoutDC:
		s.print(t, fmt.Sprintf("AOUT mode=dc mv=%d code=%d\r\n", a.mv, a.code))
	case aoutSine:
		s.print(t, fmt.Sprintf("AOUT mode=sine hz=%d amp_mv=%d offset_mv=%d\r\n", a.hz, a.ampMv, a.mv))
	default:
		s.print(t, "AOUT mode=off\r\n")
	}
}

func (s *Sim) handleAout(t int64, args string) {
	d := &s.dut
	f := fields(args)
	if len(f) == 0 {
		s.printAout(t)
		return
	}
	if d.thermo.on {
		d.thermo.on = false
		d.thermo.inBand = 0
		s.pwrMark(t)
		s.print(t, "CTL off: manual output\r\n")
	}
	if f[0] == "off" {
		s.aoutOff(t)
		s.printAout(t)
		return
	}
	if f[0] == "sine" {
		var v [3]uint32
		ok := len(f) >= 4
		for i := 0; ok && i < 3; i++ {
			v[i], ok = parseU32(f[1+i])
		}
		if !ok || !s.aoutSine(t, v[0], v[1], v[2]) {
			s.print(t, fmt.Sprintf("AOUT error: usage aout sine <1..%d Hz> <amp-mV> <offset-mV>, swing within %d mV .. VDDA-%d mV\r\n",
				aoutSineMaxHz, aoutMinMv, aoutHeadroomMv))
			return
		}
		s.printAout(t)
		return
	}
	mv, ok := parseU32(f[0])
	if !ok || !s.aoutDC(t, mv) {
		s.print(t, fmt.Sprintf("AOUT error: usage aout <mV> (%d .. VDDA-%d) | sine ... | off\r\n", aoutMinMv, aoutHeadroomMv))
		return
	}
	s.printAout(t)
}

func (s *Sim) printCtl(t int64) {
	d := &s.dut
	out := int32(0)
	if d.thermo.on {
		out = d.thermo.outMv
	}
	s.print(t, fmt.Sprintf("CTL mode=%s sp_c=%s pv_c=%s out_mv=%d in_band_ms=%d steps=%d trip=%s\r\n",
		onOff(d.thermo.on), fmtDc(d.thermo.spDc), fmtDc(mvToDc(d.ain.mv)), out,
		d.thermo.inBand*thermoPeriodMs, d.thermo.steps, d.trip))
}

func onOff(b bool) string {
	if b {
		return "on"
	}
	return "off"
}

func (s *Sim) handleCtl(t int64, args string) {
	d := &s.dut
	f := fields(args)
	if len(f) == 0 {
		s.printCtl(t)
		return
	}
	if f[0] == "off" {
		d.thermo.on = false
		d.thermo.inBand = 0
		s.aoutOff(t)
		s.pwrMark(t)
		s.printCtl(t)
		return
	}
	var sp int32
	ok := f[0] == "on" && len(f) >= 2
	if ok {
		sp, ok = parseDc(f[1])
	}
	if !ok {
		s.print(t, "CTL error: usage ctl on <degC 0..100, one decimal> | ctl off\r\n")
		return
	}
	if d.alarm.active&alarmOvertemp != 0 {
		s.print(t, "CTL error: over-temperature alarm active\r\n")
		return
	}
	current := int32(thermoOutMinMv)
	if d.aout.mode == aoutDC {
		current = int32(d.aout.mv)
	}
	wasOn := d.thermo.on
	if !d.thermo.start(sp, current) {
		s.print(t, "CTL error: setpoint out of range 0.0..100.0 degC\r\n")
		return
	}
	d.trip = "none"
	if !wasOn {
		s.aoutDC(t, uint32(d.thermo.outMv))
		d.thNext = t
	}
	s.pwrMark(t)
	s.printCtl(t)
}

func (s *Sim) printAlarm(t int64) {
	d := &s.dut
	a := d.alarm
	envNames := []string{"absent", "ok", "lost"}
	s.print(t, fmt.Sprintf("ALARM active=0x%02X overtemp=%d pv_fault=%d env_lost=%d limit_c=%s env=%s env_c=%s press_pa=%d events=%d\r\n",
		a.active, b2i(a.active&alarmOvertemp != 0), b2i(a.active&alarmPvFault != 0), b2i(a.active&alarmEnvLost != 0),
		fmtDc(a.limitDc), envNames[d.env.status], fmtDc(d.env.tempDc), d.env.pressPa, a.events))
}

func (s *Sim) handleAlarm(t int64, args string) {
	d := &s.dut
	f := fields(args)
	if len(f) == 0 {
		s.printAlarm(t)
		return
	}
	var lim int32
	ok := f[0] == "limit" && len(f) >= 2
	if ok {
		lim, ok = parseDc(f[1])
	}
	if !ok || lim < alarmLimitMinDc || lim > alarmLimitMaxDc {
		s.print(t, "ALARM error: usage alarm [limit <degC -40..85, one decimal>]\r\n")
		return
	}
	d.alarm.limitDc = lim
	s.printAlarm(t)
}

// handleI2C is the raw `i2c <addr> [w <b..>] [d <ms>] [r <n>]` transfer on the sensor bus.
func (s *Sim) handleI2C(t int64, args string) {
	f := fields(args)
	usage := "I2C err usage: i2c <addr> [w <b..>] [d <ms>] [r <n>]\r\n"
	if len(f) == 0 {
		s.print(t, usage)
		return
	}
	addr, _ := strconv.ParseUint(f[0], 16, 32)
	var w []byte
	nr, delayMs := 0, uint64(0)
	mode := byte(0)
	for _, tok := range f[1:] {
		if tok == "w" || tok == "d" || tok == "r" {
			mode = tok[0]
			continue
		}
		switch mode {
		case 'w':
			if len(w) < 32 {
				v, _ := strconv.ParseUint(tok, 16, 32)
				w = append(w, byte(v))
			}
		case 'd':
			delayMs, _ = strconv.ParseUint(tok, 10, 32)
		case 'r':
			n, _ := strconv.ParseUint(tok, 10, 32)
			nr = int(min(n, 32))
		}
	}
	if addr > 0x7F || (len(w) == 0 && nr == 0) {
		s.print(t, usage)
		return
	}
	r, ok := s.i2cRaw(t, byte(addr), w, nr, int64(delayMs)*msNs)
	if !ok {
		s.print(t, "I2C err\r\n")
		return
	}
	hex := make([]string, len(r))
	for i, b := range r {
		hex[i] = fmt.Sprintf("%02x", b)
	}
	s.print(t, "I2C ok r="+strings.Join(hex, " ")+"\r\n")
}
