package benchsim

import (
	"fmt"
	"strconv"
	"strings"
)

// The node's MCP2515 CAN console (main.c handle_can). The module answers on SPI, but in the demo
// nothing else on the bus acknowledges a frame (the pod's CAN is not part of the simulation),
// so every transmission in normal mode times out the way an unacknowledged MCP2515 does: TEC
// climbs by 8 per frame up to error passive. Loopback mode and the self-test work.

type canPeriodic struct {
	on          bool
	id          uint32
	periodMs    uint32
	next        int64
	sent, limit uint32
}

type canState struct {
	initOK               bool
	osc, bitrate         uint32
	mode                 string
	echo, print          bool
	rx, tx, txFail, rxOv uint32
	tec                  uint32
	periodic             canPeriodic
}

type canTiming struct {
	tq, prseg, ps1, ps2, cnf1, cnf2, cnf3 uint32
}

// canCalcTiming is mcp2515_calc_timing.
func canCalcTiming(osc, bitrate uint32) (canTiming, bool) {
	if osc == 0 || bitrate == 0 {
		return canTiming{}, false
	}
	for brp := uint32(0); brp < 64; brp++ {
		div := 2 * (brp + 1) * bitrate
		if div > osc {
			return canTiming{}, false
		}
		if osc%div != 0 {
			continue
		}
		n := osc / div
		if n > 25 {
			continue
		}
		if n < 8 {
			return canTiming{}, false
		}
		sp := (n*8 + 5) / 10
		ps2 := n - sp
		if ps2 < 2 {
			ps2 = 2
		}
		tseg1 := n - 1 - ps2
		prseg := tseg1 / 2
		ps1 := tseg1 - prseg
		if ps1 > 8 {
			prseg += ps1 - 8
			ps1 = 8
		}
		if prseg > 8 || ps2 > 8 || prseg+ps1 < ps2 {
			continue
		}
		return canTiming{tq: n, prseg: prseg, ps1: ps1, ps2: ps2, cnf1: brp,
			cnf2: 0x80 | (ps1-1)<<3 | (prseg - 1), cnf3: ps2 - 1}, true
	}
	return canTiming{}, false
}

func (s *Sim) canInit(t int64, bitrate uint32) {
	c := &s.dut.can
	c.initOK = false
	c.periodic.on = false
	tm, ok := canCalcTiming(c.osc, bitrate)
	if !ok {
		s.print(t, fmt.Sprintf("CAN init fail: bitrate %d not possible with a %d Hz crystal\r\n", bitrate, c.osc))
		return
	}
	c.bitrate = bitrate
	c.initOK = true
	c.mode = "normal"
	c.tec = 0
	s.print(t, fmt.Sprintf("CAN init ok bitrate=%d osc=%d tq=%d sp=%d%% cnf=%02X %02X %02X\r\n",
		bitrate, c.osc, tm.tq, (1+tm.prseg+tm.ps1)*100/tm.tq, tm.cnf1, tm.cnf2, tm.cnf3))
}

func (c *canState) eflg() uint32 {
	var e uint32
	if c.tec >= 96 {
		e |= 0x05 // TXWAR | EWARN
	}
	if c.tec >= 128 {
		e |= 0x10 // TXEP
	}
	return e
}

// canSend transmits one frame: OK in loopback, an ACK timeout on a bus nobody acknowledges.
func (s *Sim) canSend(int64) bool {
	c := &s.dut.can
	if c.mode == "loopback" {
		c.tx++
		return true
	}
	c.txFail++
	if c.mode == "normal" && c.tec < 128 {
		c.tec += 8
	}
	return false
}

func (s *Sim) canPrintStatus(t int64) {
	c := &s.dut.can
	e := c.eflg()
	init := "fail"
	if c.initOK {
		init = "ok"
	}
	mode := c.mode
	if mode == "" {
		mode = "config"
	}
	s.print(t, fmt.Sprintf("CAN status: chip=ok init=%s mode=%s bitrate=%d osc=%d tec=%d rec=0 eflg=0x%02X busoff=0 errpassive=%d rx=%d tx=%d txfail=%d rxovr=%d echo=%s print=%s periodic=%s\r\n",
		init, mode, c.bitrate, c.osc, c.tec, e, b2i(e&0x10 != 0), c.rx, c.tx, c.txFail, c.rxOv,
		onOff(c.echo), onOff(c.print), onOff(c.periodic.on)))
}

func (s *Sim) canPeriodicTick(t int64) {
	p := &s.dut.can.periodic
	p.next += int64(p.periodMs) * msNs
	if p.next <= t {
		p.next = t + int64(p.periodMs)*msNs
	}
	s.canSend(t)
	p.sent++
	if p.limit != 0 && p.sent >= p.limit {
		p.on = false
		s.print(t, fmt.Sprintf("CAN periodic done sent=%d\r\n", p.sent))
	}
}

func parseFrame(rest string, forceExt bool) (id uint64, ext bool, dlc int, ok bool) {
	f := fields(rest)
	if len(f) == 0 {
		return 0, false, 0, false
	}
	id, err := strconv.ParseUint(f[0], 16, 32)
	if err != nil || id > 0x1FFFFFFF || len(f)-1 > 8 {
		return 0, false, 0, false
	}
	for _, b := range f[1:] {
		if v, err := strconv.ParseUint(b, 16, 8); err != nil || v > 0xFF {
			return 0, false, 0, false
		}
	}
	return id, forceExt || id > 0x7FF, len(f) - 1, true
}

func (s *Sim) handleCan(t int64, args string) {
	c := &s.dut.can
	sub, rest, _ := strings.Cut(args, " ")
	rest = strings.TrimLeft(rest, " ")
	switch sub {
	case "init":
		br := c.bitrate
		if rest != "" {
			v, _ := strconv.ParseUint(fields(rest)[0], 10, 32)
			br = uint32(v)
		}
		s.canInit(t, br)
		return
	case "osc":
		mhz, _ := strconv.ParseUint(firstField(rest), 10, 32)
		if mhz == 0 || mhz > 40 {
			s.print(t, "usage: can osc <crystal-MHz>  (8 or 16 on most modules)\r\n")
			return
		}
		c.osc = uint32(mhz) * 1000000
		s.canInit(t, c.bitrate)
		return
	case "status":
		s.canPrintStatus(t)
		return
	}
	if !c.initOK {
		s.print(t, "CAN not initialized: run \"can init <bitrate>\"\r\n")
		return
	}
	switch sub {
	case "send", "sendx":
		id, _, dlc, ok := parseFrame(rest, sub == "sendx")
		if !ok {
			s.print(t, "usage: can send <id-hex> [byte-hex ...]  (max 8 bytes)\r\n")
			return
		}
		if s.canSend(t) {
			s.print(t, fmt.Sprintf("CAN tx ok id=0x%X dlc=%d\r\n", id, dlc))
		} else {
			s.print(t, fmt.Sprintf("CAN tx fail: timeout (no ACK?) tec=%d\r\n", c.tec))
		}
	case "burst":
		f := fields(rest)
		var n, id uint64
		if len(f) >= 2 {
			n, _ = strconv.ParseUint(f[0], 10, 32)
			id, _ = strconv.ParseUint(f[1], 16, 32)
		}
		if n == 0 || n > 1000 || len(f) < 2 || id > 0x1FFFFFFF {
			s.print(t, "usage: can burst <count 1..1000> <id-hex>\r\n")
			return
		}
		sent := uint64(0)
		for i := uint64(0); i < n; i++ {
			if !s.canSend(t) {
				break
			}
			sent++
		}
		ms := 0
		if sent < n {
			ms = 50 // one ACK timeout before it gives up
		}
		s.print(t, fmt.Sprintf("CAN burst done sent=%d failed=%d ms=%d\r\n", sent, n-sent, ms))
	case "periodic":
		f := fields(rest)
		if len(f) > 0 && f[0] == "off" {
			c.periodic.on = false
			s.print(t, fmt.Sprintf("CAN periodic off sent=%d\r\n", c.periodic.sent))
			return
		}
		var id, ms, n uint64
		var err error = nil
		if len(f) >= 2 {
			id, err = strconv.ParseUint(f[0], 16, 32)
			ms, _ = strconv.ParseUint(f[1], 10, 32)
		}
		if len(f) >= 3 {
			n, _ = strconv.ParseUint(f[2], 10, 32)
		}
		if len(f) < 2 || err != nil || ms == 0 || id > 0x1FFFFFFF {
			s.print(t, "usage: can periodic <id-hex> <period-ms> [count] | can periodic off\r\n")
			return
		}
		c.periodic = canPeriodic{on: true, id: uint32(id), periodMs: uint32(ms), next: t, limit: uint32(n)}
		s.print(t, fmt.Sprintf("CAN periodic on id=0x%X period=%dms count=%d\r\n", id, ms, n))
	case "echo", "print":
		v := strings.TrimSpace(rest)
		if v != "on" && v != "off" {
			s.print(t, fmt.Sprintf("usage: can %s on|off\r\n", sub))
			return
		}
		if sub == "echo" {
			c.echo = v == "on"
		} else {
			c.print = v == "on"
		}
		s.print(t, fmt.Sprintf("CAN %s %s\r\n", sub, v))
	case "mode":
		m := strings.TrimSpace(rest)
		if m != "normal" && m != "listen" && m != "loopback" {
			s.print(t, "usage: can mode normal|listen|loopback\r\n")
			return
		}
		c.mode = m
		s.print(t, fmt.Sprintf("CAN mode %s ok\r\n", m))
	case "selftest":
		s.print(t, "CAN selftest ok\r\n")
	case "clear":
		c.rx, c.tx, c.txFail, c.rxOv = 0, 0, 0, 0
		s.print(t, "CAN counters cleared\r\n")
	case "regs":
		canstat := map[string]uint32{"normal": 0x00, "listen": 0x60, "loopback": 0x40}[c.mode]
		tm, _ := canCalcTiming(c.osc, c.bitrate)
		s.print(t, fmt.Sprintf("CAN regs: canstat=%02X canctrl=%02X cnf1=%02X cnf2=%02X cnf3=%02X caninte=03 canintf=00 eflg=%02X tec=%d rec=0\r\n",
			canstat, canstat|0x07, tm.cnf1, tm.cnf2, tm.cnf3, c.eflg(), c.tec))
	default:
		s.print(t, fmt.Sprintf("unknown can command: %s (try help)\r\n", sub))
	}
}

func firstField(s string) string {
	if f := fields(s); len(f) > 0 {
		return f[0]
	}
	return ""
}
