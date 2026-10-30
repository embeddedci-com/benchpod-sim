package benchsim

import (
	"fmt"
	"sort"
)

// laPin is one LA channel's owner, as the firmware's la_pins table keeps it: one function at a
// time, GPIO mode and commanded level when it is GPIO.
type laPin struct {
	fn    string // none, gpio, uart_rx, uart_tx, i2c_sda, i2c_scl
	mode  string // input, output, open_drain (fn == gpio)
	level int
}

// pullOhms and the direction of the fixed bias resistors: LA1-LA6 pull up, LA7/LA8 down.
var pullOhms = map[int]string{1: "4.7k", 2: "4.7k", 3: "2.2k", 4: "2.2k", 5: "10k", 6: "10k", 7: "10k", 8: "10k"}

func pullDown(la int) bool { return la == 7 || la == 8 }

// releaseHint is the firmware's "how to free it" text for a pin conflict.
func releaseHint(fn string, la int) string {
	switch fn {
	case "gpio":
		return fmt.Sprintf(`release it with {"cmd":"gpio","la":%d,"mode":"off"}`, la)
	case "uart_rx", "uart_tx":
		return "stop the uart proxy first"
	case "i2c_sda", "i2c_scl":
		return `stop the sensor emulation first ({"cmd":"sensor_stop"})`
	}
	return ""
}

func (s *Sim) requireLaVoltage() error {
	if s.pod.laMV == 0 {
		return fwError("la voltage not set; set it with la_voltage (mv 1800 or 3300) first")
	}
	return nil
}

// claim checks that every channel is free (or already owned by one of the replaceable
// functions) for fn, the way la_pins_check_claim does, and returns the firmware's conflict text.
func (s *Sim) checkClaim(fn string, las []int, replaceable ...string) error {
	for _, la := range las {
		if la < 1 || la > 14 {
			return errf("invalid la %d", la)
		}
		owner := s.pod.pins[la].fn
		free := owner == "none" || owner == fn && fn == "gpio"
		for _, r := range replaceable {
			free = free || owner == r
		}
		if !free {
			return errf("pin conflict: LA%d is in use by %s; %s", la, owner, releaseHint(owner, la))
		}
		if pullDown(la) && s.pod.pulls&(1<<(la-1)) != 0 {
			switch fn {
			case "uart_rx", "uart_tx":
				return errf("pull conflict: LA%d has its 10k pull-down engaged, which %s can't work with (the line idles high); disable it first", la, fn)
			case "i2c_sda", "i2c_scl":
				return errf("pull conflict: LA%d has its 10k pull-down engaged, which %s can't work with (an open-drain bus needs pull-ups); disable it first", la, fn)
			}
		}
	}
	return nil
}

func (s *Sim) releaseFn(fns ...string) {
	for la := 1; la <= 14; la++ {
		for _, fn := range fns {
			if s.pod.pins[la].fn == fn {
				s.pod.pins[la] = laPin{fn: "none"}
			}
		}
	}
}

func (s *Sim) pinEntry(la int) map[string]any {
	p := s.pod.pins[la]
	e := map[string]any{"la": la, "function": p.fn, "gpio": nil, "level": nil, "pull": nil}
	if p.fn == "gpio" {
		e["gpio"] = p.mode
		if p.mode != "input" {
			e["level"] = p.level
		}
	}
	if ohms, ok := pullOhms[la]; ok {
		dir := "up"
		if pullDown(la) {
			dir = "down"
		}
		e["pull"] = map[string]any{"dir": dir, "ohms": ohms, "on": s.pod.pulls&(1<<(la-1)) != 0}
	}
	return e
}

func (s *Sim) pinList(las []int) []any {
	out := []any{}
	for _, la := range las {
		out = append(out, s.pinEntry(la))
	}
	return out
}

func allChannels() []int {
	out := make([]int, 14)
	for i := range out {
		out[i] = i + 1
	}
	return out
}

func (s *Sim) cmdLaPins(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	return map[string]any{"pins": s.pinList(allChannels()), "levels": s.levelsNow(now)}, nil
}

// levelsNow is the live level of every channel as a bit mask (bit n = LA n+1).
func (s *Sim) levelsNow(t int64) int {
	return int(s.sig.wordAt(t))
}

func (s *Sim) cmdGpio(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.requireLaVoltage(); err != nil {
		return nil, err
	}
	now := s.now()
	s.advance(now)
	if !r.has("mode") && !r.has("level") {
		return map[string]any{"levels": s.levelsNow(now), "pins": s.pinList(allChannels())}, nil
	}
	las, all, err := r.channels("la")
	if err != nil {
		return nil, fwError(err.Error())
	}
	mode := r.str("mode")
	if all {
		if mode != "off" {
			return nil, fwError(`"all" only works with mode off`)
		}
		for la := 1; la <= 14; la++ {
			if s.pod.pins[la].fn == "gpio" {
				las = append(las, la)
			}
		}
	}
	for _, la := range las {
		if la < 1 || la > 14 {
			return nil, errf("invalid la %d", la)
		}
	}
	level := -1
	if r.has("level") {
		level = r.int("level", 0)
		if level != 0 && level != 1 {
			return nil, fwError("level must be 0 or 1")
		}
	}
	switch mode {
	case "off":
		for _, la := range las {
			if s.pod.pins[la].fn == "gpio" {
				s.pod.pins[la] = laPin{fn: "none"}
			}
		}
	case "input", "output", "open_drain":
		if mode == "input" && level >= 0 {
			return nil, fwError("level only applies to an output or open_drain pin")
		}
		if err := s.checkClaim("gpio", las); err != nil {
			return nil, err
		}
		for _, la := range las {
			if mode == "open_drain" && pullDown(la) && s.pod.pulls&(1<<(la-1)) != 0 {
				return nil, errf("pull conflict: LA%d has its 10k pull-down engaged, which gpio open_drain can't work with (a released line would read low); disable it first", la)
			}
		}
		lv := level
		if lv < 0 {
			lv = 0
			if mode == "open_drain" {
				lv = 1
			}
		}
		for _, la := range las {
			s.pod.pins[la] = laPin{fn: "gpio", mode: mode, level: lv}
		}
	case "":
		for _, la := range las {
			p := s.pod.pins[la]
			if p.fn != "gpio" || p.mode == "input" {
				return nil, errf("LA%d is not a gpio output; set a mode first", la)
			}
		}
		for _, la := range las {
			s.pod.pins[la].level = level
		}
	default:
		return nil, errf("unknown gpio mode %q", mode)
	}
	s.touchBase(now)
	sort.Ints(las)
	return map[string]any{"pins": s.pinList(las)}, nil
}

// cmdLa is the pull-resistor half of the firmware's `la` command (the step-train form is not
// simulated).
func (s *Sim) cmdLa(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.requireLaVoltage(); err != nil {
		return nil, err
	}
	if r.has("steps") {
		return nil, notInDemo("la step trains")
	}
	avail := b2i(s.pod.laMV == 3300)
	if !r.has("la") {
		return map[string]any{"la_pullup_mask": s.pod.pulls, "pullups_available": avail}, nil
	}
	la := r.int("la", 0)
	if la < 1 || la > 8 {
		return nil, fwError("no pull-up on this la")
	}
	if r.has("pullup") {
		v := r.str("pullup")
		on := v == "on" || v == "1" || v == "true"
		if on {
			if s.pod.laMV != 3300 {
				return nil, fwError("pull-ups are 3V3-referenced; not available with the LA bank at 1.8 V")
			}
			if pullDown(la) {
				switch fn := s.pod.pins[la].fn; fn {
				case "uart_rx", "uart_tx", "i2c_sda", "i2c_scl":
					return nil, errf("pull conflict: LA%d is used by %s, which can't work with the 10k pull-down (stop it first)", la, fn)
				}
			}
			s.pod.pulls |= 1 << (la - 1)
		} else {
			s.pod.pulls &^= 1 << (la - 1)
		}
		now := s.now()
		s.advance(now)
		s.touchBase(now)
	}
	dir := "up"
	if pullDown(la) {
		dir = "down"
	}
	return map[string]any{"la": la, "pullup": b2i(s.pod.pulls&(1<<(la-1)) != 0), "ohms": pullOhms[la],
		"pull": dir, "pullups_available": avail}, nil
}

// baseWord is the idle level of every LA channel right now: what the DUT's pins, the pod's
// drivers and the bias resistors make of each line when no UART byte or I2C transfer is on it.
func (s *Sim) baseWord() uint16 {
	var w uint16
	set := func(la int, v bool) {
		if v {
			w |= 1 << (la - 1)
		} else {
			w &^= 1 << (la - 1)
		}
	}
	d := &s.dut
	pulled := func(la int) bool { return !pullDown(la) && s.pod.pulls&(1<<(la-1)) != 0 && la <= 6 }
	for la := 1; la <= 14; la++ {
		set(la, pulled(la))
	}
	if d.railOn {
		// The DUT's pins: I2C1 idles high on the node's own pull-ups once it is set up, USART1 TX
		// idles high, RX (PA10) has its pull-up, PB0/PA8 are push-pull outputs, SWDIO is pulled
		// up and SWCLK down by the MCU in every state.
		set(dutSCL, d.i2cInit || pulled(dutSCL))
		set(dutSDA, d.i2cInit || pulled(dutSDA))
		set(dutTX, d.usartInit || pulled(dutTX))
		set(dutRX, d.usartInit || pulled(dutRX))
		if d.outputsInit {
			set(dutAlarm, d.alarmPin)
			set(dutStrobe, d.strobePin)
		}
		set(dutSWCLK, false)
		set(dutSWDIO, true)
	} else {
		// An unpowered DUT clamps its pins near ground through its protection diodes.
		for _, la := range []int{dutSCL, dutSDA, dutTX, dutRX, dutAlarm, dutStrobe, dutSWCLK, dutSWDIO} {
			set(la, false)
		}
	}
	if p := s.pod.proxy; p != nil {
		set(p.tx, true) // the pod's UART TX idles high
	}
	for la := 1; la <= 14; la++ {
		p := s.pod.pins[la]
		if p.fn != "gpio" {
			continue
		}
		switch p.mode {
		case "output":
			set(la, p.level == 1)
		case "open_drain":
			if p.level == 0 {
				set(la, false)
			}
		}
	}
	return w
}

// touchBase records the idle levels after a change at t. Callers hold s.mu.
func (s *Sim) touchBase(t int64) { s.sig.pushBase(t, s.baseWord()) }

// busHeldLow says whether the pod holds SCL or SDA of the DUT's sensor bus low (a GPIO output
// at 0 on it), which makes every I2C transfer of the node fail.
func (s *Sim) busHeldLow() bool {
	for _, la := range []int{dutSCL, dutSDA} {
		p := s.pod.pins[la]
		if p.fn == "gpio" && p.mode != "input" && p.level == 0 {
			return true
		}
	}
	return false
}
