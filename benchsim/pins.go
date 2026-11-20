package benchsim

import (
	"sort"
)

// laPin is one LA channel's owner, as the firmware's la_pins table keeps it: one function at a
// time, GPIO mode and commanded level when it is GPIO.
type laPin struct {
	fn    string // none, gpio, uart_rx, uart_tx, i2c_sda, i2c_scl, sensor_*, spi_*
	mode  string // input, output, open_drain (fn == gpio)
	level int
}

// pullOhms and the direction of the fixed bias resistors: LA1-LA6 pull up, LA7/LA8 down.
var pullOhms = map[int]string{1: "4.7k", 2: "4.7k", 3: "2.2k", 4: "2.2k", 5: "10k", 6: "10k", 7: "10k", 8: "10k"}

func pullDown(la int) bool { return la == 7 || la == 8 }

func (s *Sim) requireLaVoltage() error {
	if s.pod.laMV == 0 {
		return coded(codeLaVoltageNotSet)
	}
	return nil
}

// pinInUse is BP0031 for a pin another function holds: owner is "gpio" or "sensor:env (i2c_sda)".
func (s *Sim) pinInUse(la int) error {
	inst := ""
	if on := s.sensorsOn(la); len(on) > 0 {
		inst = on[0].name
	}
	return coded(codeLaPinInUse, "la", la, "owner", s.pinOwnerText(la), "function", s.pod.pins[la].fn, "instance", inst)
}

// noPullDown is why a function cannot share its line with an engaged pull-down (fn_attr's
// no_pull_down), "" when it can.
func noPullDown(fn string) string {
	switch fn {
	case "uart_rx", "uart_tx", "sensor_tx", "sensor_rx":
		return "the line idles high"
	case "i2c_sda", "i2c_scl":
		return "an open-drain bus needs pull-ups"
	case "gpio open_drain":
		return "a released line would read low"
	}
	return ""
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
			return s.pinInUse(la)
		}
		if pullDown(la) && s.pod.pulls&(1<<(la-1)) != 0 {
			if why := noPullDown(fn); why != "" {
				return coded(codeLaPullDownEngaged, "la", la, "ohms", pullOhms[la], "function", fn, "reason", why)
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
	if owner := s.pinOwner(la); owner != "" {
		e["owner"] = owner
	}
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
		e["pull"] = map[string]any{"dir": dir, "ohms": ohms, "on": s.pod.pulls&(1<<(la-1)) != 0, "dirs": []string{dir}}
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
				return nil, coded(codeLaPullDownEngaged, "la", la, "ohms", pullOhms[la], "function", "gpio open_drain",
					"reason", noPullDown("gpio open_drain"))
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

// cmdLaStep is la_step: a step pulse train (firmware up to 3.10.13: la with steps). The
// simulator checks the request and does not run the train.
func (s *Sim) cmdLaStep(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.requireLaVoltage(); err != nil {
		return nil, err
	}
	if !r.has("steps") {
		return nil, fwError("missing steps")
	}
	if s.firmware {
		return nil, fwError("la_step is not modeled by the simulator")
	}
	return nil, notInDemo("la_step")
}

// pullDirs is every way LA la can pull: this board has one fixed resistor per channel on LA1-LA8.
func pullDirs(la int) []string {
	switch {
	case la < 1 || la > 8:
		return []string{}
	case pullDown(la):
		return []string{"down"}
	}
	return []string{"up"}
}

// cmdLaPull is la_pull (firmware up to 3.10.13: la without steps): the pull bitmask without
// "la", a channel's pull state with it, and "pull" up, down or off switches it first.
func (s *Sim) cmdLaPull(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.requireLaVoltage(); err != nil {
		return nil, err
	}
	avail := s.pod.laMV != 0
	if !r.has("la") {
		return map[string]any{"la_pullup_mask": s.pod.pulls, "pullups_available": avail}, nil
	}
	la := r.int("la", 0)
	if la < 1 || la > 14 {
		return nil, errf("LA%d is not a pin (use 1..14)", max(la, 0))
	}
	dirs := pullDirs(la)
	if r.has("pull") {
		want := r.str("pull")
		if want != "up" && want != "down" && want != "off" {
			return nil, fwError("pull must be up, down or off")
		}
		if want != "off" {
			if len(dirs) == 0 {
				return nil, coded(codeLaNoPull, "la", la)
			}
			if want != dirs[0] {
				return nil, coded(codeLaPullDirection, "la", la, "want", want, "have", dirs[0])
			}
			if want == "down" {
				fn := s.pod.pins[la].fn
				if fn == "gpio" && s.pod.pins[la].mode == "open_drain" {
					fn = "gpio open_drain"
				}
				if why := noPullDown(fn); why != "" {
					return nil, coded(codeLaPullDownConflict, "la", la, "ohms", pullOhms[la], "function", fn, "reason", why)
				}
			}
			s.pod.pulls |= 1 << (la - 1)
		} else if len(dirs) > 0 {
			s.pod.pulls &^= 1 << (la - 1)
		}
		now := s.now()
		s.advance(now)
		s.touchBase(now)
	}
	state := "off"
	if len(dirs) > 0 && s.pod.pulls&(1<<(la-1)) != 0 {
		state = dirs[0]
	}
	return map[string]any{"la": la, "state": state, "dirs": dirs, "pullups_available": avail}, nil
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
	now := s.now()
	for la := 1; la <= 14; la++ {
		if s.sensorHoldsSDA(la, now) {
			set(la, false)
		}
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
// at 0 on it, or an emulated sensor's bus_stuck fault), which makes every I2C transfer of the
// node fail.
func (s *Sim) busHeldLow(t int64) bool {
	if s.sensorHoldsSDA(dutSDA, t) {
		return true
	}
	for _, la := range []int{dutSCL, dutSDA} {
		p := s.pod.pins[la]
		if p.fn == "gpio" && p.mode != "input" && p.level == 0 {
			return true
		}
	}
	return false
}
