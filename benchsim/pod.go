package benchsim

import (
	"math"
	"sort"
	"time"
)

// What the simulated pod reports about itself: a rev3 analog BenchPod on current firmware.
const (
	podFirmware = "3.9.4"
	podBoard    = "stm32h563"
	podGateware = 48
	// The front-SMA ADC fit (cal_data.h ADC_CAL_EXT): volts = a + b*count, 16-bit unwrapped.
	adcCalA = 65.708158
	adcCalB = -0.0010026443
)

// podCaps is the caps[] list of status: the features the simulator implements. The SDK turns a
// missing flag into a clear "needs newer pod firmware" refusal, so leaving out what is not
// simulated (CAN, GPS, SPI, OTA, calibration, the 4-20 mA output) fails cleanly on the host.
var podCaps = []string{
	"scope", "la", "uart", "analog", "command", "dac", "dac_dc", "generate16", "la_pins",
	"gpio_read", "capture_trigger", "power_profile", "capture_b64", "nrst_pin", "sensor_types",
	"pod_current",
}

type efuseState struct{ enabled, fault bool }

// podEvent is a target_power change the pod scheduled with delay_ms.
type podEvent struct {
	at    int64
	efuse int
	on    bool
}

// podState is everything the pod itself holds (the DUT is in dut).
type podState struct {
	laMV    int // 0 = not selected yet
	efuse   [3]efuseState
	pending []podEvent
	nrst    bool // the pod holds the DUT in reset
	pulls   uint16
	pins    [15]laPin
	sensor  *emuSensor
	proxy   *uartProxy
	capture bool // a capture is running (the pod runs one at a time)
	analog  analogState
	profile *powerSession

	// The firmware command layer (firmware.go, dac.go, can.go, spi.go).
	lease            leaseState
	heavy            *conn // owner of the capture hardware (heavy_owner)
	otaBusy          bool
	image            int // running gateware image: 0 loop, 1 deep replay
	load             *loadBin
	trace            []uint16 // the last upload, what replay plays
	tracePsram       bool
	staged           []byte // the bytes of the last load_bin with "psram"
	lut              [2048]uint16
	loopSrc          string
	loopIn, loopStep int
	lastCap          *lastCapture
	can              podCan
	spi              *spiMaster
	gps              *gpsState
}

type leaseState struct {
	held   bool
	holder string
	until  time.Time
}

// podFlash is what the firmware keeps in flash across a reset.
type podFlash struct {
	lanPolicy, sigPolicy string
	limits               *dacLimits
	proxyHost            string
	proxyPort            int
	proxyUser, proxyPass string
	caCerts              []map[string]any
	calOffsetUV          int64
	calSet               bool
	dacCal               [3]*dacLine
	adcGain              [3]*float64
	chip                 spiFlashChip
}

func (p *podState) init() {
	for la := range p.pins {
		p.pins[la] = laPin{fn: "none"}
	}
	p.analog.adcSource = "ext"
	p.analog.dac = dacState{path: "off"}
	p.loopSrc = "adc"
}

// advance runs everything due up to t: the pod's scheduled power changes and the DUT, in time
// order, so a delayed power-on lands at its moment in the DUT's timeline. Callers hold s.mu.
func (s *Sim) advance(t int64) {
	for {
		next := int64(math.MaxInt64)
		if len(s.pod.pending) > 0 {
			next = s.pod.pending[0].at
		}
		if next <= t {
			// The DUT first catches up to the power event, then the event applies.
			s.dutAdvance(next)
			ev := s.pod.pending[0]
			s.pod.pending = s.pod.pending[1:]
			s.applyPower(ev.at, ev.efuse, ev.on)
			continue
		}
		break
	}
	s.dutAdvance(t)
	s.sig.prune(t)
	s.loopAdvance(t)
}

func (s *Sim) applyPower(t int64, efuse int, on bool) {
	was := s.pod.efuse[efuse].enabled
	s.pod.efuse[efuse].enabled = on
	if was != on && s.hooks.Efuse != nil {
		go s.hooks.Efuse(efuse, on, false)
	}
	if efuse != 1 || was == on {
		return
	}
	// The DUT is on the internal 5 V rail (eFuse 1); eFuse 2 has nothing connected.
	if on {
		s.dutRailOn(t)
	} else {
		s.dutRailOff(t)
	}
}

func (s *Sim) cmdStatus(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.advance(s.now())
	return map[string]any{
		"device": "benchpod", "version": podFirmware, "board": podBoard, "board_rev": "v3",
		"board_rev_mv": 1650, "net": "connected", "ip": "127.0.0.1", "rssi_dbm": nil,
		"wifi": "off", "cloud": "off",
		"adc_bits": 16, "adc_fullscale_mv": 4096, "adc_channels": 1, "la_vccio_mv": s.pod.laMV,
		"adc_cal_a_uv": int64(math.Round(adcCalA * 1e6)), "adc_cal_b_nv": int64(math.Round(adcCalB * 1e9)),
		"adc_cal_unwrap": true, "nrst_pin": true, "flash_kb": 2048, "analog": true,
		"lan_policy": "open", "lan_policy_cmd": true, "lease_state": true,
		"lease":    map[string]any{"held": false, "holder": "", "left_s": 0},
		"gateware": podGateware, "gateware_embedded": podGateware,
		"uptime_ms": s.now() / 1e6, "reset": "power-on", "last_crash": "none", "safe_mode": false,
		"caps":      podCaps,
		"simulator": "benchsim: a simulated BenchPod wired to a simulated NUCLEO-F446RE process node",
	}, nil
}

func (s *Sim) cmdLaVoltage(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if r.has("mv") {
		mv := r.int("mv", 0)
		if mv != 1800 && mv != 3300 {
			return nil, fwError("la voltage must be 1800 or 3300 (mv)")
		}
		if mv != s.pod.laMV && s.pod.laMV != 0 {
			for la := 1; la <= 14; la++ {
				if fn := s.pod.pins[la].fn; fn != "none" && fn != "gpio" {
					return nil, errf("la voltage change refused: LA%d is in use by %s; %s", la, fn, releaseHint(fn, la))
				}
			}
		}
		s.pod.laMV = mv
		if mv == 1800 {
			s.pod.pulls = 0 // the resistors are 3V3-referenced: the pod drops them at 1.8 V
		}
		s.advance(s.now())
		s.touchBase(s.now())
	}
	return map[string]any{"mv": s.pod.laMV, "st": 1, "readback_mv": s.pod.laMV}, nil
}

func (s *Sim) cmdTargetPower(r request) (any, error) {
	n := r.int("efuse", 0)
	if n != 1 && n != 2 {
		return nil, fwError("efuse must be 1 or 2")
	}
	on := r.flag("state")
	delay := r.int("delay_ms", 0)
	if delay < 0 || delay > 600000 {
		return nil, fwError("delay_ms out of range")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	// A new command for a rail replaces a change still pending on it (the pod has one timer
	// per rail).
	kept := s.pod.pending[:0]
	for _, ev := range s.pod.pending {
		if ev.efuse != n {
			kept = append(kept, ev)
		}
	}
	s.pod.pending = kept
	if delay > 0 {
		if s.firmware {
			time.AfterFunc(time.Duration(delay)*time.Millisecond, func() {
				s.mu.Lock()
				s.advance(s.now())
				s.mu.Unlock()
			})
		}
		s.pod.pending = append(s.pod.pending, podEvent{at: now + int64(delay)*1e6, efuse: n, on: on})
		sort.Slice(s.pod.pending, func(i, j int) bool { return s.pod.pending[i].at < s.pod.pending[j].at })
	} else {
		s.applyPower(now, n, on)
	}
	return map[string]any{"efuse": n, "enabled": b2i(on), "delay_ms": delay}, nil
}

func (s *Sim) cmdTargetStatus(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.advance(s.now())
	rail := func(e efuseState) map[string]any {
		return map[string]any{"enabled": b2i(e.enabled), "fault": b2i(e.fault), "valid": b2i(e.enabled)}
	}
	return map[string]any{"efuse1": rail(s.pod.efuse[1]), "efuse2": rail(s.pod.efuse[2]), "status_supported": true}, nil
}

func (s *Sim) cmdPowerStatus(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	ua, mv := s.railSample(now)
	internal := map[string]any{"ok": true, "bus_mv": mv, "current_ua": ua}
	// Nothing is wired to the external rail: an enabled eFuse 2 passes no supply through.
	external := map[string]any{"ok": true, "bus_mv": 0, "current_ua": 0}
	podUA := 285000 + int64(s.noise(4000))
	return map[string]any{
		"internal": internal, "external": external,
		"pod": map[string]any{"ok": true, "bus_mv": 5080 + int64(s.noise(6)), "current_ua": podUA, "total_ua": podUA + ua},
	}, nil
}

func (s *Sim) cmdNrst(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	switch {
	case r.has("pulse_ms"):
		ms := r.int("pulse_ms", 0)
		if ms < 1 || ms > 1000 {
			return nil, fwError("pulse_ms must be 1..1000")
		}
		// The pod replies once the line is released: hold, wait, release.
		s.setReset(now, true)
		s.mu.Unlock()
		ok := s.sleepUntil(now + int64(ms)*1e6)
		s.mu.Lock()
		t := s.now()
		s.advance(t)
		s.setReset(t, false)
		if !ok {
			return nil, fwError("pod shutting down")
		}
	case r.has("assert"):
		s.setReset(now, r.flag("assert"))
	}
	return map[string]any{"asserted": s.pod.nrst, "supported": true}, nil
}

func (s *Sim) setReset(t int64, asserted bool) {
	if s.pod.nrst == asserted {
		return
	}
	s.pod.nrst = asserted
	if asserted {
		s.dutResetAssert(t)
	} else {
		s.dutResetRelease(t)
	}
}

func (s *Sim) cmdUsbCc(request) (any, error) {
	return map[string]any{"supported": true, "cc1_mv": 1650, "cc2_mv": 0, "orientation": "cc1",
		"advertised": "3.0A", "advertised_ma": 3000}, nil
}

func b2i(b bool) int {
	if b {
		return 1
	}
	return 0
}
