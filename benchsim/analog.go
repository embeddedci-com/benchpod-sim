package benchsim

import (
	"math"
	"sort"
)

// The pod's analog front end: one 16-bit DAC that drives the 3.3 V, 5 V or +-12 V SMA, and one
// ADC that reads the front SMA (ext), the 5 V / 12 V DAC loopbacks (cal1/cal2) or the 4-20 mA
// input. On this bench the 3.3 V SMA feeds the node's analog input PA1 through 5-10 kOhm and
// the node's analog output PA4 goes to the ADC SMA; nothing is on the other outputs or J8.

// What the DAC does.
const (
	dacHold = iota
	dacWave
	dacReplay
	dacLoopKind
)

type dacState struct {
	path    string // off, 3v3, 5v, 12v, current_out
	kind    int
	code    float64 // the held 16-bit code (also what an armed start holds)
	wave    string
	freq    float64
	amp     float64 // codes
	mid     float64
	t0      int64
	until   int64 // 0 = runs until dac_stop
	samples []uint16
	rate    float64
	loop    *dacLoop
	armed   bool  // the start waits for the next capture's t0 (on_capture)
	stopAt  int64 // the capture's auto-stop: the code freezes here
}

type analogState struct {
	dac       dacState
	adcSource string // ext, cal1, cal2, current_in, off
	hist      []analogSeg
}

// analogSeg is the routing and DAC that held until a moment, so a capture rendered after its
// window sees what the wires carried during it.
type analogSeg struct {
	until int64
	a     analogState
}

type dacLine struct{ a, b float64 }

func (l dacLine) volts(code float64) float64 { return l.a + l.b*code/256 }

// dacLines are the rev3 lines (cal_data.c CAL_V3): volts = a + b*code/256.
var dacLines = [3]dacLine{{0.0016304, 0.01290215}, {0.0025176, 0.01953943}, {-12.087242, 0.09440379}}

func (s *Sim) dacLineFor(path int) dacLine {
	if l := s.flash.dacCal[path]; l != nil {
		return *l
	}
	return dacLines[path]
}

// adcFit is the rev3 fit of a source (cal_data.c): volts = a + b*count, unwrapped.
func adcFit(source string) dacLine {
	switch source {
	case "cal1", "current_in":
		return dacLine{65.712125, -0.0010027157}
	case "cal2":
		return dacLine{65.812962, -0.0010042767}
	}
	return dacLine{adcCalA, adcCalB}
}

var dacRange = map[string][2]float64{"3v3": {0, 3.3}, "5v": {0, 5}, "12v": {-12, 12}}

// codeAt is the DAC's 16-bit code at t.
func (d dacState) codeAt(t int64) float64 {
	if d.stopAt > 0 && t > d.stopAt {
		t = d.stopAt
	}
	if d.armed || t < d.t0 {
		return d.code
	}
	switch d.kind {
	case dacWave:
		if d.until != 0 && t >= d.until {
			return d.mid
		}
		ph := math.Mod(d.freq*float64(t-d.t0)/1e9, 1)
		v := d.mid + d.amp*math.Sin(2*math.Pi*ph)
		switch d.wave {
		case "square":
			v = d.mid - d.amp
			if ph < 0.5 {
				v = d.mid + d.amp
			}
		case "sawtooth":
			v = d.mid - d.amp + 2*d.amp*ph
		}
		return math.Max(0, math.Min(65535, v))
	case dacReplay:
		i := int(float64(t-d.t0)/1e9*d.rate) % len(d.samples)
		return float64(d.samples[i])
	case dacLoopKind:
		l := d.loop
		if i := sort.Search(len(l.hist), func(i int) bool { return l.hist[i].t >= t }); i < len(l.hist) {
			return l.hist[i].v
		}
		return l.v
	}
	return d.code
}

// value is the routed output in volts at t.
func (s *Sim) dacVolts(d dacState, t int64) float64 {
	i := pathIndex(d.path)
	if i < 0 {
		return 0
	}
	return s.dacLineFor(i).volts(d.codeAt(t))
}

// snapshotAnalog keeps the state so far for captures looking back. Callers hold s.mu.
func (s *Sim) snapshotAnalog(t int64) {
	a := &s.pod.analog
	cur := *a
	cur.hist = nil
	a.hist = append(a.hist, analogSeg{until: t, a: cur})
	if len(a.hist) > 32 {
		a.hist = append(a.hist[:0], a.hist[len(a.hist)-32:]...)
	}
}

func (s *Sim) analogAt(t int64) analogState {
	for _, h := range s.pod.analog.hist {
		if t < h.until {
			return h.a
		}
	}
	return s.pod.analog
}

// pa1Volts is what the node's PA1 sees: the 3.3 V SMA when the DAC is routed there.
func (s *Sim) pa1Volts(t int64) float64 {
	d := s.pod.analog.dac
	if d.path != "3v3" {
		return 0.002
	}
	return math.Max(0, s.dacVolts(d, t))
}

// adcVolts is what the ADC reads at t on its routed source, before noise.
func (s *Sim) adcVolts(t int64) float64 {
	a := s.analogAt(t)
	switch a.adcSource {
	case "ext":
		return s.pa4Volts(t)
	case "cal1":
		if a.dac.path == "5v" {
			return s.dacVolts(a.dac, t)
		}
	case "cal2":
		if a.dac.path == "12v" {
			return s.dacVolts(a.dac, t)
		}
	}
	return 0
}

// voltsToCount inverts a fit: volts = a + b*count, unwrapped.
func voltsToCount(v float64, fit dacLine) int {
	c := int(math.Round((v - fit.a) / fit.b))
	c = max(32768, min(98303, c))
	return c & 0xFFFF
}

func (s *Sim) adcCount(t int64) int {
	return voltsToCount(s.adcVolts(t)+s.noise(0.0012), adcFit(s.analogAt(t).adcSource))
}

func (s *Sim) cmdAnalogPath(r request) (any, error) {
	if !r.has("path") {
		return nil, fwError("missing path")
	}
	p := r.str("path")
	if s.firmware && r.has("range") && r.str("range") != "normal" {
		if r.str("range") == "low" {
			return nil, fwError(adcRangeUnsupported)
		}
		return nil, fwError("range must be normal or low")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	a := &s.pod.analog
	canon := p
	switch canonRoute(p) {
	case "off":
		a.dac.path, a.adcSource = "off", "ext"
		if !s.firmware {
			a.adcSource = "off"
		}
	case "3v3", "5v", "12v":
		a.dac.path = canonRoute(p)
		canon = "dac_" + a.dac.path
		if s.firmware {
			a.adcSource = "ext"
		}
	case "ext":
		a.adcSource, canon = "ext", "adc_ext"
	case "cal1":
		a.dac.path, a.adcSource = "5v", "cal1"
	case "cal2":
		a.dac.path, a.adcSource = "12v", "cal2"
	case "current_in":
		a.adcSource = "current_in"
	case "current_out":
		a.dac.path = "current_out"
	default:
		return nil, fwError("unknown path")
	}
	if s.firmware {
		p = canon
		return map[string]any{"path": p, "u55": 0, "u58": 0, "range": "normal"}, nil
	}
	return map[string]any{"path": p, "u55": 0, "u58": 0}, nil
}

func (s *Sim) cmdDacOut(r request) (any, error) {
	if !r.has("path") {
		return nil, fwError("missing path")
	}
	p := canonRoute(r.str("path"))
	idx := pathIndex(p)
	if idx < 0 && p != "off" {
		if s.firmware && !map[string]bool{"ext": true, "cal1": true, "cal2": true, "current_in": true, "current_out": true}[p] {
			return nil, fwError("unknown path")
		}
		return nil, fwError("path must be 3v3|5v|12v|off")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	a := &s.pod.analog
	d := &a.dac
	canon := p
	if idx >= 0 {
		canon = "dac_" + p
	}
	name := r.str("path")
	if s.firmware {
		name = canon
		a.adcSource = "ext"
	}
	if p == "off" {
		d.path = "off"
		return map[string]any{"path": "off", "mv": 0, "code": -1}, nil
	}
	d.path = p
	if v, has := r.num("volts"); has || r.has("volts") {
		line := s.dacLineFor(idx)
		if !s.firmware {
			rng := dacRange[p]
			if !has || v < rng[0]-1e-9 || v > rng[1]+1e-9 {
				return nil, errf("volts out of range for %s (%g..%g V)", p, rng[0], rng[1])
			}
		}
		code := max(0, min(int(math.Round((v-line.a)/line.b*256)), 65535))
		got := line.volts(float64(code))
		*d = dacState{path: p, code: float64(code)}
		return map[string]any{"path": name, "mv": int(math.Round(got * 1000)), "uv": int64(math.Round(got * 1e6)),
			"code": min((code+128)>>8, 255), "code16": code}, nil
	}
	return map[string]any{"path": name, "mv": 0, "code": -1}, nil
}

func (s *Sim) cmdAdcRead(r request) (any, error) {
	src := "ext"
	if r.has("source") {
		src = r.str("source")
	}
	switch src {
	case "ext", "cal1", "cal2", "current_in":
	default:
		return nil, fwError("source must be ext|cal1|cal2|current_in")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	a := &s.pod.analog
	switch src {
	case "cal1":
		a.dac.path = "5v"
	case "cal2":
		a.dac.path = "12v"
	}
	a.adcSource = src
	// A 16-sample burst at 400 kS/s: refused when the input moves more than the firmware allows.
	lo, hi, sum := math.MaxInt, math.MinInt, 0
	for i := 0; i < 16; i++ {
		c := s.adcCount(now + int64(i)*2500)
		cu := c
		if cu < 32768 {
			cu += 65536
		}
		lo, hi, sum = min(lo, cu), max(hi, cu), sum+cu
	}
	if span := hi - lo; span > 64 {
		return nil, errf("adc_read: input not settled on %s (%d counts pk-pk over the 16-sample burst, limit 64). Stop the DAC (dac_stop) or let the node settle", src, span)
	}
	mean := float64(sum) / 16
	fit := adcFit(src)
	volts := fit.a + fit.b*mean
	out := map[string]any{"source": src, "mv": int(math.Round(volts * 1000)), "count": int(math.Round(mean)) & 0xFFFF, "span": hi - lo}
	if src == "current_in" {
		out["mv"] = 0
		out["offset_mv"] = 0
		out["ua"] = 0
	}
	return out, nil
}

func (s *Sim) cmdGenerate(r request) (any, error) {
	wave := r.str("waveform")
	switch wave {
	case "sine", "square", "sawtooth":
	default:
		return nil, fwError("waveform must be sine|square|sawtooth")
	}
	freq, ok := r.num("freq")
	if !ok || freq <= 0 || freq > 100000 {
		return nil, fwError("freq must be 0 < freq <= 100000 Hz")
	}
	if r.has("amplitude_ua") {
		return nil, notInDemo("the 4-20 mA output")
	}
	path := r.str("path")
	rng, ok := dacRange[path]
	if !ok {
		if r.has("amplitude") {
			return nil, fwError("this firmware takes amplitude_v/offset_v with a path")
		}
		return nil, fwError("path must be 3v3|5v|12v")
	}
	amp, ok1 := r.num("amplitude_v")
	off, ok2 := r.num("offset_v")
	if !ok1 || !ok2 || amp <= 0 {
		return nil, fwError("missing amplitude_v/offset_v")
	}
	slack := (rng[1] - rng[0]) * 0.05
	if off-amp < rng[0]-slack || off+amp > rng[1]+slack {
		return nil, errf("generate: the peaks %.4g to %.4g V are outside %g..%g V on %s", off-amp, off+amp, rng[0], rng[1], path)
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	line := s.dacLineFor(pathIndex(path))
	d := dacState{path: path, kind: dacWave, wave: wave, freq: freq, amp: amp / line.b * 256, mid: (off - line.a) / line.b * 256, t0: now}
	if ms := r.int("duration_ms", 0); ms > 0 {
		d.until = now + int64(ms)*msNs
	}
	if r.has("route") && !r.flag("route") {
		d.path = s.pod.analog.dac.path
	}
	s.snapshotAnalog(now)
	s.pod.analog.dac = d
	return map[string]any{"running": true, "bits": 16, "path": path, "freq": freq, "cotrig": false,
		"sample_rate_hz": 888888}, nil
}
