package benchsim

import (
	"encoding/base64"
	"fmt"
	"io"
	"math"
	"sort"
	"strconv"
	"strings"
	"time"
)

// The DAC as the firmware drives it (command_handler_dac.c, command_handler_analog.c, dac_limits.c
// and dac_loop_params.c): one 16-bit code that a held level, a waveform, a replayed trace or the
// in-fabric loop sets, and the routing decides which output shows it.

// loopMaxHist is how long the loop's output stays readable for captures.
const loopMaxHist = int64(2e9)

type dacLoop struct {
	k, vmin, vmax, tick    int
	mapOn, tripEn          bool
	inZero, inGain, idxMax int
	inTrip                 int
	v                      float64
	in                     int
	last                   int64
	tripped                bool
	hist                   []loopPt
}

type loopPt struct {
	t int64
	v float64
}

type loadBin struct {
	total int
	psram bool
}

type dacLimits struct {
	path         int
	inverted     bool
	minMv, maxMv int
}

func (l dacLimits) parkMv() int {
	if l.inverted {
		return l.maxMv
	}
	return l.minMv
}

func (l dacLimits) describe() string {
	inv := ""
	if l.inverted {
		inv = "inverted "
	}
	return fmt.Sprintf("%sDAC limits are set on the %s output (%d to %d mV)", inv, []string{"3v3", "5v", "12v"}[l.path], l.minMv, l.maxMv)
}

func pathIndex(name string) int {
	switch strings.TrimPrefix(name, "dac_") {
	case "3v3":
		return 0
	case "5v":
		return 1
	case "12v":
		return 2
	}
	return -1
}

func canonRoute(name string) string {
	switch name {
	case "dac_3v3", "dac_5v", "dac_12v":
		return strings.TrimPrefix(name, "dac_")
	case "adc_ext", "sma":
		return "ext"
	}
	return name
}

// dacLimitsCheck is dac_limits_check_command: the refusal text, or "". Callers hold s.mu.
func (s *Sim) dacLimitsCheck(cmd string, r request) string {
	l := s.flash.limits
	if l == nil {
		return ""
	}
	raw := func(what string) string {
		return fmt.Sprintf("refused: %s, and %s writes raw DAC codes that can't be checked against them. Use dac_out or the control loop, or clear them with dac_limits enabled:false", l.describe(), what)
	}
	route := func(path string) string {
		if !l.inverted {
			return ""
		}
		p := canonRoute(path)
		if p == []string{"3v3", "5v", "12v"}[l.path] || p == "ext" || p == "current_in" || (p == "cal1" && l.path == 1) {
			return ""
		}
		return fmt.Sprintf("refused: %s. Routing to %s disconnects it and leaves the module input near 0 V, which is full output. dac_stop parks it at %d mV instead", l.describe(), p, l.parkMv())
	}
	switch cmd {
	case "generate", "dac_set", "load", "load_bin", "replay", "measure", "dac_mux", "cal_switch":
		return raw(cmd)
	case "analog_path":
		return route(r.str("path"))
	case "adc_read":
		src := r.str("source")
		if src == "" {
			src = "ext"
		}
		return route(src)
	case "dac_out":
		path := r.str("path")
		why := route(path)
		v, ok := r.num("volts")
		if why == "" && ok && pathIndex(path) == l.path {
			if v < float64(l.minMv)/1000-0.005 || v > float64(l.maxMv)/1000+0.005 {
				return fmt.Sprintf("refused: %s; %d mV is outside them", l.describe(), int(math.Round(v*1000)))
			}
		}
		return why
	case "current_out":
		if _, ok := r.num("ua"); ok {
			return fmt.Sprintf("refused: %s, and the 4-20 mA output shares that DAC: setting a current moves it. Clear them with dac_limits enabled:false", l.describe())
		}
	case "dac_control_loop":
		if l.inverted && r.has("in_trip") {
			return fmt.Sprintf("refused: %s. in_trip drives the DAC to vmin, the HIGHEST output on an inverted stage", l.describe())
		}
		line := s.dacLineFor(l.path)
		vmin, ok1 := r.num("vmin")
		vmax, ok2 := r.num("vmax")
		if !ok1 || !ok2 || line.volts(vmin) < float64(l.minMv)/1000-0.005 || line.volts(vmax) > float64(l.maxMv)/1000+0.005 {
			lo := math.Max(0, math.Round((float64(l.minMv)/1000-line.a)/line.b*256))
			hi := math.Min(65535, math.Floor((float64(l.maxMv)/1000-line.a)/line.b*256))
			return fmt.Sprintf("refused: %s. The loop clamp must stay inside them: vmin at least %d and vmax at most %d", l.describe(), int(lo), int(hi))
		}
	case "calibrate":
		if r.has("dac") || r.has("adc") {
			return fmt.Sprintf("refused: %s, and the DMM calibration drives the DAC outside them. Clear them with dac_limits enabled:false, calibrate, then set them again", l.describe())
		}
	}
	return ""
}

func (s *Sim) cmdDacLimits(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	reply := func() (any, error) {
		l := s.flash.limits
		if l == nil {
			return map[string]any{"enabled": false}, nil
		}
		return map[string]any{"enabled": true, "path": []string{"3v3", "5v", "12v"}[l.path], "inverted": l.inverted,
			"min_mv": l.minMv, "max_mv": l.maxMv, "park_mv": l.parkMv()}, nil
	}
	if r.has("enabled") && r.str("enabled") == "false" {
		s.flash.limits = nil
		return reply()
	}
	if !r.has("path") {
		return reply()
	}
	if !r.has("min_mv") || !r.has("max_mv") {
		return nil, fwError("dac_limits needs path, min_mv and max_mv (and inverted)")
	}
	idx := pathIndex(r.str("path"))
	if idx < 0 {
		return nil, fwError("path must be 3v3, 5v or 12v")
	}
	l := dacLimits{path: idx, inverted: r.str("inverted") == "true", minMv: r.int("min_mv", 0), maxMv: r.int("max_mv", 0)}
	if l.minMv >= l.maxMv {
		return nil, fwError("min_v must be below max_v")
	}
	line := s.dacLineFor(idx)
	lo, hi := line.a, line.a+line.b*255
	if float64(l.minMv)/1000 < lo-0.05 || float64(l.maxMv)/1000 > hi+0.05 {
		return nil, errf("limits must sit inside the %s path's range (%d to %d mV)", []string{"3v3", "5v", "12v"}[idx], int(math.Round(lo*1000)), int(math.Round(hi*1000)))
	}
	s.flash.limits = &l
	return reply()
}

func (s *Sim) cmdDacStop(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	d := &s.pod.analog.dac
	if !s.firmware {
		*d = dacState{path: d.path}
		return map[string]any{"stopped": true}, nil
	}
	// dac_stop stops the sequencer: the DAC8551 keeps its last code.
	*d = dacState{path: d.path, code: d.codeAt(now)}
	if l := s.flash.limits; l != nil {
		line := s.dacLineFor(l.path)
		d.path = []string{"3v3", "5v", "12v"}[l.path]
		d.code = math.Round((float64(l.parkMv())/1000 - line.a) / line.b * 256)
		return map[string]any{"parked_mv": l.parkMv()}, nil
	}
	return nil, nil
}

func (s *Sim) cmdDacSet(r request) (any, error) {
	if !r.has("value") {
		return nil, fwError("missing value")
	}
	v := r.int("value", -1)
	if v < 0 || v > 255 {
		return nil, fwError("value out of range")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	d := &s.pod.analog.dac
	*d = dacState{path: d.path, code: float64(v) * 256}
	return nil, nil
}

// cmdGenerateFw is handle_generate: the 8-bit levels form (no routing) and the calibrated 16-bit
// form (volts on a path, or microamps on the 4-20 mA output).
func (s *Sim) cmdGenerateFw(_ *conn, r request) (any, error) {
	wave := r.str("waveform")
	freq := 1000.0
	if v, ok := r.num("freq"); ok {
		freq = v
	}
	durMs := r.int("duration_ms", 0)
	sixteen := r.has("amplitude_v") || r.has("amplitude_ua")
	if wave != "sine" && wave != "square" && wave != "sawtooth" {
		return nil, fwError("unknown waveform")
	}
	if freq <= 0 {
		return nil, fwError("generate failed")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	var amp, mid float64
	path := ""
	if sixteen {
		var err error
		amp, mid, path, err = s.generate16Codes(r)
		if err != nil {
			return nil, err
		}
	} else {
		amp = float64(r.int("amplitude", 127)) * 256
		mid = float64(r.int("offset", 128)) * 256
	}
	s.snapshotAnalog(now)
	a := &s.pod.analog
	prev := a.dac.codeAt(now)
	nd := dacState{path: a.dac.path, kind: dacWave, wave: wave, freq: freq, amp: amp, mid: mid, t0: now, code: prev}
	if durMs > 0 {
		nd.until = now + int64(durMs)*msNs
	}
	if path != "" && !(r.has("route") && r.str("route") == "false") {
		nd.path = path
		a.adcSource = routeSource(path, a.adcSource)
	}
	cotrigAsked := r.flag("on_capture")
	nd.armed = cotrigAsked
	a.dac = nd
	switch {
	case sixteen && cotrigAsked:
		return map[string]any{"bits": 16, "cotrig": true}, nil
	case sixteen:
		return map[string]any{"bits": 16}, nil
	case cotrigAsked:
		return map[string]any{"cotrig": true}, nil
	}
	return nil, nil
}

// routeSource is what routing an output does to the ADC relay: the voltage paths clear it.
func routeSource(path, cur string) string {
	if path == "current_out" {
		return cur
	}
	return "ext"
}

func (s *Sim) generate16Codes(r request) (amp, mid float64, path string, err error) {
	ua := r.has("amplitude_ua")
	ampIn, ok1 := r.num(map[bool]string{true: "amplitude_ua", false: "amplitude_v"}[ua])
	offKey := map[bool]string{true: "offset_ua", false: "offset_v"}[ua]
	if !r.has(offKey) {
		return 0, 0, "", errf("%s is required with %s", offKey, map[bool]string{true: "amplitude_ua", false: "amplitude_v"}[ua])
	}
	offIn, ok2 := r.num(offKey)
	if !ok1 || !ok2 {
		return 0, 0, "", fwError("amplitude and offset must be numbers")
	}
	if ua {
		top, bottom := offIn+ampIn, offIn-ampIn
		if ampIn <= 0 || bottom < 4000 || top > float64(currentOutUa(0xFFFF)) {
			return 0, 0, "", errf("the peaks %d to %d uA are outside the 4-20 mA output's %d to %d uA", int64(bottom), int64(top), currentOutUa(0), currentOutUa(0xFFFF))
		}
		cTop, cBot := (top-curOutZero)/curOutSlope, (bottom-curOutZero)/curOutSlope
		return (cTop - cBot) / 2, (cTop + cBot) / 2, "current_out", nil
	}
	p := canonRoute(r.str("path"))
	idx := pathIndex(p)
	if idx < 0 {
		return 0, 0, "", fwError("path must be 3v3|5v|12v with amplitude_v")
	}
	if ampIn <= 0 {
		return 0, 0, "", fwError("amplitude_v must be above 0 V")
	}
	l := s.dacLineFor(idx)
	lo, hi := l.a, l.a+l.b*65535/256
	slack := (hi - lo) * 0.05
	top, bottom := offIn+ampIn, offIn-ampIn
	if bottom < lo-slack || top > hi+slack {
		return 0, 0, "", errf("the peaks %d to %d mV are outside the %s output's %d to %d mV",
			int(math.Round(bottom*1000)), int(math.Round(top*1000)), p, int(math.Round(lo*1000)), int(math.Round(hi*1000)))
	}
	cTop, cBot := (top-l.a)/l.b*256, (bottom-l.a)/l.b*256
	return (cTop - cBot) / 2, (cTop + cBot) / 2, p, nil
}

// cmdLoadBin is handle_load_bin and load_bin_receive: raw bytes, then JSON again on the connection.
func (s *Sim) cmdLoadBin(c *conn, r request) error {
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	if !r.has("total") {
		return fail(fwError("missing total"))
	}
	total := r.int("total", 0)
	psram := strings.HasPrefix(strings.ToLower(r.str("psram")), "t") || r.str("psram") == "1"
	limit := 4096
	if psram {
		limit = 8 << 20
	}
	if total <= 0 || total > limit {
		return fail(fwError("total out of range"))
	}
	s.mu.Lock()
	if err := s.heavyBegin(c); err != nil {
		s.mu.Unlock()
		return fail(err)
	}
	s.pod.load = &loadBin{total: total, psram: psram}
	s.pod.trace, s.pod.tracePsram = nil, false
	s.mu.Unlock()
	if err := c.send(okReply(map[string]any{"ready": total})); err != nil {
		return err
	}
	buf := make([]byte, total)
	have, ackAt := 0, 0
	for have < total {
		_ = c.SetReadDeadline(time.Now().Add(30 * time.Second))
		n, err := c.br.Read(buf[have:min(have+65536, total)])
		have += n
		if c.cloud && have-ackAt >= 8192 {
			ackAt = have
			if _, werr := c.Write(fmt.Appendf(nil, "{\"ack\":%d}\n", have)); werr != nil {
				err = werr
			}
		}
		if err != nil {
			s.mu.Lock()
			s.pod.load = nil
			s.heavyRelease(c)
			s.mu.Unlock()
			if err == io.EOF {
				return err
			}
			return fail(fwError("load_bin timeout"))
		}
	}
	_ = c.SetReadDeadline(time.Time{})
	samples := make([]uint16, total/2)
	for i := range samples {
		samples[i] = uint16(buf[2*i]) | uint16(buf[2*i+1])<<8
	}
	s.mu.Lock()
	s.pod.load = nil
	s.pod.trace, s.pod.tracePsram = samples, psram
	if psram {
		s.pod.staged = buf
	}
	s.mu.Unlock()
	return c.send(okReply(map[string]any{"total": total / 2}))
}

func (s *Sim) loadAbandon(c *conn) {
	if s.pod.load != nil && s.pod.heavy == c.heavyKey() {
		s.pod.load = nil
	}
}

// cmdReplay is handle_replay: the last upload, looping until dac_stop.
func (s *Sim) cmdReplay(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	n := len(s.pod.trace)
	if n == 0 {
		return nil, fwError("nothing to replay")
	}
	samples := r.int("samples", n)
	if samples <= 0 || samples > n {
		return nil, fwError("samples out of range")
	}
	if err := s.heavyBegin(c); err != nil {
		return nil, err
	}
	defer s.heavyRelease(c)
	if s.pod.tracePsram && s.pod.image != 1 {
		return nil, fwError("replay failed")
	}
	mhz, _ := r.num("sample_rate_mhz")
	rate := replayRate(mhz*1e6, s.pod.tracePsram)
	s.snapshotAnalog(now)
	d := &s.pod.analog.dac
	cotrig := r.flag("on_capture")
	*d = dacState{path: d.path, kind: dacReplay, samples: s.pod.trace[:samples], rate: rate, t0: now, armed: cotrig, code: d.codeAt(now)}
	return map[string]any{"samples": samples, "cotrig": cotrig, "sample_rate_hz": int(rate)}, nil
}

// replayRate is the rate a replay really plays at: a whole divider of the 64 MHz DAC clock,
// clamped to what RAM (~889 kS/s) or PSRAM (~873 kS/s) can feed.
func replayRate(req float64, psram bool) float64 {
	top := 888888.0
	if psram {
		top = 873000
	}
	if req <= 0 || req >= top {
		return top
	}
	return 64e6 / math.Round(64e6/req)
}

func (s *Sim) loopTripped() bool {
	l := s.pod.analog.dac.loop
	return l != nil && l.tripped
}

const noLoopImage = "control loop not in the running gateware image (switch with {\"cmd\":\"fpga_image\",\"image\":0})"

// loopValidate is dac_loop_params_validate on the fields both commands share.
func loopValidate(src string, step int) error {
	if src != "adc" && src != "fixed" && src != "sweep" {
		return fwError("unknown loop input source (use adc, fixed or sweep)")
	}
	if src == "sweep" && step == 0 {
		return fwError("sweep source needs a non-zero step (0 never advances)")
	}
	return nil
}

// cmdControlLoop is handle_dac_control_loop.
func (s *Sim) cmdControlLoop(c *conn, r request) (any, error) {
	k, vmin, vmax, tick := r.int("k", 8192), r.int("vmin", 0), r.int("vmax", 0xFFFF), r.int("tick_div", 64)
	src := "adc"
	if r.has("source") {
		src = r.str("source")
	}
	in, step := r.int("input", 0), r.int("step", 0)
	if err := loopValidate(src, step); err != nil {
		return nil, err
	}
	if vmin > vmax {
		return nil, fwError("vmin must be <= vmax (inverted output clamp)")
	}
	k = max(1, min(k, 32767))
	tick = max(tick, 8)
	var curve []uint16
	if b64 := r.str("curve"); b64 != "" {
		raw, err := base64.RawURLEncoding.DecodeString(strings.TrimRight(b64, "="))
		if err != nil || len(raw) > 4096 {
			return nil, fwError("curve base64 decode failed")
		}
		if len(raw) < 2 {
			return nil, fwError("curve needs at least one 16-bit point")
		}
		for i := 0; i+1 < len(raw); i += 2 {
			curve = append(curve, uint16(raw[i])|uint16(raw[i+1])<<8)
		}
	}
	lp := &dacLoop{k: k, vmin: vmin, vmax: vmax, tick: tick, v: float64(vmin)}
	if r.has("in_mv_per_unit") {
		if err := lp.deriveMap(r); err != nil {
			return nil, err
		}
		if src == "sweep" {
			return nil, fwError("sweep cannot be combined with an input map on this gateware: the sweep spans the raw count range, not the mapped window. Step the input with {\"cmd\":\"dac_loop_input\"} instead")
		}
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.heavyBegin(c); err != nil {
		return nil, err
	}
	defer s.heavyRelease(c)
	if s.pod.image != 0 {
		return nil, fwError(noLoopImage)
	}
	now := s.now()
	s.advance(now)
	lutLen := 0
	if curve != nil {
		lutLen = 4096
		s.upsampleCurve(curve, lp)
	}
	s.pod.loopSrc, s.pod.loopIn, s.pod.loopStep = src, in, step
	lp.last, lp.in = now, in
	s.snapshotAnalog(now)
	d := &s.pod.analog.dac
	*d = dacState{path: d.path, kind: dacLoopKind, loop: lp, code: float64(vmin)}
	out := map[string]any{"armed": true, "k": k, "vmin": vmin, "vmax": vmax, "tick_div": tick, "curve_pts": lutLen / 2,
		"source": src, "input": in, "step": step}
	if lp.mapOn {
		out["in_zero"], out["in_gain"], out["idx_max"] = lp.inZero, lp.inGain, lp.idxMax
		out["trip_idx"] = -1
		if lp.tripEn {
			out["trip_idx"] = lp.inTrip
		}
	}
	return out, nil
}

// deriveMap is dac_loop_inmap_derive with the front-SMA fit.
func (l *dacLoop) deriveMap(r request) error {
	perUnit, _ := r.num("in_mv_per_unit")
	atZero, _ := r.num("in_mv_at_zero")
	lo, _ := r.num("in_min")
	hi, _ := r.num("in_max")
	if perUnit == 0 {
		return fwError("input mv_per_unit must be non-zero")
	}
	if !(hi > lo) {
		return fwError("input range max must be above min")
	}
	count := func(u float64) float64 { return ((atZero+u*perUnit)/1000 - adcCalA) / adcCalB }
	cLo, cHi := count(lo), count(hi)
	span := cHi - cLo
	if math.Abs(span) < 1 || math.Abs(span) >= 32768 {
		return fwError("input range spans half the ADC or more: the count wraps ambiguously there, so narrow the range or lower the sense gain")
	}
	gain := math.Max(-32767, math.Min(32767, 2047*32768/span))
	round := func(v float64) int {
		if v < 0 {
			return int(v - 0.5)
		}
		return int(v + 0.5)
	}
	l.inZero = round(cLo) & 0xFFFF
	l.inGain = round(gain)
	l.mapOn = true
	l.idxMax = int(math.Max(1, math.Min(2047, span*float64(l.inGain)/32768+0.5)))
	l.inTrip = 2047
	if r.has("in_trip") {
		trip, _ := r.num("in_trip")
		l.tripEn = true
		t := (count(trip) - cLo) * float64(l.inGain) / 32768
		l.inTrip = int(math.Max(0, math.Min(2047, t+0.5)))
	}
	return nil
}

// upsampleCurve is dac_loop_upsample_curve[_mapped]. Callers hold s.mu.
func (s *Sim) upsampleCurve(c []uint16, l *dacLoop) {
	last := len(c) - 1
	for j := 0; j < 2048; j++ {
		if !l.mapOn {
			if len(c) >= 2048 {
				s.pod.lut[j] = c[j]
			} else {
				s.pod.lut[j] = c[j*len(c)/2048]
			}
			continue
		}
		if j >= l.idxMax || last == 0 {
			s.pod.lut[j] = c[last]
			continue
		}
		num := j * last
		i, frac := num/l.idxMax, num%l.idxMax
		v := int(c[i])
		if frac != 0 && i < last {
			v += ((int(c[i+1])-v)*frac + l.idxMax/2) / l.idxMax
		}
		s.pod.lut[j] = uint16(max(0, min(v, 65535)))
	}
}

// loopAdvance runs the fabric's loop up to t. Callers hold s.mu.
func (s *Sim) loopAdvance(t int64) {
	l := s.pod.analog.dac.loop
	if l == nil || t <= l.last {
		return
	}
	if t-l.last > loopMaxHist {
		l.last = t - loopMaxHist
	}
	tickNs := int64(float64(l.tick) / 48e6 * 1e9)
	stepNs := max(tickNs, 20000)
	ticks := float64(stepNs) / float64(max(tickNs, 1))
	f := 1 - math.Pow(1-float64(l.k)/32768, ticks)
	for l.last+stepNs <= t {
		l.last += stepNs
		switch s.pod.loopSrc {
		case "adc":
			l.in = s.adcCount(l.last)
		case "fixed":
			l.in = s.pod.loopIn
		case "sweep":
			l.in = (l.in + int(float64(s.pod.loopStep)*ticks)) & 0xFFFF
		}
		idx := l.in >> 5
		if l.mapOn {
			d := int(int16(uint16(l.in - l.inZero)))
			idx = max(0, min((d*l.inGain)>>15, 2047))
			if l.tripEn && idx >= l.inTrip {
				l.tripped = true
			}
		}
		if l.tripped {
			l.v = float64(l.vmin)
		} else {
			l.v += f * (float64(s.pod.lut[idx]) - l.v)
			l.v = math.Max(float64(l.vmin), math.Min(float64(l.vmax), l.v))
		}
		l.hist = append(l.hist, loopPt{l.last, l.v})
	}
	if n := len(l.hist); n > 0 && l.hist[0].t < t-loopMaxHist {
		i := sort.Search(n, func(i int) bool { return l.hist[i].t >= t-loopMaxHist })
		l.hist = append(l.hist[:0], l.hist[i:]...)
	}
}

func (s *Sim) cmdLoopInput(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	src, in, step := s.pod.loopSrc, s.pod.loopIn, s.pod.loopStep
	if r.has("source") {
		src = r.str("source")
	}
	in, step = r.int("input", in), r.int("step", step)
	if err := loopValidate(src, step); err != nil {
		return nil, err
	}
	if err := s.heavyBegin(c); err != nil {
		return nil, err
	}
	defer s.heavyRelease(c)
	if s.pod.image != 0 {
		return nil, fwError(noLoopImage)
	}
	now := s.now()
	s.advance(now)
	s.pod.loopSrc, s.pod.loopIn, s.pod.loopStep = src, in, step
	return map[string]any{"source": src, "input": in, "step": step, "v": int(s.pod.analog.dac.codeAt(now))}, nil
}

func (s *Sim) cmdLoopProbe(c *conn, _ request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.heavyBegin(c); err != nil {
		return nil, err
	}
	defer s.heavyRelease(c)
	if s.pod.image != 0 {
		return nil, fwError("closed-loop DAC not in the running gateware image (switch with {\"cmd\":\"fpga_image\",\"image\":0})")
	}
	now := s.now()
	s.advance(now)
	d := s.pod.analog.dac
	in, tripped := 0, false
	if d.loop != nil {
		in, tripped = d.loop.in, d.loop.tripped
	}
	return map[string]any{"i": s.adcCount(now), "in": in, "source": s.pod.loopSrc, "v": int(math.Round(d.codeAt(now))), "tripped": tripped}, nil
}

func (s *Sim) cmdCurrentOut(r request) (any, error) {
	minUa, maxUa := currentOutUa(0), currentOutUa(0xFFFF)
	if !r.has("ua") {
		return map[string]any{"min_ua": minUa, "max_ua": maxUa}, nil
	}
	req, ok := r.num("ua")
	if !ok {
		return nil, fwError("current_out: ua must be a number of microamps")
	}
	ua := int64(math.Round(req))
	if ua < 4000 || ua > maxUa {
		return nil, errf("current_out: %d uA is out of range. The output can do %d to %d uA: it cannot go below the 4 mA live zero or above the top of the DAC", ua, minUa, maxUa)
	}
	code := 0
	if float64(ua) > curOutFloor {
		code = max(0, min(int(math.Round((float64(ua)-curOutZero)/curOutSlope)), 0xFFFF))
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	s.pod.analog.dac = dacState{path: "current_out", code: float64(code)}
	return map[string]any{"ua": currentOutUa(code), "code": code, "min_ua": minUa, "max_ua": maxUa}, nil
}

const (
	curOutZero  = 4025.338
	curOutSlope = 0.24518720
	curOutFloor = 4055.0
)

func currentOutUa(code int) int64 {
	return int64(math.Round(math.Max(curOutZero+curOutSlope*float64(code), curOutFloor)))
}

func (s *Sim) cmdAdcRange(r request) (any, error) {
	if r.has("test") && r.str("test") != "false" {
		return nil, fwError(adcRangeUnsupported)
	}
	if r.has("range") {
		switch r.str("range") {
		case "normal":
		case "low":
			return nil, fwError(adcRangeUnsupported)
		default:
			return nil, fwError("range must be normal or low")
		}
	}
	return map[string]any{"range": "normal", "supported": false, "fullscale_mv": adcFullscaleMv(), "normal_fullscale_mv": adcFullscaleMv(),
		"adc_cal_a_uv": int64(math.Round(adcCalA * 1e6)), "adc_cal_b_nv": int64(math.Round(adcCalB * 1e9))}, nil
}

const adcRangeUnsupported = "adc_range: this board has no low range (analog module v3r4 and later)"

// cmdAdcReadFw is handle_adc_read: routed, a 20 ms settle, a burst, scaled with the source's fit.
func (s *Sim) cmdAdcReadFw(c *conn, r request) (any, error) {
	src := "ext"
	if r.has("source") {
		src = canonRoute(r.str("source"))
	}
	switch src {
	case "ext", "cal1", "cal2", "current_in":
	default:
		return nil, fwError("source must be ext|cal1|cal2|current_in")
	}
	if r.has("range") && r.str("range") != "normal" {
		if r.str("range") == "low" {
			return nil, fwError(adcRangeUnsupported)
		}
		return nil, fwError("range must be normal or low")
	}
	s.mu.Lock()
	if err := s.heavyBegin(c); err != nil {
		s.mu.Unlock()
		return nil, err
	}
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
	s.mu.Unlock()
	time.Sleep(20 * time.Millisecond)
	s.mu.Lock()
	defer s.mu.Unlock()
	defer s.heavyRelease(c)
	now = s.now()
	s.advance(now)
	lo, hi, sum := math.MaxInt, math.MinInt, 0
	for i := 0; i < 16; i++ {
		cu := s.adcCount(now + int64(i)*2500)
		if cu < 32768 {
			cu += 65536
		}
		lo, hi, sum = min(lo, cu), max(hi, cu), sum+cu
	}
	name := map[string]string{"ext": "adc_ext", "cal1": "cal1", "cal2": "cal2", "current_in": "current_in"}[src]
	if hi-lo > 64 {
		return nil, errf("adc_read: input not settled on %s (%d counts pk-pk over the 16-sample burst, limit 64). Stop the DAC (dac_stop) or let the node settle", name, hi-lo)
	}
	fit := adcFit(src)
	mean := float64(sum) / 16
	volts := fit.a + fit.b*mean
	out := map[string]any{"source": name, "mv": int(math.Round(volts * 1000)), "count": int(math.Round(mean)) & 0xFFFF, "span": hi - lo,
		"range": "normal", "fullscale_mv": adcFullscaleMv(), "overrange": false}
	if src == "current_in" {
		out["offset_mv"] = s.flash.calOffsetUV / 1000
		out["ua"] = int64(math.Round(volts / 249 * 1e6))
	}
	return out, nil
}

// cmdCalibrate is handle_calibrate: the stored calibration, clearing it, the J8 offset, the two
// DMM forms. A DAC line changes what the outputs do from then on.
func (s *Sim) cmdCalibrate(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if r.has("clear") && r.str("clear") != "false" {
		switch r.str("clear") {
		case "true":
			s.flash.calOffsetUV, s.flash.calSet, s.flash.dacCal, s.flash.adcGain = 0, false, [3]*dacLine{}, [3]*float64{}
		case "offset":
			s.flash.calOffsetUV, s.flash.calSet = 0, false
		case "dac":
			s.flash.dacCal = [3]*dacLine{}
		case "adc":
			s.flash.adcGain = [3]*float64{}
		default:
			return nil, fwError("calibrate: clear must be true, offset, dac or adc")
		}
		s.capsChanged()
		return s.calReply(nil), nil
	}
	if r.has("dac") {
		path := pathIndex(r.str("dac"))
		if path < 0 {
			return nil, fwError("calibrate: dac must be 3v3, 5v or 12v")
		}
		if !r.has("code1") || !r.has("uv1") || !r.has("code2") || !r.has("uv2") {
			return nil, fwError("calibrate: dac needs code1, uv1, code2 and uv2 (16-bit codes, DMM microvolts)")
		}
		c1, u1, c2, u2 := r.int("code1", 0), r.int("uv1", 0), r.int("code2", 0), r.int("uv2", 0)
		if c1 < 0 || c1 > 65535 || c2 < 0 || c2 > 65535 || u1 > 20000000 || u1 < -20000000 || u2 > 20000000 || u2 < -20000000 {
			return nil, fwError("calibrate: codes are 0-65535 and readings within +/-20 V")
		}
		if c1 == c2 {
			return nil, fwError("calibrate: the two set points must differ")
		}
		b := (float64(u2-u1) / 1e6) / float64(c2-c1) * 256
		s.flash.dacCal[path] = &dacLine{a: float64(u1)/1e6 - b*float64(c1)/256, b: b}
		return s.calReply(nil), nil
	}
	if r.has("adc") {
		if !map[string]bool{"cal1": true, "cal2": true, "ext": true}[r.str("adc")] {
			return nil, fwError("calibrate: adc must be cal1, cal2 or ext")
		}
		return s.calReply(nil), nil
	}
	if !r.has("source") {
		return s.calReply(nil), nil
	}
	if r.str("source") != "current_in" {
		return nil, fwError("only current_in can be calibrated on the pod: source must be current_in")
	}
	if err := s.heavyBegin(c); err != nil {
		return nil, err
	}
	defer s.heavyRelease(c)
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	s.pod.analog.adcSource = "current_in"
	sum := 0
	for i := 0; i < 1024; i++ {
		cu := s.adcCount(now + int64(i)*2500)
		if cu < 32768 {
			cu += 65536
		}
		sum += cu
	}
	mean := float64(sum) / 1024
	fit := adcFit("current_in")
	s.flash.calOffsetUV = int64(math.Round((fit.a + fit.b*mean) * 1e6))
	s.flash.calSet = true
	s.capsChanged()
	return s.calReply(map[string]any{"count": int(mean) & 0xFFFF, "span": 4, "samples": 1024}), nil
}

func (s *Sim) calReply(extra map[string]any) map[string]any {
	dac := map[string]any{}
	dmm := false
	for i, name := range []string{"3v3", "5v", "12v"} {
		l := s.flash.dacCal[i]
		if l == nil {
			dac[name] = nil
			continue
		}
		dmm = true
		comp := dacLines[i]
		code8 := 0.0
		if i == 2 {
			code8 = -comp.a / comp.b
		}
		dac[name] = map[string]any{"a_uv": int64(math.Round(l.a * 1e6)), "b_pv": int64(math.Round(l.b / 256 * 1e12)),
			"offset_uv": int64(math.Round(((l.a + l.b*code8) - (comp.a + comp.b*code8)) * 1e6)),
			"gain_ppm":  int64(math.Round((l.b/comp.b - 1) * 1e6))}
	}
	fit := adcFit("cal1")
	out := map[string]any{"source": "current_in", "calibrated": s.flash.calSet, "offset_mv": s.flash.calOffsetUV / 1000, "offset_uv": s.flash.calOffsetUV,
		"a_uv": int64(math.Round(fit.a * 1e6)), "b_nv": int64(math.Round(fit.b * 1e9)), "dac": dac,
		"adc_gain_ppm": map[string]any{"cal1": nil, "cal2": nil, "ext": nil}, "dmm": dmm}
	for k, v := range extra {
		out[k] = v
	}
	return out
}

func (s *Sim) capsChanged() {
	if s.hooks.CapsChanged != nil {
		go s.hooks.CapsChanged()
	}
}

func (s *Sim) cmdFpgaImage(c *conn, r request) (any, error) {
	if !r.has("image") {
		return nil, fwError("image required (0=loop, 1=deep-replay)")
	}
	img, err := strconv.Atoi(r.str("image"))
	if err != nil || img < 0 || img > 1 {
		img = -1
	}
	s.mu.Lock()
	if err := s.heavyBegin(c); err != nil {
		s.mu.Unlock()
		return nil, err
	}
	s.mu.Unlock()
	if img < 0 {
		s.mu.Lock()
		s.heavyRelease(c)
		s.mu.Unlock()
		return nil, fwError("image out of range (0=loop, 1=deep-replay)")
	}
	// A swap reflashes the config flash; the reset below is what reconfiguring does.
	time.Sleep(500 * time.Millisecond)
	s.mu.Lock()
	defer s.mu.Unlock()
	defer s.heavyRelease(c)
	now := s.now()
	s.advance(now)
	s.snapshotAnalog(now)
	if s.pod.image != img {
		d := &s.pod.analog.dac
		*d = dacState{path: d.path, code: d.codeAt(now)}
		s.pod.image = img
		s.pod.loopSrc, s.pod.loopIn, s.pod.loopStep = "adc", 0, 0
		s.pod.lastCap = nil
		s.pod.spi = nil
		s.releaseFn("spi_sck", "spi_mosi", "spi_miso", "spi_cs")
	}
	s.capsChanged()
	return map[string]any{"image": img, "version": podGateware, "features": img + 1}, nil
}

func (s *Sim) cmdPsramRecover(request) (any, error) {
	if s.hooks.Reboot != nil {
		time.AfterFunc(400*time.Millisecond, s.hooks.Reboot)
	}
	return map[string]any{"recover": "rebooting"}, nil
}

func (s *Sim) cmdPsramPing(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if err := s.heavyBegin(c); err != nil {
		return nil, err
	}
	defer s.heavyRelease(c)
	n := max(4, min(r.int("count", 16), 256))
	return map[string]any{"pass": true, "count": n, "first_bad": -1}, nil
}

// The policies and cloud link settings (command_handler_policy.c, pod_policy.c).

func (s *Sim) cmdSigPolicy(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if r.has("set") {
		order := map[string]int{"audit": 0, "permissive": 1, "required": 2}
		v, ok := order[r.str("set")]
		if !ok {
			return nil, fwError("sig_policy: set must be audit, permissive or required")
		}
		if !c.cloud {
			return nil, fwError("sig_policy: change it from the cloud or the USB console")
		}
		if v < order[s.flash.sigPolicy] {
			return nil, errf("sig_policy: only the USB console can loosen the policy (now %s)", s.flash.sigPolicy)
		}
		s.flash.sigPolicy = r.str("set")
	}
	return map[string]any{"policy": s.flash.sigPolicy, "enforces": true, "keys": 1}, nil
}

func (s *Sim) cmdLanPolicy(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if r.has("set") {
		p := r.str("set")
		if p != "open" && p != "locked" && p != "off" {
			return nil, fwError("lan_policy: set must be open, locked or off")
		}
		if !c.cloud {
			return nil, fwError("lan_policy: change it from the cloud or the USB console")
		}
		s.flash.lanPolicy = p
	}
	return map[string]any{"policy": s.flash.lanPolicy}, nil
}

func (s *Sim) cloudLinkGate(c *conn, verb string) error {
	if !c.cloud {
		return errf("%s: change it from the cloud or over USB", verb)
	}
	if s.pod.capture || s.pod.heavy != nil || s.pod.load != nil {
		return fwError("busy: a capture, upload or update is using the PSRAM bus; try again when it ends")
	}
	return nil
}

func (s *Sim) cmdCloudCa(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if r.str("clear") == "true" {
		if err := s.cloudLinkGate(c, "cloud_ca"); err != nil {
			return nil, err
		}
		s.flash.caCerts = nil
	}
	certs := []map[string]any{}
	certs = append(certs, s.flash.caCerts...)
	return map[string]any{"present": len(certs) > 0, "certs": certs}, nil
}

func (s *Sim) cmdCloudProxy(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	changed := false
	switch {
	case r.has("set"):
		if err := s.cloudLinkGate(c, "cloud_proxy"); err != nil {
			return nil, err
		}
		spec := r.str("set")
		host, portS, ok := strings.Cut(spec, ":")
		port, err := strconv.Atoi(portS)
		if !ok || host == "" || err != nil || port < 1 || port > 65535 {
			return nil, fwError("cloud_proxy: set must be host:port")
		}
		s.flash.proxyHost, s.flash.proxyPort, s.flash.proxyUser, s.flash.proxyPass = host, port, r.str("user"), r.str("password")
		changed = true
	case r.str("clear") == "true":
		if err := s.cloudLinkGate(c, "cloud_proxy"); err != nil {
			return nil, err
		}
		s.flash.proxyHost, s.flash.proxyPort, s.flash.proxyUser, s.flash.proxyPass = "", 0, "", ""
		changed = true
	}
	if changed && s.hooks.ProxyChanged != nil {
		go s.hooks.ProxyChanged()
	}
	if s.flash.proxyHost == "" {
		return map[string]any{}, nil
	}
	return map[string]any{"host": s.flash.proxyHost, "port": s.flash.proxyPort, "auth": s.flash.proxyUser != ""}, nil
}
