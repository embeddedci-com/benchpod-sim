package benchsim

import "math"

// Power profiles (command_handler_power.c): the INA238 on a target rail sampled a few hundred
// times a second. The blocking form ("duration_ms") replies after the window; start/stop
// bracket other commands. Statistics cover every sample; the trace is averaged down to
// keep_samples points.

type powerSession struct {
	efuse  int
	rateHz float64
	keep   int
	start  int64
	maxEnd int64
}

// achievedRate is what the pod's sampling loop delivers: accurate to ~200 Hz, flattening near
// 365 Hz above that.
func achievedRate(req float64) float64 {
	if req <= 200 {
		return req
	}
	return 200 + (math.Min(req, 500)-200)*0.55
}

func (s *Sim) cmdPowerProfile(c *conn, r request) error {
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	action := r.str("action")
	efuse := r.int("efuse", 1)
	rate := float64(r.int("rate_hz", 500))
	keep := r.int("keep_samples", 0)
	if action == "" || action == "start" {
		if efuse != 1 && efuse != 2 {
			return fail(fwError("efuse must be 1 or 2"))
		}
		if rate < 100 || rate > 1000 {
			return fail(fwError("rate_hz must be 100..1000"))
		}
		if keep < 0 || keep > 4096 {
			return fail(fwError("keep_samples must be 0..4096"))
		}
	}
	switch action {
	case "":
		ms := r.int("duration_ms", 0)
		if ms <= 0 || ms > 600000 {
			return fail(fwError("duration_ms must be 1..600000"))
		}
		s.mu.Lock()
		t0 := s.now()
		s.mu.Unlock()
		t1 := t0 + int64(ms)*msNs
		if !s.sleepUntil(t1) {
			return fail(fwError("pod shutting down"))
		}
		s.mu.Lock()
		s.advance(s.now())
		frames := s.profileFrames(powerSession{efuse: efuse, rateHz: rate, keep: keep, start: t0}, t1, false)
		s.mu.Unlock()
		return sendFrames(c, frames)
	case "start":
		maxMs := r.int("max_duration_ms", 60000)
		s.mu.Lock()
		now := s.now()
		s.advance(now)
		s.pod.profile = &powerSession{efuse: efuse, rateHz: rate, keep: keep, start: now,
			maxEnd: now + int64(max(1, min(maxMs, 600000)))*msNs}
		s.mu.Unlock()
		return c.send(okReply(map[string]any{"started": true, "efuse": efuse, "rate_hz": int(rate)}))
	case "status":
		s.mu.Lock()
		p := s.pod.profile
		now := s.now()
		s.mu.Unlock()
		if p == nil {
			return c.send(okReply(map[string]any{"running": false, "efuse": nil, "elapsed_ms": 0, "n": 0}))
		}
		end := min(now, p.maxEnd)
		el := (end - p.start) / msNs
		return c.send(okReply(map[string]any{"running": now < p.maxEnd, "efuse": p.efuse, "elapsed_ms": el,
			"n": int(float64(el) / 1000 * achievedRate(p.rateHz))}))
	case "stop":
		s.mu.Lock()
		p := s.pod.profile
		if p == nil {
			s.mu.Unlock()
			return fail(fwError("no power profile running"))
		}
		now := s.now()
		s.advance(now)
		end := now
		truncated := false
		if end > p.maxEnd {
			end, truncated = p.maxEnd, true
		}
		frames := s.profileFrames(*p, end, truncated)
		s.pod.profile = nil
		s.mu.Unlock()
		return sendFrames(c, frames)
	}
	return fail(errf("unknown power_profile action %q", action))
}

func sendFrames(c *conn, frames []map[string]any) error {
	for _, f := range frames {
		if err := c.send(f); err != nil {
			return nil
		}
	}
	return nil
}

// profileFrames samples the rail over [p.start, end] and builds the reply frames.
func (s *Sim) profileFrames(p powerSession, end int64, truncated bool) []map[string]any {
	rate := achievedRate(p.rateHz)
	dt := int64(1e9 / rate)
	var n int
	var sumUA, sumMV, energy, charge float64
	minUA, peakUA := math.Inf(1), math.Inf(-1)
	minMV, maxMV := math.Inf(1), math.Inf(-1)
	type bin struct {
		t, ua, mv float64
		n         int
	}
	nBins := 0
	total := int((end - p.start) / dt)
	if p.keep > 0 {
		nBins = min(p.keep, max(1, total))
	}
	bins := make([]bin, nBins)
	pw := s.dut.pwr
	ci, ma := 0, 0.0
	for t := p.start; t < end; t += dt {
		for ci < len(pw) && pw[ci].t <= t {
			ma = pw[ci].ma
			ci++
		}
		var ua, mv int64
		if p.efuse == 1 {
			ua, mv = s.railFromMA(ma)
		}
		u, v := float64(ua), float64(mv)
		sumUA += u
		sumMV += v
		minUA, peakUA = math.Min(minUA, u), math.Max(peakUA, u)
		minMV, maxMV = math.Min(minMV, v), math.Max(maxMV, v)
		energy += u / 1e6 * v / 1e3 * float64(dt) / 1e9 // J
		charge += u / 1e6 * float64(dt) / 1e9           // C
		if nBins > 0 {
			b := &bins[min(nBins-1, n*nBins/max(1, total))]
			b.t += float64(t-p.start) / 1e3
			b.ua += u
			b.mv += v
			b.n++
		}
		n++
	}
	if n == 0 {
		minUA, peakUA, minMV, maxMV = 0, 0, 0, 0
	}
	avg := func(sum float64) float64 {
		if n == 0 {
			return 0
		}
		return sum / float64(n)
	}
	stats := map[string]any{
		"efuse": p.efuse, "rate_hz": math.Round(rate*10) / 10, "adc_rate_hz": p.rateHz, "n": n,
		"duration_ms": (end - p.start) / msNs,
		"avg_ua":      math.Round(avg(sumUA)), "min_ua": minUA, "peak_ua": peakUA,
		"avg_mv": math.Round(avg(sumMV)), "min_mv": minMV, "max_mv": maxMV,
		"energy_uj": math.Round(energy * 1e6), "charge_uc": math.Round(charge * 1e6),
		"fault": false, "truncated": truncated,
	}
	var ts, ua, mv []int64
	for _, b := range bins {
		if b.n == 0 {
			continue
		}
		ts = append(ts, int64(b.t/float64(b.n)))
		ua = append(ua, int64(b.ua/float64(b.n)))
		mv = append(mv, int64(b.mv/float64(b.n)))
	}
	var frames []map[string]any
	const per = 128
	for i := 0; i < len(ts) || i == 0; i += per {
		j := min(i+per, len(ts))
		status := "ok"
		if i > 0 {
			status = "chunk"
		}
		f := map[string]any{"status": status, "t_us": nonNil(ts[i:j]), "current_ua": nonNil(ua[i:j]), "bus_mv": nonNil(mv[i:j])}
		last := j >= len(ts)
		if last {
			f["stats"] = stats
		}
		f["more"] = !last
		frames = append(frames, f)
		if last {
			break
		}
	}
	return frames
}

func nonNil(v []int64) []int64 {
	if v == nil {
		return []int64{}
	}
	return v
}
