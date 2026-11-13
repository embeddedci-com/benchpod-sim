package benchsim

import (
	"encoding/base64"
	"encoding/binary"
	"math"
	"time"
)

// Captures look forward in time like the real pod: a capture of n samples at rate r starts now
// (or at its trigger) and replies once n/r seconds have passed, rendered from the line
// histories. Caps keep one visitor bounded; the SDK accepts a shorter capture than it asked for.
const (
	laClockHz      = 24e6
	maxLaSamples   = 1_000_000
	maxAdcSamples  = 262_144
	adcMaxRateHz   = 400e3
	maxCaptureNs   = int64(10e9)
	maxSensorLaB   = 4096
	edgesPerChunk  = 2048
	samplesPerChnk = 256
	maxTriggerNs   = int64(600e9)
)

// laRate is the achieved LA rate for a request in Hz: 24 MHz over a whole divider (0 = max).
func laRate(req float64) float64 {
	if req <= 0 || req >= laClockHz {
		return laClockHz
	}
	div := math.Round(laClockHz / req)
	return laClockHz / math.Max(1, div)
}

func adcRate(req float64) float64 {
	if req <= 0 || req >= adcMaxRateHz {
		return adcMaxRateHz
	}
	div := math.Round(24e6 / req)
	return 24e6 / math.Max(1, div)
}

// clampSamples keeps a capture within the sample and duration caps.
func (s *Sim) clampSamples(n, cap int, rate float64) int {
	if s.firmware {
		return n
	}
	n = min(n, cap)
	if limit := int(rate * float64(maxCaptureNs) / 1e9); n > limit {
		n = max(1, limit)
	}
	return n
}

// beginCapture claims the pod's capture engine (one capture at a time).
func (s *Sim) beginCapture(c *conn) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.firmware {
		if err := s.heavyBegin(c); err != nil {
			return err
		}
		s.heavyRelease(c)
	} else if s.pod.capture {
		return fwError("busy: a capture is running")
	}
	s.pod.capture = true
	return nil
}

func (s *Sim) endCapture() {
	s.mu.Lock()
	s.pod.capture = false
	s.mu.Unlock()
}

type trigSpec struct {
	la      int
	edge    string
	timeout int64
}

func parseTrigger(r request) (*trigSpec, error) {
	t, ok := r["trigger"].(map[string]any)
	if !ok {
		return nil, nil
	}
	tr := request(t)
	ts := &trigSpec{la: tr.int("la", 0), edge: tr.str("edge"), timeout: int64(r.int("trigger_timeout_ms", 10000)) * msNs}
	if ts.la < 1 || ts.la > 14 {
		return nil, fwError("trigger la must be 1..14")
	}
	switch ts.edge {
	case "rising", "falling", "high", "low":
	default:
		return nil, fwError("trigger edge must be rising|falling|high|low")
	}
	if ts.timeout <= 0 || ts.timeout > maxTriggerNs {
		return nil, fwError("trigger_timeout_ms out of range")
	}
	return ts, nil
}

// pretrigger is how far before an edge trigger a capture at rate starts: two samples, so the
// edge itself is in the capture (a decoder needs the START condition or the start bit) and
// t = 0 stays within two sample periods of the trigger moment.
func (ts *trigSpec) pretrigger(rate float64) int64 {
	if ts.edge == "rising" || ts.edge == "falling" {
		return int64(2e9 / rate)
	}
	return 0
}

// waitTrigger returns when the trigger condition first held after arming, polling the line
// history as simulated time passes.
func (s *Sim) waitTrigger(ts *trigSpec) (int64, error) {
	s.mu.Lock()
	armed := s.now()
	s.mu.Unlock()
	from := armed
	deadline := armed + ts.timeout
	for {
		s.mu.Lock()
		now := min(s.now(), deadline)
		s.advance(now)
		at, ok := s.sig.findTrigger(ts.la, ts.edge, from, now)
		s.mu.Unlock()
		if ok {
			return at, nil
		}
		if now >= deadline {
			kind := "edge"
			if ts.edge == "high" || ts.edge == "low" {
				kind = "level"
			}
			return 0, errf("trigger timeout: no %s %s on LA%d within %d ms", ts.edge, kind, ts.la, ts.timeout/msNs)
		}
		if ts.edge == "rising" || ts.edge == "falling" {
			from = now // edges after what was searched; a level can be checked from anywhere
		}
		select {
		case <-s.done:
			return 0, fwError("pod shutting down")
		case <-time.After(5 * time.Millisecond):
		}
	}
}

// laFrames sends an RLE LA capture as the firmware does: {"la":true,"la_edges":[[i,w],...]}.
func laFrames(c *conn, edges [][2]int, n int, rate float64, first map[string]any, final bool) error {
	for i := 0; i < len(edges) || i == 0; i += edgesPerChunk {
		j := min(i+edgesPerChunk, len(edges))
		last := j >= len(edges)
		upto := n
		if !last {
			upto = edges[j-1][0] + 1
		}
		m := map[string]any{"status": "chunk", "la": true, "la_edges": edges[i:j], "la_upto": upto, "more": !(last && final)}
		if i == 0 || !c.sim.firmware {
			m["status"], m["la_rate_hz"] = "ok", int(rate)
		}
		if i == 0 && c.sim.firmware {
			m["bits"] = 16
		}
		for k, v := range first {
			m[k] = v
		}
		first = nil
		if last && final && c.trigger != nil {
			m["trigger"] = c.trigger
		}
		if err := c.send(m); err != nil {
			return err
		}
		if last {
			break
		}
	}
	return nil
}

func (s *Sim) cmdLaCapture(c *conn, r request) error {
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	s.mu.Lock()
	err := s.requireLaVoltage()
	s.mu.Unlock()
	if err != nil {
		return fail(err)
	}
	n := r.int("samples", 256)
	if n <= 0 || n > 4_000_000 {
		return fail(fwError("samples out of range"))
	}
	mhz, _ := r.num("sample_rate_mhz")
	rate := laRate(mhz * 1e6)
	n = s.clampSamples(n, maxLaSamples, rate)
	ts, err := parseTrigger(r)
	if err != nil {
		return fail(err)
	}
	if err := s.beginCapture(c); err != nil {
		return fail(err)
	}
	defer s.endCapture()
	s.mu.Lock()
	t0 := s.now()
	s.mu.Unlock()
	if ts != nil {
		if t0, err = s.waitTrigger(ts); err != nil {
			return fail(err)
		}
		t0 -= ts.pretrigger(rate)
	}
	stopUs := s.captureStart(t0, r)
	t1 := t0 + int64(float64(n)/rate*1e9)
	if !s.sleepUntil(t1) {
		return fail(fwError("pod shutting down"))
	}
	s.mu.Lock()
	s.advance(s.now())
	edges := s.sig.renderRLE(t0, n, rate)
	s.captureEnd(stopUs)
	s.pod.lastCap = &lastCapture{edges: edges, nl: n, lr: rate}
	s.mu.Unlock()
	if ts != nil {
		c.trigger = map[string]any{"la": ts.la, "edge": ts.edge, "fired": true}
	}
	return laFrames(c, edges, n, rate, nil, true)
}

// cmdSensorLa is an emulated I2C sensor's bus capture: SCL/SDA packed four samples per byte. The
// simulator starts the window just before the node's next transfer on the bus (within 1.2 s),
// so a short capture shows a whole transaction instead of an idle bus; a real pod free-runs
// and only catches one by chance.
func (s *Sim) cmdSensorLa(c *conn, r request) error {
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	s.mu.Lock()
	e, err := s.sensorInstance(r)
	var scl, sda int
	if err == nil {
		switch {
		case e.kind() != "i2c":
			err = errf("sensor_la captures I2C sensors only; use la_capture for the pins of %s", e.name)
		case !e.running:
			err = errf("%s is not running: sensor_start it first", e.name)
		default:
			scl, sda = e.pins["scl"], e.pins["sda"]
		}
	}
	s.mu.Unlock()
	if err != nil {
		return fail(err)
	}
	bytes := r.int("samples", 256)
	if bytes <= 0 || bytes > maxSensorLaB {
		return fail(fwError("samples must be 1..4096"))
	}
	mhz, _ := r.num("sample_rate_mhz")
	rate := laRate(mhz * 1e6)
	n := bytes * 4
	if err := s.beginCapture(c); err != nil {
		return fail(err)
	}
	defer s.endCapture()
	s.mu.Lock()
	armed := s.now()
	s.mu.Unlock()
	t0 := armed
	for wait := armed + int64(1200*msNs); ; {
		s.mu.Lock()
		now := s.now()
		s.advance(now)
		found := int64(-1)
		for _, x := range s.sig.i2c {
			if x.start >= armed {
				found = x.start
				break
			}
		}
		s.mu.Unlock()
		if found >= 0 {
			t0 = found - int64(math.Min(50e3, 0.05*float64(n)/rate*1e9))
			break
		}
		if now >= wait {
			t0 = now
			break
		}
		select {
		case <-s.done:
			return fail(fwError("pod shutting down"))
		case <-time.After(10 * time.Millisecond):
		}
	}
	t1 := t0 + int64(float64(n)/rate*1e9)
	if !s.sleepUntil(t1) {
		return fail(fwError("pod shutting down"))
	}
	s.mu.Lock()
	s.advance(s.now())
	edges := s.sig.renderRLE(t0, n, rate)
	s.mu.Unlock()
	out := make([]int, bytes)
	ei := 0
	w := 0
	for i := 0; i < n; i++ {
		for ei < len(edges) && edges[ei][0] <= i {
			w = edges[ei][1]
			ei++
		}
		pair := (w>>(scl-1)&1)<<1 | (w >> (sda - 1) & 1)
		out[i/4] |= pair << ((3 - i%4) * 2)
	}
	return sendSamples(c, out, 1024)
}

// sendSamples writes a sample list the way bulk_begin does: chunks of "data" with "more".
func sendSamples(c *conn, data []int, per int) error {
	if len(data) == 0 {
		return c.send(map[string]any{"status": "ok", "data": []int{}, "more": false})
	}
	for i := 0; i < len(data); i += per {
		j := min(i+per, len(data))
		if err := c.send(map[string]any{"status": "ok", "data": data[i:j], "more": j < len(data)}); err != nil {
			return nil
		}
	}
	return nil
}

// adcChunk is one dense ADC frame: base64url of little-endian uint16 ("enc":"b64") or decimal.
func adcChunk(counts []int, b64 bool) map[string]any {
	if !b64 {
		return map[string]any{"status": "ok", "data": counts}
	}
	buf := make([]byte, 2*len(counts))
	for i, v := range counts {
		binary.LittleEndian.PutUint16(buf[2*i:], uint16(v))
	}
	return map[string]any{"status": "ok", "b64": base64.RawURLEncoding.EncodeToString(buf)}
}

func (s *Sim) adcCounts(t0 int64, n int, rate float64) []int {
	out := make([]int, n)
	for i := range out {
		out[i] = s.adcCount(t0 + int64(float64(i)/rate*1e9))
	}
	return out
}

func sendAdc(c *conn, counts []int, rate float64, b64, final bool) error {
	for i := 0; i < len(counts) || i == 0; i += samplesPerChnk {
		j := min(i+samplesPerChnk, len(counts))
		m := adcChunk(counts[i:j], b64)
		if c.sim.firmware {
			m["status"] = "chunk"
		}
		if i == 0 {
			m["status"] = "ok"
			m["adc_rate_hz"] = int(rate)
			m["bits"] = 16
		}
		if final && j >= len(counts) && c.trigger != nil {
			m["trigger"] = c.trigger
		}
		m["more"] = !(final && j >= len(counts))
		if err := c.send(m); err != nil {
			return err
		}
		if j >= len(counts) {
			break
		}
	}
	return nil
}

func (s *Sim) cmdCapture(c *conn, r request) error {
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	n := r.int("samples", 256)
	if n <= 0 || n > 32768 {
		if s.firmware {
			return fail(fwError("samples out of range (use capture_dual for deep captures)"))
		}
		return fail(fwError("samples out of range"))
	}
	mhz, _ := r.num("sample_rate_mhz")
	rate := adcRate(mhz * 1e6)
	n = s.clampSamples(n, maxAdcSamples, rate)
	if err := s.beginCapture(c); err != nil {
		return fail(err)
	}
	defer s.endCapture()
	s.mu.Lock()
	t0 := s.now()
	s.mu.Unlock()
	s.captureStart(t0, nil)
	if !s.sleepUntil(t0 + int64(float64(n)/rate*1e9)) {
		return fail(fwError("pod shutting down"))
	}
	s.mu.Lock()
	s.advance(s.now())
	counts := s.adcCounts(t0, n, rate)
	s.mu.Unlock()
	return sendAdc(c, counts, rate, r.str("enc") == "b64", true)
}

// cmdCaptureDual is one trigger for an ADC and an LA stream (either may be empty): the deep
// ADC capture and the correlated capture.
func (s *Sim) cmdCaptureDual(c *conn, r request) error {
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	na, nl := r.int("adc_samples", 0), r.int("la_samples", 0)
	if s.firmware && na == 0 && nl == 0 {
		return fail(fwError("adc_samples and la_samples are both 0"))
	}
	if s.firmware && (na+nl)*2 > 8<<20 {
		return fail(fwError("capture too deep: LA+ADC+DAC exceed 8 MB PSRAM"))
	}
	if na < 0 || nl < 0 || na+nl == 0 || na > 8_000_000 || nl > 4_000_000 {
		return fail(fwError("samples out of range"))
	}
	if nl > 0 {
		s.mu.Lock()
		err := s.requireLaVoltage()
		s.mu.Unlock()
		if err != nil {
			return fail(err)
		}
	}
	amhz, _ := r.num("adc_rate_mhz")
	lmhz, _ := r.num("la_rate_mhz")
	ar, lr := adcRate(amhz*1e6), laRate(lmhz*1e6)
	na = s.clampSamples(na, maxAdcSamples, ar)
	if nl > 0 {
		nl = s.clampSamples(nl, maxLaSamples, lr)
	}
	ts, err := parseTrigger(r)
	if err != nil {
		return fail(err)
	}
	if err := s.beginCapture(c); err != nil {
		return fail(err)
	}
	defer s.endCapture()
	s.mu.Lock()
	t0 := s.now()
	s.mu.Unlock()
	if ts != nil {
		if t0, err = s.waitTrigger(ts); err != nil {
			return fail(err)
		}
		if nl > 0 {
			t0 -= ts.pretrigger(lr)
		}
	}
	dur := int64(0)
	if na > 0 {
		dur = int64(float64(na) / ar * 1e9)
	}
	if nl > 0 {
		dur = max(dur, int64(float64(nl)/lr*1e9))
	}
	stopUs := s.captureStart(t0, r)
	if !s.sleepUntil(t0 + dur) {
		return fail(fwError("pod shutting down"))
	}
	s.mu.Lock()
	s.advance(s.now())
	var counts []int
	if na > 0 {
		counts = s.adcCounts(t0, na, ar)
	}
	var edges [][2]int
	if nl > 0 {
		edges = s.sig.renderRLE(t0, nl, lr)
	}
	s.captureEnd(stopUs)
	s.pod.lastCap = &lastCapture{adc: counts, ar: ar, edges: edges, nl: nl, lr: lr}
	s.mu.Unlock()
	if ts != nil {
		c.trigger = map[string]any{"la": ts.la, "edge": ts.edge, "fired": true}
	}
	if na > 0 {
		if err := sendAdc(c, counts, ar, r.str("enc") == "b64", nl == 0); err != nil {
			return nil
		}
	}
	if nl > 0 {
		var first map[string]any
		if na > 0 {
			first = map[string]any{"status": "chunk"}
		}
		return laFrames(c, edges, nl, lr, first, true)
	}
	return nil
}

// lastCapture is what stays in PSRAM until the next capture (capture_read resumes from it).
type lastCapture struct {
	adc   []int
	ar    float64
	edges [][2]int
	nl    int
	lr    float64
}

// captureStart is a capture's t0 in the fabric: a co-triggered DAC starts on it and the auto-stop
// threshold counts from it. It returns the armed stop, 0 for none.
func (s *Sim) captureStart(t0 int64, r request) int64 {
	if !s.firmware {
		return 0
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	d := &s.pod.analog.dac
	stop := int64(0)
	if r != nil {
		stop = int64(r.int("stop_dac_after_us", 0))
	}
	if d.armed || stop > 0 {
		s.snapshotAnalog(t0)
		if d.armed {
			d.armed, d.t0 = false, t0
		}
		if stop > 0 {
			d.stopAt = t0 + stop*1000
		}
	}
	return stop
}

// captureEnd is the firmware's cleanup after an auto-stop: the official dac_stop. Callers hold s.mu.
func (s *Sim) captureEnd(stop int64) {
	if stop <= 0 {
		return
	}
	now := s.now()
	s.snapshotAnalog(now)
	d := &s.pod.analog.dac
	*d = dacState{path: d.path, code: d.codeAt(now)}
}

// cmdCaptureRead is handle_capture_read: the last capture again from a sample index.
func (s *Sim) cmdCaptureRead(c *conn, r request) error {
	s.mu.Lock()
	lc := s.pod.lastCap
	s.mu.Unlock()
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	if lc == nil {
		return fail(fwError("no capture to resume (run capture_dual first)"))
	}
	off := r.int("offset", 0)
	na := len(lc.adc)
	total := na + lc.nl
	if off > total || off < 0 {
		return fail(fwError("offset out of range"))
	}
	if off == total {
		return c.send(map[string]any{"status": "ok", "data": []int{}, "more": false})
	}
	if off < na {
		for i := off; i < na; i += samplesPerChnk {
			j := min(i+samplesPerChnk, na)
			m := adcChunk(lc.adc[i:j], r.str("enc") == "b64")
			m["status"] = "chunk"
			m["more"] = !(j >= na && lc.nl == 0)
			if err := c.send(m); err != nil {
				return nil
			}
		}
		off = na
		if lc.nl == 0 {
			return nil
		}
	}
	var edges [][2]int
	lastW := -1
	for _, e := range lc.edges {
		if e[0] < off-na {
			lastW = e[1]
			continue
		}
		edges = append(edges, e)
	}
	if lastW >= 0 && (len(edges) == 0 || edges[0][0] != off-na) {
		edges = append([][2]int{{off - na, lastW}}, edges...)
	}
	return laFrames(c, edges, lc.nl, lc.lr, map[string]any{"status": "chunk"}, true)
}
