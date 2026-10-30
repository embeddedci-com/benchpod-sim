package benchsim

import (
	"math"
	"sort"
)

// The LA lines keep a short history so a capture, a trigger or a decode sees what happened on
// the wire. A line's level is its idle level (base: power, pin modes, the alarm output, the pod's
// drivers) with protocol activity on top: UART frames on the DUT's TX and the pod's TX, I2C
// transfers on the sensor bus, and short strobe pulses. Everything is time-stamped in
// simulation nanoseconds and pruned after historyNs, and every list is capped, so a long-lived
// visitor costs a few hundred KB at most.
const (
	historyNs      = int64(30e9)
	maxBaseEntries = 16384
	maxPulses      = 8192
	maxI2CTxns     = 2048
	maxStreamBytes = 32 << 10
	// maxEvents caps the edges one capture renders (a dense I2C/UART window at a high rate).
	maxEvents = 600_000
)

type baseEntry struct {
	t int64
	w uint16
}

// pulse is a short high pulse on one channel (the node's strobe output).
type pulse struct {
	start, end int64
	ch         int
}

// byteChunk is a run of back-to-back 8N1 UART frames on one channel.
type byteChunk struct {
	start  int64
	byteNs int64
	ch     int
	seq    uint64 // sequence number of data[0] in its stream
	data   []byte
}

func (c *byteChunk) end() int64 { return c.start + int64(len(c.data))*c.byteNs }

// byteStream is one UART direction: the DUT's TX, or the pod's TX into the DUT.
type byteStream struct {
	chunks []byteChunk
	next   uint64 // sequence number of the next byte written
	end    int64  // when the last queued byte has left the wire
	size   int
}

// write queues p on the wire at t (after whatever is still being sent) and returns when its
// first byte starts.
func (b *byteStream) write(t int64, ch int, byteNs int64, p []byte) int64 {
	if len(p) == 0 {
		return t
	}
	start := t
	if b.end > start {
		start = b.end
	}
	if n := len(b.chunks); n > 0 {
		last := &b.chunks[n-1]
		if last.end() == start && last.ch == ch && last.byteNs == byteNs && len(last.data) < 4096 {
			last.data = append(last.data, p...)
			goto done
		}
	}
	b.chunks = append(b.chunks, byteChunk{start: start, byteNs: byteNs, ch: ch, seq: b.next, data: append([]byte(nil), p...)})
done:
	b.next += uint64(len(p))
	b.size += len(p)
	b.end = start + int64(len(p))*byteNs
	return start
}

// readUpTo returns the bytes after seq whose frame has completed by t, and the new cursor.
// lost reports bytes that were pruned before the reader got to them.
func (b *byteStream) readUpTo(seq uint64, t int64) (out []byte, next uint64, lost bool) {
	next = seq
	for i := range b.chunks {
		c := &b.chunks[i]
		cEnd := c.seq + uint64(len(c.data))
		if cEnd <= next {
			continue
		}
		if c.seq > next {
			lost = true
			next = c.seq
		}
		for k := int(next - c.seq); k < len(c.data); k++ {
			if c.start+int64(k+1)*c.byteNs > t {
				return out, next, lost
			}
			out = append(out, c.data[k])
			next++
		}
	}
	return out, next, lost
}

func (b *byteStream) prune(before int64) {
	i := 0
	for i < len(b.chunks) && (b.chunks[i].end() < before || b.size-len(b.chunks[i].data) > maxStreamBytes) {
		b.size -= len(b.chunks[i].data)
		i++
	}
	if i > 0 {
		b.chunks = append([]byteChunk(nil), b.chunks[i:]...)
	}
}

// i2cSeg is one addressed part of an I2C transfer: START (or repeated START), address + R/W,
// data bytes. ack[0] is the address ACK, ack[k] the ACK after data[k-1] (the target's on a
// write, the controller's on a read).
type i2cSeg struct {
	addr byte
	read bool
	data []byte
	ack  []bool
}

type i2cTxn struct {
	start int64
	segs  []i2cSeg
}

// I2C timing of the node's bus: 100 kHz standard mode, a quarter bit is 2.5 us.
const i2cQuarterNs = int64(2500)

func (x *i2cTxn) duration() int64 {
	bits := int64(0)
	for i, sg := range x.segs {
		if i > 0 {
			bits++ // repeated START
		}
		bits += int64(9 * (1 + len(sg.data)))
		if !sg.ack[0] {
			bits -= int64(9 * len(sg.data))
			break
		}
	}
	return 2*i2cQuarterNs + bits*4*i2cQuarterNs + 3*i2cQuarterNs
}

type signals struct {
	base   []baseEntry
	pulses []pulse
	dutTx  byteStream
	podTx  byteStream
	i2c    []i2cTxn
	busEnd int64
}

func (g *signals) pushBase(t int64, w uint16) {
	if n := len(g.base); n > 0 {
		last := g.base[n-1]
		if last.w == w {
			return
		}
		if t < last.t {
			t = last.t
		}
	}
	g.base = append(g.base, baseEntry{t, w})
	if len(g.base) > maxBaseEntries {
		g.base = append([]baseEntry(nil), g.base[len(g.base)-maxBaseEntries/2:]...)
	}
}

func (g *signals) addPulse(start, end int64, ch int) {
	g.pulses = append(g.pulses, pulse{start, end, ch})
	if len(g.pulses) > maxPulses {
		g.pulses = append([]pulse(nil), g.pulses[len(g.pulses)-maxPulses/2:]...)
	}
}

// addI2C queues a transfer on the bus at t (after the one in progress) and returns its span.
func (g *signals) addI2C(t int64, segs []i2cSeg) (start, end int64) {
	start = t
	if g.busEnd > start {
		start = g.busEnd
	}
	x := i2cTxn{start: start, segs: segs}
	end = start + x.duration()
	g.busEnd = end
	g.i2c = append(g.i2c, x)
	if len(g.i2c) > maxI2CTxns {
		g.i2c = append([]i2cTxn(nil), g.i2c[len(g.i2c)-maxI2CTxns/2:]...)
	}
	return start, end
}

func (g *signals) prune(now int64) {
	before := now - historyNs
	// Keep the last base entry before the cutoff: it is the level at the cutoff.
	i := sort.Search(len(g.base), func(i int) bool { return g.base[i].t >= before })
	if i > 1 {
		g.base = append([]baseEntry(nil), g.base[i-1:]...)
	}
	j := 0
	for j < len(g.pulses) && g.pulses[j].end < before {
		j++
	}
	if j > 0 {
		g.pulses = append([]pulse(nil), g.pulses[j:]...)
	}
	k := 0
	for k < len(g.i2c) && g.i2c[k].start+g.i2c[k].duration() < before {
		k++
	}
	if k > 0 {
		g.i2c = append([]i2cTxn(nil), g.i2c[k:]...)
	}
	g.dutTx.prune(before)
	g.podTx.prune(before)
}

func (g *signals) baseAt(t int64) uint16 {
	i := sort.Search(len(g.base), func(i int) bool { return g.base[i].t > t })
	if i == 0 {
		return 0
	}
	return g.base[i-1].w
}

// wordAt is every channel's level at t, protocol activity included.
func (g *signals) wordAt(t int64) uint16 {
	w, _ := g.events(t, t, 0x3FFF)
	return w
}

type edgeEv struct {
	t   int64
	ch  int
	lvl uint8
}

// lookbackNs is how far before a window the history is replayed to find each line's level at
// the window's start (a UART frame or an I2C transfer in progress). It covers a whole frame down
// to 2400 baud and the longest transfer of the node.
const lookbackNs = int64(5e6)

// events returns the level word at t0 and every level change on the masked channels in
// (t0, t1], in time order. Protocol activity is drawn on top of the idle level.
func (g *signals) events(t0, t1 int64, mask uint16) (uint16, []edgeEv) {
	lb := t0 - lookbackNs
	initial := g.baseAt(lb)
	var evs []edgeEv
	add := func(t int64, ch int, lvl uint8) {
		if t > lb && t <= t1 && mask&(1<<(ch-1)) != 0 && len(evs) < maxEvents {
			evs = append(evs, edgeEv{t, ch, lvl})
		}
	}
	// Idle-level changes.
	i := sort.Search(len(g.base), func(i int) bool { return g.base[i].t > lb })
	prev := initial
	for ; i < len(g.base) && g.base[i].t <= t1; i++ {
		w := g.base[i].w
		for ch := 1; ch <= 14; ch++ {
			bit := uint16(1) << (ch - 1)
			if (w^prev)&bit != 0 {
				add(g.base[i].t, ch, uint8((w&bit)>>(ch-1)))
			}
		}
		prev = w
	}
	for _, p := range g.pulses {
		if p.end >= lb && p.start <= t1 {
			add(p.start, p.ch, 1)
			add(p.end, p.ch, 0)
		}
	}
	for _, st := range []*byteStream{&g.dutTx, &g.podTx} {
		for ci := range st.chunks {
			c := &st.chunks[ci]
			if c.end() < lb || c.start > t1 || mask&(1<<(c.ch-1)) == 0 {
				continue
			}
			bitNs := c.byteNs / 10
			k0 := 0
			if c.start < lb {
				k0 = int((lb - c.start) / c.byteNs)
			}
			for k := k0; k < len(c.data); k++ {
				ts := c.start + int64(k)*c.byteNs
				if ts > t1 || len(evs) >= maxEvents {
					break
				}
				b := c.data[k]
				add(ts, c.ch, 0) // start bit
				for bit := 0; bit < 8; bit++ {
					add(ts+int64(1+bit)*bitNs, c.ch, (b>>bit)&1)
				}
				add(ts+9*bitNs, c.ch, 1) // stop bit
			}
		}
	}
	for xi := range g.i2c {
		x := &g.i2c[xi]
		if x.start > t1 || x.start+x.duration() < lb {
			continue
		}
		x.edges(func(t int64, scl, sda int8) {
			if scl >= 0 {
				add(t, dutSCL, uint8(scl))
			}
			if sda >= 0 {
				add(t, dutSDA, uint8(sda))
			}
		})
	}
	sort.SliceStable(evs, func(a, b int) bool { return evs[a].t < evs[b].t })
	k := 0
	for ; k < len(evs) && evs[k].t <= t0; k++ {
		e := evs[k]
		if e.lvl == 1 {
			initial |= 1 << (e.ch - 1)
		} else {
			initial &^= 1 << (e.ch - 1)
		}
	}
	return initial, evs[k:]
}

// edges walks the transfer's bus levels: emit(t, scl, sda) with -1 for "unchanged".
func (x *i2cTxn) edges(emit func(t int64, scl, sda int8)) {
	q := i2cQuarterNs
	t := x.start
	emit(t, -1, 0) // START: SDA falls while SCL is high
	emit(t+q, 0, -1)
	t += 2 * q
	bit := func(v int8) {
		emit(t, -1, v)
		emit(t+q, 1, -1)
		emit(t+3*q, 0, -1)
		t += 4 * q
	}
	byteOut := func(b byte, ack bool) {
		for i := 7; i >= 0; i-- {
			bit(int8((b >> i) & 1))
		}
		if ack {
			bit(0)
		} else {
			bit(1)
		}
	}
	for i, sg := range x.segs {
		if i > 0 { // repeated START
			emit(t, -1, 1)
			emit(t+q, 1, -1)
			emit(t+2*q, -1, 0)
			emit(t+3*q, 0, -1)
			t += 4 * q
		}
		a := sg.addr << 1
		if sg.read {
			a |= 1
		}
		byteOut(a, sg.ack[0])
		if !sg.ack[0] {
			break
		}
		for k, d := range sg.data {
			byteOut(d, sg.ack[k+1])
		}
	}
	// STOP: SDA low, SCL high, then SDA rises.
	emit(t, -1, 0)
	emit(t+q, 1, -1)
	emit(t+2*q, -1, 1)
}

// sampleIndex is the first sample at or after time t of a capture starting at t0.
func sampleIndex(t, t0 int64, rate float64) int {
	return int(math.Ceil(float64(t-t0) * rate / 1e9))
}

// renderRLE turns a window into the firmware's run-length LA frames: [[sampleIndex, word], ...]
// starting with sample 0. Several changes inside one sample period collapse into the last one,
// which is what a real sampler sees.
func (g *signals) renderRLE(t0 int64, n int, rate float64) [][2]int {
	t1 := t0 + int64(float64(n)/rate*1e9)
	cur, evs := g.events(t0, t1, 0x3FFF)
	out := [][2]int{{0, int(cur)}}
	for i := 0; i < len(evs); {
		idx := sampleIndex(evs[i].t, t0, rate)
		if idx >= n {
			break
		}
		for ; i < len(evs) && sampleIndex(evs[i].t, t0, rate) == idx; i++ {
			e := evs[i]
			if e.lvl == 1 {
				cur |= 1 << (e.ch - 1)
			} else {
				cur &^= 1 << (e.ch - 1)
			}
		}
		last := &out[len(out)-1]
		switch {
		case int(cur) == last[1]:
		case idx == last[0]:
			last[1] = int(cur)
		default:
			out = append(out, [2]int{idx, int(cur)})
		}
	}
	return out
}

// findTrigger returns the first moment in (from, to] the trigger condition holds on channel ch:
// an edge (rising/falling) or a level (high/low, which also holds at from itself).
func (g *signals) findTrigger(ch int, edge string, from, to int64) (int64, bool) {
	w, evs := g.events(from, to, 1<<(ch-1))
	lvl := uint8((w >> (ch - 1)) & 1)
	if (edge == "high" && lvl == 1) || (edge == "low" && lvl == 0) {
		return from, true
	}
	for _, e := range evs {
		if e.lvl == lvl {
			continue
		}
		lvl = e.lvl
		switch {
		case edge == "rising" && lvl == 1, edge == "falling" && lvl == 0,
			edge == "high" && lvl == 1, edge == "low" && lvl == 0:
			return e.t, true
		}
	}
	return 0, false
}
