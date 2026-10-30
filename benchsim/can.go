package benchsim

// Classic CAN on FDCAN1 / TCAN1044 (command_handler_can.c, can_bus.c). The pod is one node: the
// loopback modes echo its frames into its own RX FIFO, normal mode has nobody to ACK them.

const canRxMax = 64

type podCanFrame struct {
	id       uint32
	ext, rtr bool
	data     []int
	ts       int64
}

type podCanRule struct {
	matchID  uint32
	matchExt bool
	reply    podCanFrame
}

type podCan struct {
	enabled  bool
	mode     string
	bitrate  int
	term     bool
	rx       []podCanFrame
	overflow int
	rules    []podCanRule
	hits     int
}

func podCanTiming(bitrate int) bool {
	if bitrate <= 0 || 25000000%bitrate != 0 {
		return false
	}
	tq := 25000000 / bitrate
	for presc := 1; presc <= 32; presc++ {
		if tq%presc == 0 && tq/presc >= 8 && tq/presc <= 81 {
			return true
		}
	}
	return false
}

func byteArray(v any) []int {
	list, _ := v.([]any)
	out := []int{}
	for _, x := range list {
		if f, ok := x.(float64); ok && len(out) < 8 {
			out = append(out, int(f)&0xFF)
		}
	}
	return out
}

func (s *Sim) cmdCanConfig(r request) (any, error) {
	bitrate := r.int("bitrate", 500000)
	mode := "normal"
	if r.has("mode") {
		mode = r.str("mode")
	}
	if mode != "normal" && mode != "internal" && mode != "external" && mode != "listen" {
		return nil, fwError("mode must be normal|internal|external|listen")
	}
	if !podCanTiming(bitrate) {
		return nil, fwError("unsupported bitrate (no exact bit timing)")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	c := &s.pod.can
	c.enabled, c.mode, c.bitrate, c.term = true, mode, bitrate, r.flag("term")
	c.rx, c.overflow = nil, 0
	return map[string]any{"bitrate": bitrate, "mode": mode, "term": c.term}, nil
}

func (s *Sim) canReceive(f podCanFrame) {
	c := &s.pod.can
	f.ts = s.now() / 1e6
	if len(c.rx) >= canRxMax {
		c.overflow++
		return
	}
	c.rx = append(c.rx, f)
}

func (s *Sim) cmdCanWrite(r request) (any, error) {
	if !r.has("id") {
		return nil, fwError("missing id")
	}
	f := podCanFrame{id: uint32(r.int("id", 0)), ext: r.flag("ext"), rtr: r.flag("rtr"), data: byteArray(r["data"])}
	s.mu.Lock()
	defer s.mu.Unlock()
	c := &s.pod.can
	if !c.enabled {
		return nil, fwError("can not enabled")
	}
	if c.mode == "listen" {
		return nil, fwError("tx failed (fifo full?)")
	}
	if c.mode == "internal" || c.mode == "external" {
		s.canReceive(f)
		for _, rule := range c.rules {
			if rule.matchID == f.id && rule.matchExt == f.ext {
				c.hits++
				s.canReceive(rule.reply)
			}
		}
	}
	return map[string]any{"id": f.id, "ext": f.ext, "dlc": len(f.data)}, nil
}

func (s *Sim) cmdCanRead(r request) (any, error) {
	want := max(1, min(r.int("max", 8), 8))
	s.mu.Lock()
	defer s.mu.Unlock()
	c := &s.pod.can
	n := min(want, len(c.rx))
	frames := []any{}
	for _, f := range c.rx[:n] {
		frames = append(frames, map[string]any{"id": f.id, "ext": f.ext, "rtr": f.rtr, "dlc": len(f.data), "data": f.data, "ts": f.ts})
	}
	c.rx = append(c.rx[:0], c.rx[n:]...)
	return map[string]any{"frames": frames, "overflow": c.overflow}, nil
}

func (s *Sim) cmdCanStatus(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	c := s.pod.can
	mode := c.mode
	if mode == "" {
		mode = "normal"
	}
	return map[string]any{"enabled": c.enabled, "mode": mode, "bitrate": c.bitrate, "term": c.term, "tec": 0, "rec": 0,
		"bus_off": false, "error_passive": false, "rx_pending": len(c.rx), "rx_overflow": c.overflow,
		"responder_rules": len(c.rules), "responder_hits": c.hits, "bus_off_recoveries": 0}, nil
}

func (s *Sim) cmdCanRespond(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	c := &s.pod.can
	if r.flag("clear") {
		c.rules, c.hits = nil, 0
		return map[string]any{"rules": 0}, nil
	}
	if !r.has("match_id") {
		return nil, fwError("missing match_id")
	}
	if !r.has("reply_id") {
		return nil, fwError("missing reply_id")
	}
	if len(c.rules) >= 8 {
		return nil, fwError("responder table full")
	}
	c.rules = append(c.rules, podCanRule{matchID: uint32(r.int("match_id", 0)), matchExt: r.flag("ext"),
		reply: podCanFrame{id: uint32(r.int("reply_id", 0)), ext: r.flag("reply_ext"), rtr: r.flag("reply_rtr"), data: byteArray(r["reply_data"])}})
	return map[string]any{"rule": len(c.rules) - 1, "rules": len(c.rules)}, nil
}

func (s *Sim) cmdCanTerm(r request) (any, error) {
	if !r.has("on") {
		return nil, fwError("missing on")
	}
	v := r.str("on")
	on := v != "" && (v[0] == '1' || v[0] == 't' || v[0] == 'T' || v[0] == 'y' || v[0] == 'Y')
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pod.can.term = on
	return map[string]any{"term": on}, nil
}

func (s *Sim) cmdCanDisable(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pod.can.enabled = false
	return map[string]any{"enabled": false}, nil
}
