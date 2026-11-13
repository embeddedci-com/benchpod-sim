package benchsim

import (
	"encoding/base64"
)

// The SPI master on four LA pins (command_handler_spi.c) with a W25Q64 on the far end: the bench
// has one when a firmware Sim runs, on whatever pins the session names.

const (
	flashSize  = 8 << 20
	flashStart = 0xFF
)

type spiMaster struct {
	sck, mosi, miso, cs, hz, mode int
	csHeld                        bool
	txn                           []byte // the bytes of the transaction under CS so far
	wel                           bool
}

type spiFlashChip struct{ mem []byte }

func (f *spiFlashChip) bytes() []byte {
	if f.mem == nil {
		f.mem = make([]byte, flashSize)
		for i := range f.mem {
			f.mem[i] = flashStart
		}
	}
	return f.mem
}

func (s *Sim) spiSession() (*spiMaster, error) {
	if s.pod.spi == nil {
		return nil, fwError("no SPI session: send spi_start first")
	}
	return s.pod.spi, nil
}

func (s *Sim) cmdSpiStart(r request) (any, error) {
	keys := []string{"sck", "mosi", "miso", "cs"}
	fns := []string{"spi_sck", "spi_mosi", "spi_miso", "spi_cs"}
	v := make([]int, 4)
	for i, k := range keys {
		if !r.has(k) || r.int(k, 0) < 1 || r.int(k, 0) > 14 {
			return nil, errf("missing or invalid %s (LA 1..14)", k)
		}
		v[i] = r.int(k, 0)
		for j := 0; j < i; j++ {
			if v[j] == v[i] {
				return nil, fwError("each SPI signal needs its own LA pin")
			}
		}
	}
	hz := r.int("hz", 1000000)
	if hz <= 0 {
		return nil, fwError("invalid hz")
	}
	mode := r.int("mode", 0)
	if mode != 0 && mode != 3 {
		return nil, fwError("mode must be 0 or 3")
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.pod.spi != nil {
		return nil, fwError("spi busy: send spi_stop first")
	}
	if err := s.requireLaVoltage(); err != nil {
		return nil, err
	}
	for i := range v {
		if err := s.checkClaim(fns[i], v[i:i+1]); err != nil {
			return nil, err
		}
	}
	for i := range v {
		s.pod.pins[v[i]] = laPin{fn: fns[i]}
	}
	half := max(1, 24000000/hz/2)
	got := 24000000 / (2 * half)
	s.pod.spi = &spiMaster{sck: v[0], mosi: v[1], miso: v[2], cs: v[3], hz: got, mode: mode}
	return map[string]any{"sck": v[0], "mosi": v[1], "miso": v[2], "cs": v[3], "hz": got, "mode": mode}, nil
}

func (s *Sim) cmdSpiStop(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pod.spi = nil
	s.releaseFn("spi_sck", "spi_mosi", "spi_miso", "spi_cs")
	return "spi stopped", nil
}

func (s *Sim) cmdSpiStatus(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	m := s.pod.spi
	if m == nil {
		return map[string]any{"active": false}, nil
	}
	return map[string]any{"active": true, "sck": m.sck, "mosi": m.mosi, "miso": m.miso, "cs": m.cs, "hz": m.hz, "mode": m.mode, "cs_held": m.csHeld}, nil
}

// flashExchange is the W25Q on the wire for one CS-low stretch: it answers tx and, when CS rises,
// applies the command. txn holds what earlier spi_xfer calls of this stretch sent.
func (s *Sim) flashExchange(m *spiMaster, tx []byte, release bool) []byte {
	rx := make([]byte, len(tx))
	start := len(m.txn)
	m.txn = append(m.txn, tx...)
	mem := s.flash.chip.bytes()
	addr := func() int { return int(m.txn[1])<<16 | int(m.txn[2])<<8 | int(m.txn[3]) }
	switch m.txn[0] {
	case 0x9F:
		id := []byte{0xEF, 0x40, 0x17}
		for i := range rx {
			if k := start + i - 1; k >= 0 && k < 3 {
				rx[i] = id[k]
			}
		}
	case 0x05:
		for i := range rx {
			if start+i >= 1 && m.wel {
				rx[i] = 0x02
			}
		}
	case 0x03:
		if len(m.txn) >= 4 {
			base := addr()
			for i := range rx {
				if k := start + i - 4; k >= 0 {
					rx[i] = mem[(base+k)%flashSize]
				}
			}
		}
	}
	if release {
		switch m.txn[0] {
		case 0x06:
			m.wel = true
		case 0x04:
			m.wel = false
		case 0x02:
			if m.wel && len(m.txn) > 4 {
				base := addr()
				for i, b := range m.txn[4:] {
					a := base&^0xFF | (base+i)&0xFF
					mem[a%flashSize] &= b
				}
			}
			m.wel = false
		case 0x20, 0x52, 0xD8:
			if m.wel && len(m.txn) >= 4 {
				n := map[byte]int{0x20: 4096, 0x52: 32768, 0xD8: 65536}[m.txn[0]]
				base := addr() &^ (n - 1)
				for i := 0; i < n; i++ {
					mem[(base+i)%flashSize] = flashStart
				}
			}
			m.wel = false
		case 0xC7, 0x60:
			if m.wel {
				for i := range mem {
					mem[i] = flashStart
				}
			}
			m.wel = false
		}
		m.txn = m.txn[:0]
	}
	return rx
}

func csMode(r request) (hold bool, err error) {
	switch v := r.str("cs"); {
	case !r.has("cs") || v == "release":
		return false, nil
	case v == "hold":
		return true, nil
	}
	return false, fwError("cs must be release or hold")
}

func (s *Sim) cmdSpiXfer(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	m, err := s.spiSession()
	if err != nil {
		return nil, err
	}
	tx, derr := base64.RawURLEncoding.DecodeString(r.str("tx"))
	if derr != nil || len(tx) == 0 || len(tx) > 768 {
		return nil, fwError("tx must be 1..768 bytes of base64url")
	}
	hold, err := csMode(r)
	if err != nil {
		return nil, err
	}
	rx := s.flashExchange(m, tx, !hold)
	m.csHeld = hold
	state := "released"
	if hold {
		state = "held"
	}
	return map[string]any{"rx": base64.RawURLEncoding.EncodeToString(rx), "cs": state}, nil
}

func (s *Sim) cmdSpiStream(c *conn, r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	m, err := s.spiSession()
	if err != nil {
		return nil, err
	}
	if !r.has("len") {
		return nil, fwError("missing or invalid len")
	}
	n := r.int("len", 0)
	var head []byte
	if r.has("head") {
		var derr error
		head, derr = base64.RawURLEncoding.DecodeString(r.str("head"))
		if derr != nil || len(head) > 64 {
			return nil, fwError("head must be up to 64 bytes of base64url")
		}
	}
	hold, err := csMode(r)
	if err != nil {
		return nil, err
	}
	if n > 0 && len(s.pod.staged) == 0 {
		return nil, fwError("nothing staged: load_bin with \"psram\":true first")
	}
	if n > len(s.pod.staged) {
		return nil, fwError("len is more than the staged upload")
	}
	if n == 0 && len(head) == 0 {
		return nil, fwError("nothing to send")
	}
	if err := s.heavyBegin(c); err != nil {
		return nil, err
	}
	defer s.heavyRelease(c)
	if m.csHeld {
		s.flashExchange(m, nil, true)
	}
	tx := append(append([]byte(nil), head...), s.pod.staged[:n]...)
	s.flashExchange(m, tx, !hold)
	m.csHeld = hold
	state := "released"
	if hold {
		state = "held"
	}
	return map[string]any{"sent": len(tx), "ms": 1, "cs": state}, nil
}

// cmdSpiFlash is handle_spi_flash's ops against the chip.
func (s *Sim) cmdSpiFlash(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, err := s.spiSession(); err != nil {
		return nil, err
	}
	mem := s.flash.chip.bytes()
	addr, hasAddr := r.num("addr")
	a := int(addr)
	switch r.str("op") {
	case "id":
		return map[string]any{"id": "ef4017", "present": true, "size": flashSize, "status": 0}, nil
	case "read":
		n := r.int("len", 0)
		if !hasAddr || n <= 0 || n > 1024 {
			return nil, fwError("read needs addr and len 1..1024")
		}
		if a+n > flashSize {
			return nil, fwError("address past the end of the flash")
		}
		return map[string]any{"addr": a, "len": n, "data": base64.RawURLEncoding.EncodeToString(mem[a : a+n])}, nil
	case "erase":
		n := r.int("len", 0)
		if !hasAddr || n <= 0 || n > 1<<20 {
			return nil, fwError("erase needs addr and len 1..1048576")
		}
		start := a &^ 4095
		end := (a + n + 4095) &^ 4095
		for i := start; i < end && i < flashSize; i++ {
			mem[i] = flashStart
		}
		return map[string]any{"addr": start, "len": end - start, "ms": 1}, nil
	case "chip_erase":
		for i := range mem {
			mem[i] = flashStart
		}
		return map[string]any{"ms": 1}, nil
	case "write":
		if !hasAddr {
			return nil, fwError("write needs addr")
		}
		data, derr := base64.RawURLEncoding.DecodeString(r.str("data"))
		if derr != nil || len(data) == 0 || len(data) > 768 {
			return nil, fwError("data must be 1..768 bytes of base64url")
		}
		for i, b := range data {
			mem[(a+i)%flashSize] &= b
		}
		verify := !r.has("verify") || r.flag("verify")
		if verify {
			for i, b := range data {
				if mem[(a+i)%flashSize] != b {
					return nil, errf("verify failed at 0x%06x: not erased, write-protected, or a bad wire", a+i)
				}
			}
		}
		return map[string]any{"addr": a, "len": len(data), "verified": verify, "ms": 1}, nil
	}
	return nil, fwError("op must be id, read, erase, chip_erase or write")
}
