package benchsim

import (
	"math"
	"time"
)

// The UART proxy (command_handler_uart.c): after the "uart ready" ack the connection carries
// raw 8N1 bytes both ways until the client closes it. The pod samples rx (wire the DUT's TX
// there) and drives tx (the DUT's RX). Only a wire that carries the node's console at its baud
// gives readable text: another channel is silent, another baud is garbage.

type uartProxy struct {
	rx, tx, baud int
	cursor       uint64 // next DUT TX byte to forward
}

// proxyPoll is how often the proxy forwards what the DUT sent. 115200 baud fills ~60 bytes in
// that time, the size of a short console line.
const proxyPoll = 5 * time.Millisecond

func (s *Sim) cmdUartProxy(c *conn, r request) error {
	fail := func(err error) error {
		_ = c.send(errReply(err.Error()))
		return err
	}
	if !r.has("rx") {
		return fail(fwError("missing rx"))
	}
	if !r.has("tx") {
		return fail(fwError("missing tx"))
	}
	rx, tx, baud := r.int("rx", 0), r.int("tx", 0), r.int("baud", 115200)
	if rx < 1 || rx > 14 || tx < 1 || tx > 14 || rx == tx || baud < 300 || baud > 4000000 {
		return fail(fwError("invalid uart args"))
	}
	s.mu.Lock()
	if err := s.requireLaVoltage(); err != nil {
		s.mu.Unlock()
		return fail(err)
	}
	if err := s.checkClaim("uart_rx", []int{rx}); err != nil {
		s.mu.Unlock()
		return fail(err)
	}
	if err := s.checkClaim("uart_tx", []int{tx}); err != nil {
		s.mu.Unlock()
		return fail(err)
	}
	now := s.now()
	s.advance(now)
	p := &uartProxy{rx: rx, tx: tx, baud: baud, cursor: s.sig.dutTx.next}
	s.pod.pins[rx] = laPin{fn: "uart_rx"}
	s.pod.pins[tx] = laPin{fn: "uart_tx"}
	s.pod.proxy = p
	s.touchBase(now)
	s.mu.Unlock()

	defer func() {
		s.mu.Lock()
		if s.pod.proxy == p {
			s.pod.proxy = nil
			s.releaseFn("uart_rx", "uart_tx")
			t := s.now()
			s.advance(t)
			s.touchBase(t)
		}
		s.mu.Unlock()
	}()
	if err := c.send(okReply("uart ready")); err != nil {
		return nil
	}

	// Reader: what the client types goes onto the tx wire.
	stop := make(chan struct{})
	go func() {
		defer close(stop)
		buf := make([]byte, 512)
		for {
			n, err := c.br.Read(buf)
			if n > 0 {
				s.mu.Lock()
				t := s.now()
				s.advance(t)
				s.podUartWrite(t, tx, baud, buf[:n])
				s.mu.Unlock()
			}
			if err != nil {
				return
			}
		}
	}()

	tick := time.NewTicker(proxyPoll)
	defer tick.Stop()
	for {
		select {
		case <-stop:
			_ = c.Close()
			return nil
		case <-s.done:
			_ = c.Close()
			<-stop
			return nil
		case <-tick.C:
		}
		s.mu.Lock()
		t := s.now()
		s.advance(t)
		out, next, _ := s.sig.dutTx.readUpTo(p.cursor, t)
		p.cursor = next
		s.mu.Unlock()
		if len(out) == 0 {
			continue
		}
		out = p.received(out)
		if len(out) == 0 {
			continue
		}
		_ = c.SetWriteDeadline(time.Now().Add(5 * time.Second))
		if _, err := c.Write(out); err != nil {
			_ = c.Close()
			<-stop
			return nil
		}
	}
}

// received is what the pod's soft UART makes of the node's TX bytes: the text itself on the
// right channel at the right baud, nothing on another channel, framing garbage at another baud.
func (p *uartProxy) received(b []byte) []byte {
	if p.rx != dutTX {
		return nil
	}
	ratio := float64(p.baud) / dutBaud
	if math.Abs(ratio-1) <= 0.03 {
		return b
	}
	n := max(1, int(float64(len(b))*ratio))
	out := make([]byte, n)
	for i := range out {
		v := b[min(len(b)-1, int(float64(i)/ratio))]
		if ratio > 1 {
			out[i] = 0xF0 | v>>4 // sampling too fast: each frame splits into several
		} else {
			out[i] = 0x80 | v<<1 // too slow: frames merge
		}
	}
	return out
}
