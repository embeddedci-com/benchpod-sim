package benchsim

import (
	"bufio"
	"encoding/json"
	"net"
	"strings"
	"testing"
	"time"
)

// After a pod reset the DUT is off, and powering it boots it again: the banner reaches a UART proxy.
func TestFirmwareRebootThenDUTBootsAgain(t *testing.T) {
	s := New(WithFirmware("3.9.4", Hooks{}))
	defer s.Close()
	for round := 0; round < 2; round++ {
		exec := func(cmd map[string]any) {
			t.Helper()
			if _, msg := s.Exec(cmd, true); msg != "" {
				t.Fatalf("%v: %s", cmd, msg)
			}
		}
		exec(map[string]any{"cmd": "la_voltage", "mv": float64(3300)})
		here, there := net.Pipe()
		go s.ServeTunnel(there, 3)
		line, _ := json.Marshal(map[string]any{"cmd": "uart_proxy_start", "rx": 3, "tx": 4, "baud": 115200})
		go here.Write(append(line, '\n'))
		br := bufio.NewReader(here)
		if _, err := br.ReadString('\n'); err != nil {
			t.Fatal(err)
		}
		exec(map[string]any{"cmd": "target_power", "efuse": float64(1), "state": float64(1)})
		seen := ""
		_ = here.SetReadDeadline(time.Now().Add(8 * time.Second))
		for !strings.Contains(seen, "APP_OK") {
			buf := make([]byte, 512)
			n, err := br.Read(buf)
			seen += string(buf[:n])
			if err != nil {
				t.Fatalf("round %d: no banner: %v; got %q", round, err, seen)
			}
		}
		here.Close()
		s.Reboot()
	}
}
