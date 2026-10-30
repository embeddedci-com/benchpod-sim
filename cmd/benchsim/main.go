// Command benchsim runs one simulated BenchPod (wired to a simulated process-node DUT) on a
// loopback TCP port, for driving the BenchPod MCP server or SDK without hardware:
//
//	go run ./cmd/benchsim -addr 127.0.0.1:18080 -wiring /tmp/process-node-wiring.json
//	BENCHPOD_TLS=off BENCHPOD_WIRING=/tmp/process-node-wiring.json \
//	    embeddedci-mcp --connection 127.0.0.1:18080 --la-voltage 3.3
package main

import (
	"flag"
	"log"
	"net"
	"os"
	"os/signal"
	"syscall"

	"github.com/embeddedci-com/benchpod-sim/benchsim"
)

func main() {
	addr := flag.String("addr", "127.0.0.1:18080", "listen address (loopback only)")
	wiring := flag.String("wiring", "", "also write the bench's wiring profile (for BENCHPOD_WIRING) to this file")
	la := flag.Int("la-mv", 0, "start with the LA I/O bank selected: 1800 or 3300 (default: unset, like a real pod)")
	powered := flag.Bool("powered", false, "start with the DUT powered and booted")
	flag.Parse()

	host, _, err := net.SplitHostPort(*addr)
	if err != nil {
		log.Fatalf("benchsim: -addr: %v", err)
	}
	if ip := net.ParseIP(host); host != "localhost" && (ip == nil || !ip.IsLoopback()) {
		log.Fatalf("benchsim: -addr %s is not a loopback address; the simulator only binds to loopback", *addr)
	}
	if *wiring != "" {
		if err := os.WriteFile(*wiring, []byte(benchsim.WiringProfileJSON), 0o644); err != nil {
			log.Fatalf("benchsim: write wiring: %v", err)
		}
	}
	var opts []benchsim.Option
	if *la != 0 {
		opts = append(opts, benchsim.WithLaVoltage(*la))
	}
	if *powered {
		opts = append(opts, benchsim.WithDUTPowered())
	}
	ln, err := net.Listen("tcp", *addr)
	if err != nil {
		log.Fatalf("benchsim: %v", err)
	}
	sim := benchsim.New(opts...)
	go func() {
		ch := make(chan os.Signal, 1)
		signal.Notify(ch, os.Interrupt, syscall.SIGTERM)
		<-ch
		_ = sim.Close()
	}()
	log.Printf("benchsim: simulated BenchPod on %s", ln.Addr())
	if err := sim.Serve(ln); err != nil {
		log.Fatalf("benchsim: %v", err)
	}
}
