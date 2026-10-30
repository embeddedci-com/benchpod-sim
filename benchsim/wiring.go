package benchsim

// WiringProfileJSON is the bench's wiring profile (schema version 1) for the process node, as
// its README wires it. The SDK resolves the wiring on the host, not on the pod: point the MCP
// server at this file with BENCHPOD_WIRING=<path> so the `wiring` tool, and every tool whose
// channel, baud or rail it fills in, matches the simulated bench.
const WiringProfileJSON = `{
  "version": 1,
  "la_mv": 3300,
  "efuse": 1,
  "uart_rx": 3,
  "uart_tx": 4,
  "uart_baud": 115200,
  "i2c_sda": 2,
  "i2c_scl": 1,
  "i2c_addr": "0x76",
  "swd_swclk": 11,
  "swd_swdio": 12,
  "swd_nreset": true,
  "swd_target": "target/stm32f4x.cfg",
  "signals": [
    {"name": "ALARM", "la": 5, "direction": "input", "active_low": false,
     "description": "PB0 (Nucleo A3): high while any alarm is active"},
    {"name": "STROBE", "la": 6, "direction": "input", "active_low": false,
     "description": "PA8 (Nucleo D7): marks the task chosen with the strobe console command"}
  ]
}
`
