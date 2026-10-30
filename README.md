# BenchPod Sim

A simulated [EmbeddedCI BenchPod](https://www.embeddedci.com/benchpod) wired to a simulated target
board, so software that drives a pod can be developed and tested with no hardware.

- **`benchsim`** answers the pod's LAN line protocol: power, UART console, logic and ADC captures,
  the DAC and control loop, I2C sensor emulation, CAN, SPI flash and GPS. Its board is a simulated
  NUCLEO-F446RE process node.
- **`podsim`** connects to an EmbeddedCI server over the pod's cloud link, with `benchsim` behind it.

## Try it

```bash
go run ./cmd/benchsim -addr 127.0.0.1:18080 -wiring /tmp/wiring.json
```

Point the [BenchPod MCP server](https://pypi.org/project/embeddedci-mcp/) or the
[Python SDK](https://pypi.org/project/embeddedci/) at it:

```bash
BENCHPOD_TLS=off BENCHPOD_WIRING=/tmp/wiring.json embeddedci-mcp --connection 127.0.0.1:18080 --la-voltage 3.3
```

## What it is not

It is a model of the firmware, not proof of it. Analog is idealized, firmware signatures are not
checked, and some commands answer that they are not modeled. Test on a real pod before you rely on
timing or electrical behavior.

## Tests

```bash
go test ./...
```

`TestMCPEndToEnd` also runs when `BENCHPOD_MCP_COMMAND` points at `embeddedci-mcp`.

## License

Apache License 2.0. See [LICENSE](LICENSE).
