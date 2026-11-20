# BenchPod Sim

A simulated [EmbeddedCI BenchPod](https://www.embeddedci.com/benchpod) wired to a simulated target
board, so software that drives a pod can be developed and tested with no hardware.

- **`benchsim`** answers the pod's LAN line protocol: power, UART console, logic and ADC captures,
  the DAC and control loop, emulated sensors from definitions, CAN and SPI flash. Its board is a
  simulated NUCLEO-F446RE process node, which reads a BMP280 on I2C.
- **`sensorengine`** runs the pod's sensor definition engine (the firmware's C, built as
  WebAssembly) with [wazero](https://wazero.io/).
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

## Firmware 4.0 protocol

The simulator answers as BenchPod firmware 4.0.0 does: client API 2. Clients send
`"client_api":2` on every command; a command without it (or with a lower level) is refused with
`BP0030`, except `status`, `diag`, `ping`, `lease_release`, the update commands, `blob_status` and
`identity_public`. A refusal a client acts on carries a code and its args, no message:

```bash
printf '{"cmd":"la_pins"}\n' | nc 127.0.0.1 18080
# {"args":{"api":2,"client_api":0,"min_client_api":2},"code":"BP0030","message":"Update the BenchPod SDK, ...","status":"error"}
printf '{"cmd":"frobnicate","client_api":2}\n' | nc 127.0.0.1 18080
# {"args":{"cmd":"frobnicate"},"code":"BP0002","status":"error"}
```

`status` reports `api`, `min_client_api`, `features_rev`, `fpga_image` and `hw` instead of a
`caps` list. Some requests with the 4.0 names:

```json
{"cmd":"target_power","efuse":1,"enabled":true,"client_api":2}
{"cmd":"la_pull","la":1,"pull":"up","client_api":2}
{"cmd":"generate","waveform":"sine","freq_hz":1000,"path":"3v3","amplitude_v":1,"offset_v":1.65,"client_api":2}
{"cmd":"dac_out","path":"3v3","v":1.1,"client_api":2}
{"cmd":"la_capture","samples":20000,"sample_rate_hz":2000000,"client_api":2}
{"cmd":"capture_dual","adc_samples":4096,"adc_sample_rate_hz":400000,"la_samples":4096,"la_sample_rate_hz":1000000,"client_api":2}
{"cmd":"spi_start","sck":10,"mosi":11,"miso":12,"cs":13,"clock_hz":1000000,"client_api":2}
{"cmd":"power_profile","efuse":1,"sample_rate_hz":500,"duration_ms":1000,"client_api":2}
```

ADC samples come as `"b64"` unless the request says `"enc":"dec"`. SCPI is off, as on a new pod;
`lan_tls` `"scpi":true` from the cloud link (`podsim`) turns it on. The simulator serves plaintext
only: it reports `https_only` as the firmware does but keeps answering JSON on its plaintext port.

## Emulated sensors

`sensor_define`, `sensor_start`, `sensor_set`, `sensor_stop`, `sensor_status`, `sensor_regs`,
`sensor_la`, `sensor_fault`, `sensor_fault_clear` and the `sensor_stimulus` commands take the
definitions of the [sensor library](https://github.com/embeddedci-com/benchpod-sensors), or your
own, and answer as firmware 4.0.0 does. The register images, UART messages and analog values
come from the same engine the pod compiles definitions with, so the node reads what it would read
from a real pod. Try `bmp280` on LA2 (SDA) and LA1 (SCL) and power-cycle the node.

`sensorengine/sensor_def.wasm` is a vendored build of benchpod-firmware `stm32h563/wasm`; its
sha256 and the firmware commit are in `sensor_def.wasm.sha256`. Refresh it with
`scripts/update-sensor-engine.sh ../benchpod-firmware` (needs Homebrew LLVM and LLD, and Node
for the engine's own tests).

The simulator models the I2C target (8- and 16-bit register addresses, word registers, commands,
the conversion busy flag), UART messages on the sensor's TX pin and analog sensors on the DAC
output. It does not model the sensor FIFO, data-ready pins, `on_write` rules beyond storing the
byte, UART requests from the DUT, the SPI bus on the wire or the 4-20 mA output.

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
