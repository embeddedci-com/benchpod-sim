#!/usr/bin/env bash
# Build sensor_def.wasm in a benchpod-firmware checkout and vendor it into sensorengine/,
# recording its sha256 and the firmware commit it came from. The package test fails when the
# vendored file does not match the record, so the copy only changes through this script.
#
#   scripts/update-sensor-engine.sh [path/to/benchpod-firmware]   (default ../benchpod-firmware)
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
fw="${1:-$here/../benchpod-firmware}"
fw="$(cd "$fw" && pwd)"
make -C "$fw/stm32h563/wasm" test
dst="$here/sensorengine"
cp "$fw/stm32h563/wasm/build/sensor_def.wasm" "$dst/sensor_def.wasm"
sum="$(shasum -a 256 "$dst/sensor_def.wasm" | awk '{print $1}')"
commit="$(git -C "$fw" rev-parse --short=12 HEAD)$(test -n "$(git -C "$fw" status --porcelain -- stm32h563/src stm32h563/wasm)" && echo -dirty || true)"
printf '%s  sensor_def.wasm  benchpod-firmware %s\n' "$sum" "$commit" > "$dst/sensor_def.wasm.sha256"
echo "vendored sensor_def.wasm $(wc -c < "$dst/sensor_def.wasm" | tr -d ' ') bytes, sha256 $sum, firmware $commit"
