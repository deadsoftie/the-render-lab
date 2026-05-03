#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

RELEASE="$SCRIPT_DIR/out/build/linux-release/the-render-lab"
DEBUG="$SCRIPT_DIR/out/build/linux-debug/the-render-lab"

if [[ -x "$RELEASE" ]]; then
    BIN="$RELEASE"
elif [[ -x "$DEBUG" ]]; then
    BIN="$DEBUG"
else
    echo "No binary found. Run: cmake --preset linux-debug && cmake --build --preset linux-debug"
    exit 1
fi

exec env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia "$BIN" "$@"
