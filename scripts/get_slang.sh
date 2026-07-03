#!/usr/bin/env bash
#
# get_slang.sh — download the Slang shader compiler toolchain into tools/slang/.
#
# Slang (https://github.com/shader-slang/slang) compiles our single-source .slang shaders to
# both SPIR-V (Vulkan) and Metal (MSL/metallib). The toolchain is a prebuilt release binary,
# vendored under external/ but not committed (external/slang is gitignored). CMake locates
# external/slang/bin/slangc.
#
# Usage: scripts/get_slang.sh [version]
set -euo pipefail

VERSION="${1:-2026.12.2}"

case "$(uname -s)-$(uname -m)" in
    Darwin-arm64)  PLATFORM="macos-aarch64" ;;
    Darwin-x86_64) PLATFORM="macos-x86_64" ;;
    Linux-x86_64)  PLATFORM="linux-x86_64" ;;
    Linux-aarch64) PLATFORM="linux-aarch64" ;;
    *) echo "unsupported platform: $(uname -s)-$(uname -m)" >&2; exit 1 ;;
esac

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/external/slang"
ARCHIVE="slang-${VERSION}-${PLATFORM}.tar.gz"
URL="https://github.com/shader-slang/slang/releases/download/v${VERSION}/${ARCHIVE}"

echo "Fetching Slang ${VERSION} for ${PLATFORM}..."
mkdir -p "$DEST"
tmp="$(mktemp -d)"
curl -fsSL --max-time 180 "$URL" -o "$tmp/$ARCHIVE"
tar -xzf "$tmp/$ARCHIVE" -C "$DEST"
rm -rf "$tmp"

"$DEST/bin/slangc" -v
echo "slangc installed at $DEST/bin/slangc"
