#!/usr/bin/env bash
# Build static WebGL2 assets in dist/. Pass the deployment path, e.g. /drive/.
set -euo pipefail

if (( $# > 1 )); then
  echo "Usage: $0 [public-url]" >&2
  exit 2
fi

cd "$(dirname "${BASH_SOURCE[0]}")/.."
command -v trunk >/dev/null || {
  echo "Install Trunk with: cargo install trunk --locked" >&2
  exit 1
}
if ! rustup target list --installed | grep -qx wasm32-unknown-unknown; then
  echo "Install the wasm target with: rustup target add wasm32-unknown-unknown" >&2
  exit 1
fi

# Trunk expects a boolean rather than the commonly inherited NO_COLOR=1.
NO_COLOR=true trunk build --release --locked --no-default-features --features web \
  --public-url "${1:-./}" --dist dist
touch dist/.nojekyll
