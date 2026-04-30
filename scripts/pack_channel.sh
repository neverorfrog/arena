#!/bin/bash
# Run from anywhere — uses the arena environment in the root pyproject.toml
set -e

ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
CHANNEL="$ROOT/local-channel/channel"

# Pack both platforms from the root arena environment
pixi-pack "$ROOT/pyproject.toml" --environment arena --platform linux-64 --output-file /tmp/bt-linux64.tar
pixi-pack "$ROOT/pyproject.toml" --environment arena --platform linux-aarch64 --output-file /tmp/bt-aarch64.tar

# Extract linux-64 first (establishes linux-64/ + noarch/)
rm -rf "$ROOT/local-channel"
mkdir -p "$ROOT/local-channel"
tar -xf /tmp/bt-linux64.tar -C "$ROOT/local-channel"

# Save the aarch64 pixi-pack bundle for deploy.sh (before it gets cleared)
cp /tmp/bt-aarch64.tar "$ROOT/local-channel/aarch64-environment.tar"

# Extract linux-aarch64 to temp, then merge carefully
rm -rf /tmp/bt-aarch64-extract && mkdir /tmp/bt-aarch64-extract
tar -xf /tmp/bt-aarch64.tar -C /tmp/bt-aarch64-extract

# Copy linux-aarch64 platform dir (no conflict)
cp -r /tmp/bt-aarch64-extract/channel/linux-aarch64 "$CHANNEL/linux-aarch64"

# Copy new noarch packages (both .conda and .tar.bz2)
cp /tmp/bt-aarch64-extract/channel/noarch/*.conda "$CHANNEL/noarch/" 2>/dev/null || true
cp /tmp/bt-aarch64-extract/channel/noarch/*.tar.bz2 "$CHANNEL/noarch/" 2>/dev/null || true

# Merge noarch repodata.json
CHANNEL="$CHANNEL" python3 - << 'EOF'
import json, os, pathlib
noarch = pathlib.Path(os.environ["CHANNEL"]) / "noarch"
src = pathlib.Path("/tmp/bt-aarch64-extract/channel/noarch/repodata.json")
existing = json.loads((noarch / "repodata.json").read_text())
new = json.loads(src.read_text())
for key in ("packages", "packages.conda"):
    existing.setdefault(key, {}).update(new.get(key, {}))
(noarch / "repodata.json").write_text(json.dumps(existing))
print(f"noarch: {len(existing.get('packages.conda', {}))} packages indexed")
EOF

echo "Channel ready at $CHANNEL"
