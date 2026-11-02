#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
target=${1:-"$root/src/nav2_overlays/navigation2_humble"}
revision=3c3db59d6969d8ecee8e68468693d006397f4a0c
if [[ -e "$target" ]]; then
  echo "Destination already exists: $target" >&2
  exit 2
fi
git init "$target"
git -C "$target" remote add origin https://github.com/ros-navigation/navigation2.git
git -C "$target" fetch --depth=1 origin "$revision"
git -C "$target" checkout --detach FETCH_HEAD
git -C "$target" apply --check "$root/patches/navigation2_humble.patch"
git -C "$target" apply "$root/patches/navigation2_humble.patch"
python3 - "$root/docs/jetson_source_inventory.json" "$target" <<'PY'
import hashlib,json,pathlib,sys
manifest=json.loads(pathlib.Path(sys.argv[1]).read_text())
target=pathlib.Path(sys.argv[2]);files=manifest['roots']['navigation2_humble']['files']
bad=[]
for relative,info in files.items():
    path=target/relative
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=info['sha256']:
        bad.append(relative)
if bad:
    raise SystemExit('Source hash mismatch: '+', '.join(bad))
print(f'PASS: {len(files)} upstream/patched files match the Jetson snapshot')
PY
echo 'Source restored. No build, services or robot commands were started.'
