#!/usr/bin/env bash
# Read-only host/image inventory. Uses an explicitly named LOCAL official image.
set -euo pipefail
uname -m
if ! command -v docker >/dev/null; then echo 'Docker unavailable on this server'; exit 2; fi
docker version --format '{{json .Server.Version}}'
docker image ls --digests --format '{{.Repository}}:{{.Tag}} {{.ID}} {{.Digest}}'
if [[ $# -eq 0 ]]; then
  echo 'Pass one existing official S-series image name to inspect its packages.'
  exit 0
fi
image="$1"
docker image inspect "$image" --format '{{json .RepoDigests}} {{.Id}} {{.Architecture}}'
docker run --rm --pull=never --network=none --entrypoint python3 "$image" -c '
import importlib.metadata as m, platform
print("python",platform.python_version())
for d in m.distributions():
 n=d.metadata.get("Name","")
 if any(s in n.lower() for s in ("hbdk","horizon","hbrt","hat","onnx","torch")):
  print(n,d.version)
'
