#!/usr/bin/env bash
# Installed beside openvla_act.py. Uses the user's existing conda environment.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$script_dir/.." && pwd)"
if [[ ! -f "$script_dir/openvla_act.py" ]]; then
  echo "Install this script beside vla-scripts/openvla_act.py first." >&2
  exit 1
fi
cd -- "$repo_dir"
if [[ "${CONDA_DEFAULT_ENV:-}" == "openvla" ]]; then
  exec python "$script_dir/openvla_agile_server.py" "$@"
fi
conda_executable="${CONDA_EXE:-}"
if [[ -z "$conda_executable" ]]; then
  conda_executable="$(command -v conda || true)"
fi
if [[ -z "$conda_executable" ]]; then
  for candidate in "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda"; do
    if [[ -x "$candidate" ]]; then conda_executable="$candidate"; break; fi
  done
fi
if [[ -z "$conda_executable" ]]; then
  echo "Run 'conda activate openvla' in your configured shell first." >&2
  exit 1
fi
exec "$conda_executable" run --no-capture-output -n openvla \
  python "$script_dir/openvla_agile_server.py" "$@"
