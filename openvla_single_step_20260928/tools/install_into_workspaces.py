#!/usr/bin/env python3
"""Install additive scripts/package; back up differing files, never start control."""
import argparse
import ast
from datetime import datetime, timezone
from pathlib import Path
import shutil
import uuid

ROOT = Path(__file__).resolve().parents[1]


def install(server_repo=None, agile_ws=None):
    copies = []
    if server_repo:
        repo = Path(server_repo).expanduser().resolve(strict=True)
        script = repo / "vla-scripts/openvla_act.py"
        tree = ast.parse(script.read_text())
        agents = [node for node in tree.body if isinstance(node, ast.ClassDef)
                  and node.name == "OpenVLAActionAgent"]
        if not agents or not any(isinstance(node, ast.FunctionDef) and node.name == "act"
                                 for node in agents[0].body):
            raise ValueError("Existing script does not expose OpenVLAActionAgent.act; inspect before installing")
        for name in ("openvla_agile_server.py", "start_openvla_agile.sh"):
            copies.append((ROOT / "server" / name, repo / "vla-scripts" / name))
    if agile_ws:
        workspace = Path(agile_ws).expanduser().resolve(strict=True)
        if not (workspace / "src").is_dir():
            raise ValueError("agile_ws/src does not exist")
        source = ROOT / "agile_ws/src/agile_openvla_bridge"
        for name in ("start_openvla_bridge.sh", "openvla_step.sh"):
            copies.append((ROOT / "drone" / name, workspace.parent / name))
        for path in sorted(source.rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                copies.append((path, workspace / "src/agile_openvla_bridge" / path.relative_to(source)))
    # Validate both destinations before making any changes.
    for source, destination in copies:
        if destination.is_symlink() or (destination.exists() and not destination.is_file()):
            raise ValueError("Refusing non-regular destination: " + str(destination))
        for parent in destination.parents:
            if parent.is_symlink():
                raise ValueError("Refusing destination through symlink: " + str(parent))
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    for source, destination in copies:
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if destination.read_bytes() == source.read_bytes():
                print("unchanged", destination)
                continue
            backup = destination.with_name(destination.name + ".backup-" + stamp)
            shutil.copy2(destination, backup)
            print("backup", backup)
        shutil.copy2(source, destination)
        if destination.suffix == ".sh" or destination.parent.name == "scripts" or destination.name == "openvla_agile_server.py":
            destination.chmod(0o755)
        print("installed", destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-repo", type=Path)
    parser.add_argument("--agile-ws", type=Path)
    args = parser.parse_args()
    if not args.server_repo and not args.agile_ws:
        parser.error("provide --server-repo and/or --agile-ws")
    install(args.server_repo, args.agile_ws)


if __name__ == "__main__":
    main()
