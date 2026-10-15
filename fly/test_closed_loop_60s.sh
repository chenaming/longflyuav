#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
echo '原60秒入口已按用户要求改为30秒；使用相同拆桨保护。'
exec bash "$BASE/test_closed_loop_30s.sh" "$@"
