#!/bin/bash
set -e
cd /home/nvidia/agile
source ./vla_env.sh
exec python3 ./vla_stack.py status
