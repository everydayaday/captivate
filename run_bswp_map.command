#!/bin/zsh
# BSWP 정전용량 지도 실행기. iCloud(~/Desktop) 밖의 venv(~/.venvs/galvanic)로 돌린다 (dds_signal_gen/run_receiver_gui.command 와 같은 이유).
# Finder 에서 더블클릭하거나 터미널에서 ./run_bswp_map.command
cd "$(dirname "$0")"
exec ~/.venvs/galvanic/bin/python bswp_map.py "$@"
