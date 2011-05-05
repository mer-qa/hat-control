#!/bin/sh
echo "Starting data storing"
../src/hat_ctrl --config_file ../sensor_configs/config_audiolat_demo &
echo "Playing test signal"
play -q pulse.wav
sleep 1
python pygraph.py ../src/data.txt &
echo "Test end"

