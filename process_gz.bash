#!/bin/bash

# Caller must set BIN_FILE and LOG_DIR, e.g.,
# export BIN_FILE=~/ardupilot/logs/00000028.BIN
# export LOG_DIR=~/projects/ardusub_surftrak/logs/sitl/surftrak/trapezoid

mkdir -p $LOG_DIR
mavlogdump.py --types CTUN --format csv $BIN_FILE > $LOG_DIR/ctun.csv
graph_gz.py
