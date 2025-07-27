#!/bin/bash
sudo ip link set can0 up type can bitrate 125000

sudo ip link set can0 txqueuelen 10000

sudo ip link set up can0

candump -td can0
