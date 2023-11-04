#!/bin/bash

v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0 -c  white_balance_temperature=1000
