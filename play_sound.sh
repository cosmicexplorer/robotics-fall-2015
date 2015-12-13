#!/bin/sh

# aplay must be installed to use this, and it's not available for windows, or
# mac i think

# RUN THIS COMMAND BEFORE RUNNING THIS FILE
# ./update_subs.sh

PYTHONPATH=./wavebender python sound/get_notes_and_play.py | aplay
