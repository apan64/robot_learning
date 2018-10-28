#!/bin/bash
a=$1 
sshpass -p "raspberry" ssh pi@192.168.17.201 "~pi/use_case_scripts/ride-of-the-neatos/audio_stream.sh -q" > recording_${a}.wav

