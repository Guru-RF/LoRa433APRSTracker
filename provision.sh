#!/bin/bash

args=("$@")

if [ -z "${args[0]}" ]; then
  echo "Please provide a tracker callsign"
  exit 1
fi

if [ -z "${args[1]}" ]; then
  echo "Please provide an error messages callsign"
  exit 1
fi

DIR="/Volumes/RPI-RP2"
if [ -d "$DIR" ]; then
  echo "Installing firmwire to pico in ${DIR}..."
  cp adafruit-circuitpython-rfguru_rp2040-en_US-9.2.9.uf2 /Volumes/RPI-RP2
  echo "Sleeping 20 seconds for firmware to install"
  sleep 20
fi

DIR="/Volumes/CIRCUITPY"
if [ -d "$DIR" ]; then
  echo "Install software in ${DIR}..."
  cp -r lib /Volumes/CIRCUITPY
  cp boot.py /Volumes/CIRCUITPY
  cp config.py /tmp
  perl -i -pe "s/--CALL--/${args[0]}/g" /tmp/config.py
  perl -i -pe "s/--MSGS--/${args[1]}/g" /tmp/config.py
  cp -f /tmp/config.py /Volumes/CIRCUITPY
  cp code.py /Volumes/CIRCUITPY
  rm -f /tmp/config.py
  sync
  diskutil unmount /Volumes/CIRCUITPY
  echo "done"
fi

DIR="/Volumes/APRSTRKR"
if [ -d "$DIR" ]; then
  echo "Updating software in ${DIR}..."
  sync
  cp -r lib /Volumes/APRSTRKR
  cp boot.py /Volumes/APRSTRKR
  cp code.py /Volumes/APRSTRKR
  if [ ! -f "$DIR/config.py" ]; then
    cp config.py /tmp
    perl -i -pe "s/--CALL--/${args[0]}/g" /tmp/config.py
    perl -i -pe "s/--MSGS--/${args[1]}/g" /tmp/config.py
    cp -f /tmp/config.py /Volumes/APRSTRKR
    rm -f /tmp/config.py
  fi
  sync
  diskutil unmount /Volumes/APRSTRKR
  echo "done"
fi
