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
  cd /tmp
  wget https://downloads.circuitpython.org/bin/rfguru_rp2040/en_US/adafruit-circuitpython-rfguru_rp2040-en_US-9.1.4.uf2
  cp adafruit-circuitpython-rfguru_rp2040-en_US-9.1.4.uf2 /Volumes/RPI-RP2
  rm adafruit-circuitpython-rfguru_rp2040-en_US-9.1.4.uf2
  echo "Sleeping 20 seconds for firmware to install"
  cd -
  sleep 20
fi

DIR="/Volumes/CIRCUITPY"
if [ -d "$DIR" ]; then
  echo "Install software in ${DIR}..."
  cp -r lib /Volumes/CIRCUITPY
  cp boot.py /Volumes/CIRCUITPY
  echo "1" > /Volumes/CIRCUITPY/sequence
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
  unlink /Volumes/APRSTRKR/ro
  sync
  cp -r lib /Volumes/APRSTRKR
  cp boot.py /Volumes/APRSTRKR
  cp code.py /Volumes/APRSTRKR
  sync
  diskutil unmount /Volumes/APRSTRKR
  echo "done"
fi
