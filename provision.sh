#!/bin/bash

args=("$@")

if [ -z "${args[0]}" ]; then
  echo "Please provide a callsign"
  exit 1
fi

if ! ([ "${args[1]}" = "False" ] || [ "${args[1]}" = "True"  ]); then
  echo "Please provide a True/False for i2c"
  exit 1
fi

DIR="/Volumes/RPI-RP2"
if [ -d "$DIR" ]; then
  echo "Installing firmwire to pico in ${DIR}..."
  cd /tmp
  wget https://downloads.circuitpython.org/bin/raspberry_pi_pico/en_US/adafruit-circuitpython-raspberry_pi_pico-en_US-8.2.10.uf2
  cp adafruit-circuitpython-raspberry_pi_pico-en_US-8.2.10.uf2 /Volumes/RPI-RP2
  rm adafruit-circuitpython-raspberry_pi_pico-en_US-8.2.10.uf2
  echo "Sleeping 20 seconds for firmware to install"
  cd -
  sleep 20
fi

DIR="/Volumes/CIRCUITPY"
if [ -d "$DIR" ]; then
  echo "Install software in ${DIR}..."
  cp -r src/lib /Volumes/CIRCUITPY
  cp src/boot.py /Volumes/CIRCUITPY
  echo "1" > /Volumes/CIRCUITPY/sequence
  cp src/config.py /tmp
  perl -i -pe "s/--CALL--/${args[0]}/g" /tmp/config.py
  perl -i -pe "s/--BOOL--/${args[1]}/g" /tmp/config.py
  cp -f /tmp/config.py /Volumes/CIRCUITPY
  cp src/code.py /Volumes/CIRCUITPY
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
  cp -r src/lib /Volumes/APRSTRKR
  cp src/boot.py /Volumes/APRSTRKR
  cp src/code.py /Volumes/APRSTRKR
  sync
  diskutil unmount /Volumes/APRSTRKR
  echo "done"
fi
