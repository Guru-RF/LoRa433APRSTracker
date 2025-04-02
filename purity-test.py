import binascii
import os
import time
from math import atan2, ceil, cos, log, radians, sin, sqrt

import adafruit_gps
import adafruit_rfm9x
import board
import busio
import digitalio
import microcontroller
import rtc
import supervisor
import usb_cdc
from analogio import AnalogIn
from microcontroller import watchdog as w

import config

# stop autoreloading
supervisor.runtime.autoreload = False

amp = digitalio.DigitalInOut(board.GP2)
amp.direction = digitalio.Direction.OUTPUT
amp.value = False

# LoRa APRS frequency
RADIO_FREQ_MHZ = 433.775
CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

# Lora Module
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
rfm9x.tx_power = 23

message = "testsdhkjfklsdhjfklsdjfklsjfklsjfklsfklsdjfklsdjflkjskljskldjflks"
while True:
    amp.value = True
    time.sleep(0.1)
    print("LoRa send message: " + message)
    rfm9x.send(
        w,
        bytes("{}".format("<"), "UTF-8")
        + binascii.unhexlify("FF")
        + binascii.unhexlify("01")
        + bytes("{}".format(message), "UTF-8"),
    )
    amp.value = False
    time.sleep(0.2)

