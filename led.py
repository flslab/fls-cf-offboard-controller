#!/usr/bin/env python3
# NeoPixel library strandtest example
# Author: Tony DiCola (tony@tonydicola.com)
#
# Direct port of the Arduino NeoPixel library strandtest example.  Showcases
# various animations on a strip of NeoPixels.

import time
from rpi_ws281x import *
import json
import argparse

# LED strip configuration:
LED_COUNT      = 60     # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating a signal (try 10)
LED_BRIGHTNESS = 65      # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53


class LED:
    def __init__(self):
        self.strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        self.strip.begin()

    def set_frame(self, brightness_map):
        for idx, brightness in brightness_map.items():
            b = brightness_map[idx] / 10
            self.strip.setPixelColor(int(idx), Color(int(b*227), int(b*253), int(b*255)))
        self.strip.show()


# Define functions which animate LEDs in various ways.
    def colorWipe(self, color, wait_ms=50):
        """Wipe color across display a pixel at a time."""
        for i in range(self.strip.numPixels()):
            self.strip.setPixelColor(i, color)
            self.strip.show()
            time.sleep(wait_ms/1000.0)

    def clear(self):
        self.colorWipe(Color(0, 0, 0), 10)

if __name__ == '__main__':
    led = LED()

    # led.colorWipe(Color(227,253,255))

    with open("animation_data.json", "r") as f:
        animation_data = json.load(f)

    for i in range(5):
        for frame, data in animation_data.items():
            led.set_frame(data['led'])
            time.sleep(1/24)

    led.clear()