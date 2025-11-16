import time
import threading
import board
import neopixel_spi as neopixel


class LED(threading.Thread):
    def __init__(self, num_pixels=72, color=(227, 253, 255), tail_decay=0.75, delay=0.02, brightness=1.0):
        super().__init__()
        self.num_pixels = num_pixels
        self.color = color
        self.tail_decay = tail_decay
        self.delay = delay
        self.leds = [(0, 0, 0)] * self.num_pixels
        self.running = False

        # Setup SPI and NeoPixel strip
        self.pixels = neopixel.NeoPixel_SPI(
            board.SPI(),
            self.num_pixels,
            pixel_order=neopixel.GRB,
            auto_write=False,
            brightness=brightness
        )

    def fade_tail(self):
        for i in range(self.num_pixels):
            r, g, b = self.leds[i]
            self.leds[i] = (
                int(r * self.tail_decay),
                int(g * self.tail_decay),
                int(b * self.tail_decay)
            )

    def draw_frame(self):
        for i, color in enumerate(self.leds):
            self.pixels[i] = color
        self.pixels.show()

    def turn_on(self):
        # self.leds[0] = self.color
        self.leds[11] = self.color
        # self.leds[22] = self.color
        self.leds[34] = self.color
        self.draw_frame()

    def run(self):
        self.running = True

        # # self.leds[0] = self.color
        # self.leds[11] = self.color
        # # self.leds[22] = self.color
        # self.leds[34] = self.color
        # self.draw_frame()

    def show_single_color(self):
        for pos in range(self.num_pixels):
            self.leds[pos] = self.color
        self.draw_frame()

    def halo_loop(self):
        while self.running:
            for pos in range(self.num_pixels):
                if not self.running:
                    break
                self.fade_tail()
                self.leds[pos] = self.color
                self.draw_frame()
                time.sleep(self.delay)

    def stop(self):
        self.running = False
        self.join()
        self.clear()

    def clear(self):
        self.leds = [(0, 0, 0)] * self.num_pixels
        self.draw_frame()


if __name__ == '__main__':
    led = LED()
    led.start()
    led.show_single_color()
    time.sleep(10)
    led.stop()
