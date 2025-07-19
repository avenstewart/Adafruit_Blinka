# SPDX-FileCopyrightText: 2025 Aven Stewart for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""RK3588S NeoPixel Driver Class (Experimental SPI-based)

This module allows Adafruit_Blinka to drive WS2812/NeoPixel RGB LEDs using
hardware SPI on Rockchip RK3588S-based boards (e.g., Orange Pi 5 Pro).
It leverages SPI bit patterns to meet strict WS2812 timing, offloading
precise signaling to the SPI hardware rather than relying on CPU-driven GPIO.

This driver uses hardware SPI to generate the precise 800kHz WS2812 waveform.
Each WS2812 bit (1.25µs) is encoded as 3 SPI bits at 2.4MHz:
  - Logic 0 -> 100 (0.416µs high, 0.832µs low)
  - Logic 1 -> 110 (0.832µs high, 0.416µs low)
This technique avoids CPU timing jitter by letting the SPI peripheral handle the waveform.

- The RK3588S GPIO logic is 3.3V. Many WS2812 LEDs will accept 3.3V data, but
  a level shifter is strongly recommended for reliability.
"""

import time
import atexit
import spidev
from adafruit_platformdetect import Detector

try:
    from typing import Optional
    from digitalio import DigitalInOut
except ImportError:
    pass

# Default SPI configuration (overridden by detection unless user sets manually)
SPI_BUS = None  # Will auto-detect if left None
SPI_DEV = 0
SPI_HZ = 2400000  # Chosen for WS2812 bit encoding
RESET_US = 60  # >50µs latch/reset between frames

_spi: Optional[spidev.SpiDev] = None
_buf: Optional[bytearray] = None

# Board detection for automatic SPI bus selection
if SPI_BUS is None:
    detector = Detector()
    board_name = detector.board.id
    if board_name in ("ORANGE_PI_5_PRO", "ORANGE_PI_5", "ORANGE_PI_5B"):
        SPI_BUS = 4  # SPI4 MOSI (Pin 38)
    elif board_name == "ORANGE_PI_5_PLUS":
        SPI_BUS = 0  # SPI0 MOSI (Pin 19)
    else:
        raise NotImplementedError("Only RK3588/s OPi boards are supported.")

# Build lookup table to encode each byte (8 bits) into 24 SPI bits (8 WS2812 bits)
_BIT_PATTERNS = {0: 0b100, 1: 0b110}
_LOOKUP = []
for byte in range(256):
    encoded_bits = []
    for i in range(8)[::-1]:
        bit = (byte >> i) & 1
        encoded_bits.extend([
            (_BIT_PATTERNS[bit] >> 2) & 1,
            (_BIT_PATTERNS[bit] >> 1) & 1,
            (_BIT_PATTERNS[bit] >> 0) & 1,
        ])
    packed = bytearray()
    for i in range(0, len(encoded_bits), 8):
        val = 0
        for b in encoded_bits[i:i+8]:
            val = (val << 1) | b
        packed.append(val)
    _LOOKUP.append(bytes(packed))


def neopixel_write(gpio: Optional[DigitalInOut] = None,
        buf: bytearray = bytearray(),
        spi_bus: int = None,
        spi_dev: int = None) -> None:
    """Send RGB pixel data to a NeoPixel strip over SPI.
    DigitalInOut is optional (for API compatibility); allows user to override SPI bus/device."""
    global _spi, _buf
    bus = spi_bus if spi_bus is not None else SPI_BUS
    dev = spi_dev if spi_dev is not None else SPI_DEV

    if len(buf) % 3 != 0:
        raise RuntimeError("RGBW (4-byte) pixels not yet supported - buffer length must be divisible by 3")

    if _spi is None:
        _spi = spidev.SpiDev()
        _spi.open(bus, dev)
        _spi.max_speed_hz = SPI_HZ
        _spi.mode = 0  # CPOL=0, CPHA=0 per WS2812 timing
        atexit.register(neopixel_cleanup)

    if buf is not _buf:
        _buf = buf

    encoded = bytearray()
    for _byte in buf:
        encoded.extend(_LOOKUP[_byte])

    _spi.writebytes(encoded)
    time.sleep(RESET_US / 1_000_000)  # Ensure latch/reset period


def neopixel_cleanup():
    global _spi
    if _spi is not None:
        _spi.close()
        _spi = None

# Example usage with wiring notes:
# Wiring:
#   - Connect NeoPixel DIN to MOSI: Pin 38 (SPI4) by default, or Pin 19 (SPI0) if overridden.
#   - Connect NeoPixel 5V rail.
#   - Use a level shifter for MOSI (3.3V → 5V) for reliability.
