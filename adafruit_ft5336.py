# SPDX-FileCopyrightText: Copyright (c) 2023 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ft5336`
================================================================================

Touchscreen driver for the FT5336 touch controller


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies
  based on the library's use of either.

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

from adafruit_register.i2c_bits import ROBits
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const
try:
    from typing import List
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_FT5336.git"

_DEFAULT_ADDR = const(0x38)
_REG_VENDID = const(0xA3)
_REG_CHIPID = const(0xA8)
_VENDID = const(0x11)
_CHIPID = const(0x79)
_REG_NUMTOUCHES = const(0x02)
_TD_STATUS = const(0x02)
_TOUCH1_XH = const(0x03)
_TOUCH1_XL = const(0x04)
_TOUCH1_YH = const(0x05)
_TOUCH1_YL = const(0x06)

class Adafruit_FT5336:
    # Define read-only register bits for vendor ID, chip ID, and number of touches.
    _vend_id = ROBits(8, _REG_VENDID, 0)  # 8-bit read-only register for vendor ID
    _chip_id = ROBits(8, _REG_CHIPID, 0)  # 8-bit read-only register for chip ID
    _num_touches = ROBits(8, _REG_NUMTOUCHES, 0)  # 8-bit read-only register for number of touches

    def __init__(self, i2c, i2c_addr: int = _DEFAULT_ADDR, max_touches: int = 5):
        """
        Initializes the FT5336 touchscreen driver.

        Args:
            i2c: The I2C bus object.
            i2c_addr (int): The I2C address of the device. Defaults to _DEFAULT_ADDR.
            max_touches (int): Maximum number of touch points to track. Defaults to 5.
        
        Raises:
            ValueError: If the detected vendor ID or chip ID does not match the expected values.
        """
        self.i2c_device = I2CDevice(i2c, i2c_addr)  # I2C device instance
        self.i2c_addr = i2c_addr  # Store the I2C address
        self._touches = 0  # Number of current touches
        self.max_touches = max_touches  # Maximum number of touches to track

        # Initialize touch point arrays
        self._touchX: List[int] = [0] * self.max_touches
        self._touchY: List[int] = [0] * self.max_touches
        self._touchID: List[int] = [0] * self.max_touches

        # Verify device identity by checking the vendor and chip IDs
        if self._vend_id != _VENDID:
            raise ValueError("Incorrect vendor ID")
        if self._chip_id != _CHIPID:
            raise ValueError("Incorrect chip ID")

    @property
    def touched(self):
        n = self._num_touches
        return 0 if n > 5 else n
    
    def _read_data(self):
        buffer = bytearray(32)
        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytearray([0]), buffer, in_end=32)

        self._touches = buffer[_TD_STATUS]
        if self._touches > 5 or self._touches == 0:
            self._touches = 0

        for i in range(self._touches):
            self._touchX[i] = (buffer[_TOUCH1_XH + i * 6] & 0x0F) << 8 | buffer[_TOUCH1_XL + i * 6]
            self._touchY[i] = (buffer[_TOUCH1_YH + i * 6] & 0x0F) << 8 | buffer[_TOUCH1_YL + i * 6]
            self._touchID[i] = buffer[_TOUCH1_YH + i * 6] >> 4
    
    @property
    def points(self):
        self._read_data()

        points = []
        for i in range(min(self._touches, self.max_touches)):
            point = (self._touchX[i], self._touchY[i], 1)  # 1 indicates touch is active
            points.append(point)

        return points

    def point(self, point_index: int):
        self._read_data()
        if self._touches == 0 or point_index >= self._touches:
            return (0, 0, 0)
        else:
            return (self._touchX[point_index], self._touchY[point_index], 1)
