"""Library for Waveshare 1.44inch LCD HAT"""
from dataclasses import dataclass
from enum import Enum
from PIL import Image
from typing import List

import numpy as np
import RPi.GPIO as GPIO
import spidev
import time


class ScanType(Enum):
    """Enum for the scan direction of the LCD"""
    L2R_U2D = 1
    L2R_D2U = 2
    R2L_U2D = 3
    R2L_D2U = 4
    U2D_L2R = 5
    U2D_R2L = 6
    D2U_L2R = 7
    D2U_R2L = 8


@dataclass(frozen=True)
class Config:
    """Basic configuration settings for the LCD"""
    width: int
    height: int
    x: int
    y: int
    rgb: bool
    scan_dir: ScanType


@dataclass(frozen=True)
class PinConfig:
    """Hardware interface configuration for the LCD"""
    rst: int
    dc: int
    cs: int
    bl: int
    spi: spidev.SpiDev


LCD_1IN44_Config = Config(128, 128, 2, 1, True, ScanType.U2D_R2L)
LCD_1IN8_Config = Config(160, 128, 1, 2, False, ScanType.U2D_R2L)

# SPI device, bus = 0, device = 0
__spi = spidev.SpiDev(0, 0)

LCD_1IN44_Pins = PinConfig(27, 25, 8, 24, __spi)
LCD_1IN8_Pins = PinConfig(27, 25, 8, 24, __spi)


class LCD:
    """Class for using a Waveshare LCD HAT for Raspberry Pi Zero"""
    def __init__(self, config: Config, pins: PinConfig):
        self.__config = config
        self.__pins = pins
        self.__width = config.width
        self.__height = config.height
        self.__x_adjust = config.x
        self.__y_adjust = config.y

        self.__init_lcd()

    def __init_lcd(self) -> None:
        self.__init_gpio()

        self.__reset()
        self.__init_reg()

        self.__set_memory_data_access_control(self.__config.scan_dir)

        self.sleep_in(False)

        # turn on the display
        self.__write_reg(0x29)

        self.backlight(True)

    def __init_gpio(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.__pins.rst, GPIO.OUT)
        GPIO.setup(self.__pins.dc, GPIO.OUT)
        GPIO.setup(self.__pins.cs, GPIO.OUT)
        GPIO.setup(self.__pins.bl, GPIO.OUT)

        self.__pins.spi.max_speed_hz = 9000000
        self.__pins.spi.mode = 0b00

    def __init_reg(self) -> None:
        # ST7735R frame rate
        self.__write_reg(0xB1)
        self.__write_data([0x01, 0x2C, 0x2D])

        self.__write_reg(0xB2)
        self.__write_data([0x01, 0x2C, 0x2D])

        self.__write_reg(0xB3)
        self.__write_data([0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D])

        # ST7735R display inversion control
        self.__write_reg(0xB4)
        self.__write_data([0x07])

        # ST7735R power sequence
        self.__write_reg(0xC0)
        self.__write_data([0xA2, 0x02, 0x84])

        self.__write_reg(0xC1)
        self.__write_data([0xC5])

        self.__write_reg(0xC2)
        self.__write_data([0x0A, 0x00])

        self.__write_reg(0xC3)
        self.__write_data([0x8A, 0x2A])

        self.__write_reg(0xC4)
        self.__write_data([0x8A, 0xEE])

        self.__write_reg(0xC5)  # VCOM
        self.__write_data([0x0E])

        # ST7735R gamma adjustment +
        self.__write_reg(0xE0)
        self.__write_data(
            [
                0x0F,
                0x1A,
                0x0F,
                0x18,
                0x2F,
                0x28,
                0x20,
                0x22,
                0x1F,
                0x1B,
                0x23,
                0x37,
                0x00,
                0x07,
                0x02,
                0x10,
            ]
        )

        # ST7735R gamma adjustment -
        self.__write_reg(0xE1)
        self.__write_data(
            [
                0x0F,
                0x1B,
                0x0F,
                0x17,
                0x33,
                0x2C,
                0x29,
                0x2E,
                0x30,
                0x30,
                0x39,
                0x3F,
                0x00,
                0x07,
                0x03,
                0x10,
            ]
        )

        # 65k color mode
        self.__write_reg(0x3A)
        self.__write_data([0x05])

    def __reset(self) -> None:
        """Perform hardware reset"""
        GPIO.output(self.__pins.rst, GPIO.HIGH)
        self.__delay_ms(100)
        GPIO.output(self.__pins.rst, GPIO.LOW)
        self.__delay_ms(100)
        GPIO.output(self.__pins.rst, GPIO.HIGH)
        self.__delay_ms(100)

    def __write_reg(self, reg: int) -> None:
        """Set write register"""
        GPIO.output(self.__pins.dc, GPIO.LOW)
        self.__pins.spi.writebytes([reg])

    def __write_data(self, data: List[int]) -> None:
        """Write data to register"""
        GPIO.output(self.__pins.dc, GPIO.HIGH)
        self.__pins.spi.writebytes(data)

    def __delay_ms(self, ms: int) -> None:
        time.sleep(ms / 1000)

    def __set_memory_data_access_control(self, scan_dir: ScanType) -> None:
        """Set the displays scan and color transfer modes"""
        reg_data_map = {
            ScanType.L2R_U2D: 0x00,
            ScanType.L2R_D2U: 0x80,
            ScanType.R2L_U2D: 0x40,
            ScanType.R2L_D2U: 0x40 | 0x80,
            ScanType.U2D_L2R: 0x20,
            ScanType.U2D_R2L: 0x20 | 0x40,
            ScanType.D2U_L2R: 0x20 | 0x80,
            ScanType.D2U_R2L: 0x20 | 0x40 | 0x80,
        }

        if scan_dir in [
            ScanType.L2R_U2D,
            ScanType.L2R_D2U,
            ScanType.R2L_U2D,
            ScanType.R2L_D2U,
        ]:
            self.__width = self.__config.height
            self.__height = self.__config.width
        else:
            self.__width = self.__config.width
            self.__height = self.__config.height

        memory_access_reg_data = reg_data_map[scan_dir]

        if (memory_access_reg_data & 0x10) != 1:
            self.__x_adjust = self.__config.y
            self.__y_adjust = self.__config.x
        else:
            self.__x_adjust = self.__config.x
            self.__y_adjust = self.__config.y

        self.__write_reg(0x36)  # MX, MY, RGB mode
        if self.__config.rgb:
            self.__write_data([memory_access_reg_data | 0x08])
        else:
            self.__write_data([memory_access_reg_data & 0xF7])

    def __set_window(self, start_x: int, start_y: int, end_x: int, end_y: int) -> None:
        """Set the start position and size of the display area"""
        # set x coordinates
        self.__write_reg(0x2A)
        self.__write_data(
            [
                0x00,
                (start_x & 0xFF) + self.__x_adjust,
                0x00,
                ((end_x - 1) & 0xFF) + self.__x_adjust,
            ]
        )

        # set y coordinates
        self.__write_reg(0x2B)
        self.__write_data(
            [
                0x00,
                (start_y & 0xFF) + self.__y_adjust,
                0x00,
                ((end_y - 1) & 0xFF) + self.__y_adjust,
            ]
        )

        self.__write_reg(0x2C)

    @property
    def width(self) -> int:
        return self.__width
    
    @property
    def height(self) -> int:
        return self.__height

    def sleep_in(self, enable) -> None:
        """Enter or exit low power consumption mode, sleep in"""
        if enable:
            self.__write_reg(0x10)
        else:
            self.__write_reg(0x11)
            self.__delay_ms(120)

    def clear(self) -> None:
        """Clear the screen by setting it to all white"""
        buffer = [0xFF] * (self.__width * self.__height * 2)
        self.__set_window(0, 0, self.__width, self.__height)
        GPIO.output(self.__pins.dc, GPIO.HIGH)
        for i in range(0, len(buffer), 4096):
            self.__write_data(buffer[i : i + 4096])

    def backlight(self, on: bool) -> None:
        """Turn the backlight on or off"""
        if on:
            GPIO.output(self.__pins.bl, GPIO.HIGH)
        else:
            GPIO.output(self.__pins.bl, GPIO.LOW)

    def display_image(self, image: Image.Image) -> None:
        """Display an image on the LCD"""
        if image is None:
            raise ValueError("an image must be suplied")

        if image.width != self.__width or image.height != self.__height:
            raise ValueError(
                f"image must have the same dimensions as the display ({self.__width}x{self.__height})"
            )

        img = np.asarray(image)
        pix = np.zeros((self.__width, self.__height, 2), dtype=np.uint8)
        pix[..., [0]] = np.add(
            np.bitwise_and(img[..., [0]], 0xF8), np.right_shift(img[..., [1]], 5)
        )
        pix[..., [1]] = np.add(
            np.bitwise_and(np.left_shift(img[..., [1]], 3), 0xE0),
            np.right_shift(img[..., [2]], 3),
        )
        pix = pix.flatten().tolist()

        self.__set_window(0, 0, self.__width, self.__height)
        for i in range(0, len(pix), 4096):
            self.__write_data(pix[i : i + 4096])


if __name__ == "__main__":
    from PIL import Image, ImageDraw

    lcd = LCD(LCD_1IN44_Config, LCD_1IN44_Pins)
    lcd.clear()

    image = Image.new("RGB", (lcd.width, lcd.height), "WHITE")
    draw = ImageDraw.Draw(image)

    color = [ "Red", "Orange", "Yellow", "Green", "Cyan", "Blue", "Violet" ]
    for i in range(7):
        p: float = i * 9.0
        draw.rectangle(((p, p),(127-p,127-p)), fill=color[i])

    lcd.display_image(image)
