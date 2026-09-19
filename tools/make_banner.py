"""Build a GameCube BNR1 banner (opening.bnr) from a 96x32 PNG.

Layout, 0x1960 bytes total:
  0x0000  "BNR1"
  0x0020  96x32 image, RGB5A3, big-endian, in 4x4 tiles   (6144 bytes)
  0x1820  short game name   (32)
  0x1840  short maker       (32)
  0x1860  long game name    (64)
  0x18A0  long maker        (64)
  0x18E0  description      (128)
"""

import struct
import sys

from PIL import Image

SRC = r"C:\Users\NoSig\Downloads\Connect_4_game_logobnr.png"
DST = r"C:\Users\NoSig\Documents\connect4\Connect4\opening.bnr"

SHORT_NAME = "Connect4"
SHORT_MAKER = "https://github.com/myuu-151"
LONG_NAME = "Connect4"
LONG_MAKER = "https://github.com/myuu-151"
DESCRIPTION = "Connect 4, remade in Octave Engine"


def to_rgb5a3(r, g, b, a):
    # Opaque pixels get 5 bits per channel and no alpha; the rest trade colour
    # precision for 3 bits of alpha.
    if a >= 0xE0:
        return 0x8000 | ((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3)
    return ((a >> 5) << 12) | ((r >> 4) << 8) | ((g >> 4) << 4) | (b >> 4)


def encode(image):
    image = image.convert("RGBA")

    if image.size != (96, 32):
        image = image.resize((96, 32), Image.LANCZOS)

    px = image.load()
    out = bytearray()

    # 4x4 tiles, left to right then top to bottom.
    for block_y in range(0, 32, 4):
        for block_x in range(0, 96, 4):
            for y in range(4):
                for x in range(4):
                    r, g, b, a = px[block_x + x, block_y + y]
                    out += struct.pack(">H", to_rgb5a3(r, g, b, a))

    assert len(out) == 6144, len(out)
    return bytes(out)


def field(text, size):
    raw = text.encode("ascii", "replace")[: size - 1]
    return raw + b"\0" * (size - len(raw))


def main():
    banner = bytearray(b"\0" * 0x1960)
    banner[0x0000:0x0004] = b"BNR1"
    banner[0x0020:0x1820] = encode(Image.open(SRC))
    banner[0x1820:0x1840] = field(SHORT_NAME, 0x20)
    banner[0x1840:0x1860] = field(SHORT_MAKER, 0x20)
    banner[0x1860:0x18A0] = field(LONG_NAME, 0x40)
    banner[0x18A0:0x18E0] = field(LONG_MAKER, 0x40)
    banner[0x18E0:0x1960] = field(DESCRIPTION, 0x80)

    with open(DST, "wb") as f:
        f.write(banner)

    print("wrote %s (%d bytes)" % (DST, len(banner)))


if __name__ == "__main__":
    sys.exit(main())
