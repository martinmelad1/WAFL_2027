"""Render ROS OccupancyGrid messages to browser-friendly PNG images."""
from io import BytesIO

from PIL import Image


def render_occupancy_grid(message) -> bytes:
    """Return a PNG for a nav_msgs/OccupancyGrid (unknown cells are gray)."""
    width, height = int(message.info.width), int(message.info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        return b""
    # OccupancyGrid starts at bottom-left; PNG rows start at top-left.
    pixels = bytearray(width * height)
    for y in range(height):
        for x in range(width):
            value = int(message.data[(height - 1 - y) * width + x])
            shade = 205 if value < 0 else 255 - round(2.55 * min(value, 100))
            pixels[y * width + x] = max(0, min(255, shade))
    image = Image.frombytes("L", (width, height), bytes(pixels)).convert("RGB")
    output = BytesIO()
    image.save(output, format="PNG", optimize=True)
    return output.getvalue()
