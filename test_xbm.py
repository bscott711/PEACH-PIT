import os
from PIL import Image, ImageOps

img = Image.open("src/OTAanimation.jpg").convert("RGB")
# Crop frame 0
sw, sh = 374, 285
sprite = img.crop((0, 0, sw, sh))

# Target size 128x64
target_w, target_h = 128, 64
sprite = sprite.resize((target_w, target_h), Image.LANCZOS)

# Convert to 1-bit
sprite_bw = sprite.convert("1")
sprite_bw.save("test_frame0_bw.png")

# Inverted
sprite_inv = ImageOps.invert(sprite.convert("L")).convert("1")
sprite_inv.save("test_frame0_inv.png")

print("Saved test frames.")
