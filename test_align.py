import cv2
import numpy as np
from PIL import Image

# Read image
img = Image.open("src/OTAanimation.png").convert("L")
img_np = np.array(img)

h, w = img_np.shape
cols, rows = 4, 3

print(f"Image shape: {w}x{h}")

sw, sh = w // cols, h // rows
print(f"sw={sw}, sh={sh}")

# Save the first 3 frames with sw, sh grid to see if they align
for i in range(3):
    x = i * sw
    y = 0
    frame = img_np[y:y+sh, x:x+sw]
    Image.fromarray(frame).save(f"test_frame_{i}.png")
