from PIL import Image

img = Image.open("src/OTAanimation.png").convert("L")
w, h = img.size

cols, rows = 4, 3
sw = w // cols
sh = h // rows

print(f"sw={sw}, sh={sh}")

for i in range(cols):
    x = i * sw
    y = 0
    frame = img.crop((x, y, x+sw, y+sh))
    frame.save(f"test_frame_{i}.png")
