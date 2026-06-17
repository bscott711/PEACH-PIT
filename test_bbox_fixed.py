from PIL import Image, ImageChops

img = Image.open("src/OTAanimation.png").convert("L")
w, h = img.size
cols, rows = 4, 3
sw = w // cols
sh = h // rows

bboxes = []
cells = []

for i in range(cols * rows):
    col = i % cols
    row = i // cols
    x = col * sw
    y = row * sh
    cell = img.crop((x, y, x+sw, y+sh))
    cells.append(cell)
    
    bg_color = cell.getpixel((0,0))
    if bg_color < 128:
        mask = cell.point(lambda p: 255 if p > 20 else 0, mode="1")
    else:
        mask = cell.point(lambda p: 255 if p < 235 else 0, mode="1")
        
    bbox = mask.getbbox()
    bboxes.append(bbox)

# Find max width and height
max_w = max(b[2] - b[0] for b in bboxes if b)
max_h = max(b[3] - b[1] for b in bboxes if b)

print(f"Max bbox size: {max_w}x{max_h}")

for i, bbox in enumerate(bboxes):
    if bbox:
        cx = (bbox[0] + bbox[2]) / 2
        cy = (bbox[1] + bbox[3]) / 2
        
        crop_x0 = int(cx - max_w / 2)
        crop_y0 = int(cy - max_h / 2)
        crop_x1 = int(cx + max_w / 2)
        crop_y1 = int(cy + max_h / 2)
        
        # We save this centered crop
        c_crop = cells[i].crop((crop_x0, crop_y0, crop_x1, crop_y1))
        c_crop.save(f"test_frame_{i}.png")
