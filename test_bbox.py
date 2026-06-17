from PIL import Image, ImageChops

img = Image.open("src/OTAanimation.png").convert("L")
w, h = img.size

cols, rows = 4, 3
sw = w // cols
sh = h // rows

for i in range(cols * rows):
    col = i % cols
    row = i // cols
    x = col * sw
    y = row * sh
    
    # Extract cell
    cell = img.crop((x, y, x+sw, y+sh))
    
    # Get bounding box of non-black pixels
    # In PIL, getbbox works on truthy pixels. We can invert or just point it out.
    # Since background might be black (0) or white (255), let's check:
    bg_color = cell.getpixel((0,0))
    
    # Create mask of non-bg pixels
    if bg_color < 128:
        # Black bg
        mask = cell.point(lambda p: 255 if p > 20 else 0, mode="1")
    else:
        # White bg
        mask = cell.point(lambda p: 255 if p < 235 else 0, mode="1")
        
    bbox = mask.getbbox()
    if bbox:
        bx, by, bw, bh = bbox[0], bbox[1], bbox[2], bbox[3]
        width = bw - bx
        height = bh - by
        print(f"Frame {i}: bbox={bbox} -> {width}x{height}")
    else:
        print(f"Frame {i}: empty")

