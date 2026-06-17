import sys
from PIL import Image

def test():
    img = Image.open("src/OTAanimation.jpg")
    print("Image size:", img.size)
    cols = 4
    rows = 3
    # Try with and without border
    total_w, total_h = img.size
    
    # Assuming no border:
    sw = total_w // cols
    sh = total_h // rows
    print("No border: sw=", sw, "sh=", sh)
    
    # Assuming 4px border
    border = 4
    sw2 = (total_w - (cols - 1) * border) // cols
    sh2 = (total_h - (rows - 1) * border) // rows
    print("4px border: sw=", sw2, "sh=", sh2)
    
test()
