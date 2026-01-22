import os
from PIL import Image
import random

IMAGE_DIR = "augmented_crabs"
OUTPUT_DIR = "crabs_on_background_aboveGround"
BACKGROUND_SIZE = (1024, 1024)
NUM_OUTPUT_IMAGES = 20
MIN_GREEN_CRABS = 1
MIN_OTHER_CRABS = 1
MAX_TOTAL_CRABS = 10

CLASSES = ["jonahCrab", "greenCrab", "rockCrab"]

os.makedirs(OUTPUT_DIR, exist_ok=True)

class_images = {c: [] for c in CLASSES}

for file in os.listdir(IMAGE_DIR):
    for c in CLASSES:
        if file.startswith(c):
            class_images[c].append(os.path.join(IMAGE_DIR, file))


def check_overlap(new_box, placed_boxes, margin=10):
    """Check if new_box overlaps with any placed boxes."""
    x1, y1, x2, y2 = new_box
    for box in placed_boxes:
        bx1, by1, bx2, by2 = box
        # Add margin to prevent touching
        if not (x2 + margin < bx1 or x1 > bx2 + margin or 
                y2 + margin < by1 or y1 > by2 + margin):
            return True
    return False


def find_valid_position(img_size, bg_size, placed_boxes, max_attempts=100):
    """Find a valid non-overlapping position for the image."""
    img_w, img_h = img_size
    bg_w, bg_h = bg_size
    
    for _ in range(max_attempts):
        x = random.randint(0, max(0, bg_w - img_w))
        y = random.randint(0, max(0, bg_h - img_h))
        
        new_box = (x, y, x + img_w, y + img_h)
        
        if not check_overlap(new_box, placed_boxes):
            return x, y, new_box
    
    return None, None, None


def scale_to_fit(images, bg_size, max_total):
    """Calculate appropriate scale to fit all images on background."""
    bg_w, bg_h = bg_size
    
    # Estimate total area needed
    total_area = sum(img.width * img.height for img in images)
    bg_area = bg_w * bg_h
    
    # Use 70% of background to leave space between crabs
    usable_area = bg_area * 0.7
    
    if total_area > usable_area:
        scale = (usable_area / total_area) ** 0.5
    else:
        # If images are small, limit max size
        scale = min(1.0, (bg_w / max_total / max(img.width for img in images)) * 0.8)
    
    return scale


for i in range(NUM_OUTPUT_IMAGES):

    if i % 20 == 0:
        print(f"Generating image {i}")

    num_green_crabs = random.randint(MIN_GREEN_CRABS, MAX_TOTAL_CRABS - MIN_OTHER_CRABS)
    num_other_crabs = MAX_TOTAL_CRABS - num_green_crabs
    
    # Load all images first
    all_crabs = []
    
    for _ in range(num_green_crabs):
        obj_path = random.choice(class_images["greenCrab"])
        obj = Image.open(obj_path).convert("RGBA")
        all_crabs.append(obj)
    
    for _ in range(num_other_crabs):
        obj_path = random.choice(class_images["jonahCrab"] + class_images["rockCrab"])
        obj = Image.open(obj_path).convert("RGBA")
        all_crabs.append(obj)
    
    # Calculate scale to fit all crabs
    scale = scale_to_fit(all_crabs, BACKGROUND_SIZE, MAX_TOTAL_CRABS)
    
    # Scale all crabs uniformly
    scaled_crabs = []
    for crab in all_crabs:
        new_w = int(crab.width * scale)
        new_h = int(crab.height * scale)
        scaled_crabs.append(crab.resize((new_w, new_h), Image.Resampling.LANCZOS))
    
    # Create background
    bg = Image.new("RGB", BACKGROUND_SIZE, (255, 255, 255))
    placed_boxes = []
    
    # Place crabs without overlap
    for crab in scaled_crabs:
        x, y, box = find_valid_position(crab.size, BACKGROUND_SIZE, placed_boxes)
        
        if x is not None:
            bg.paste(crab, (x, y), crab)
            placed_boxes.append(box)
        else:
            print(f"Warning: Could not place crab in image {i}")
    
    out_path = os.path.join(OUTPUT_DIR, f"{i}.jpg")
    bg.save(out_path)

print(f"Generated {NUM_OUTPUT_IMAGES} images in {OUTPUT_DIR}")