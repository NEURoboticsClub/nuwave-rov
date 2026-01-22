import os
from PIL import Image
import random

IMAGE_DIR = ""
OUTPUT_DIR = ""
BACKGROUND_SIZE = (1024, 1024)
NUM_OUTPUT_IMAGES = 100
MIN_GREEN_CRABS = 1
MIN_OTHER_CRABS = 1
MAX_TOTAL_CRABS = 4
SCALE_RANGE = (0.1, 0.4)
SEED = 5

CLASSES = ["jonahCrab", "greenCrab", "rockCrab"]

random.seed(SEED)
os.makedirs(OUTPUT_DIR, exist_ok=True)

class_images = {c: [] for c in CLASSES}

for file in os.listdir(IMAGE_DIR):
    for c in CLASSES:
        if file.startswith(c):
            class_images[c].append(os.path.join(IMAGE_DIR, file))


def random_scale(img, bg_size):
    bg_w, bg_h = bg_size
    scale = random.uniform(SCALE_RANGE[0], SCALE_RANGE[1])
    target_w = int(bg_w * scale)
    aspect = img.height / img.width
    target_h = int(target_w * aspect)
    return img.resize((target_w, target_h, Image.LANCZOS))


for i in range(NUM_OUTPUT_IMAGES):

    if i % 20 == 0:
        print(f"Generating image {i}")

    num_green_crabs = random.randint(MIN_GREEN_CRABS, MAX_TOTAL_CRABS - MIN_OTHER_CRABS)
    num_other_crabs = MAX_TOTAL_CRABS - num_green_crabs
    bg = Image.new("RGB", BACKGROUND_SIZE, (255, 255, 255))

    for _ in range(num_green_crabs):
        obj_path = random.choice(class_images["greenCrab"])
        obj = Image.open(obj_path).convert("RGBA")
        obj = random_scale(obj, BACKGROUND_SIZE)

        x = random.randint(0, BACKGROUND_SIZE[0])
        y = random.randint(0, BACKGROUND_SIZE[1])

        bg.paste(obj, (x, y), obj)

    
    for _ in range(num_other_crabs):
        obj_path = random.choice(class_images["jonahCrab"] + class_images["rockCrab"])
        obj = Image.open(obj_path).convert("RGBA")
        obj = random_scale(obj, BACKGROUND_SIZE)

        x = random.randint(0, BACKGROUND_SIZE[0])
        y = random.randint(0, BACKGROUND_SIZE[1])

        bg.paste(obj, (x, y), obj)

    
    out_path = os.path.join(OUTPUT_DIR, f"{i}.jpg")
    bg.save(out_path)