import cv2
import os
import time
import threading
import re
import importlib.util
from pathlib import Path


def load_camera_probe():
    module_path = Path(__file__).resolve().parent / "src" / "utils" / "camera_probe.py"
    spec = importlib.util.spec_from_file_location("camera_probe", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

save_dir = "./capture/round4"
os.makedirs(save_dir, exist_ok=True)

camera_probe = load_camera_probe()
cap, camera_index = camera_probe.open_camera({
    'camera': 0,
    'max_camera_index': 5,
    'width': 1280,
    'height': 720,
    'fps': 30
}, cv2_module=cv2)
print(f"Camera opened on index {camera_index}")

if not cap.isOpened():
    print("? Failed to open camera")
    exit()


existing_files = [f for f in os.listdir(save_dir) if f.endswith(".jpg")]
pattern = re.compile(r"(\d{5})\.jpg")
existing_indices = [int(pattern.match(f).group(1)) for f in existing_files if pattern.match(f)]
frame_idx = max(existing_indices, default=0) + 1

save_interval = 0.5  
last_save_time = time.time()

def save_image(image, path):
    cv2.imwrite(path, image)

print("?? Start capturing images... Press Ctrl+C to stop.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("? Failed to read frame")
            break

        now = time.time()
        if now - last_save_time >= save_interval:
            filename = f"{frame_idx:05d}.jpg"
            filepath = os.path.join(save_dir, filename)
            threading.Thread(target=save_image, args=(frame.copy(), filepath)).start()
            print(f"? Saved {filename}")
            frame_idx += 1
            last_save_time = now

except KeyboardInterrupt:
    print("\n?? Interrupted by user (Ctrl+C)")

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("? Camera released, program exited.")
