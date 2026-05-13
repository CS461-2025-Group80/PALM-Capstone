from http.server import HTTPServer, BaseHTTPRequestHandler
import json, csv, os, base64, argparse
from datetime import datetime

parser = argparse.ArgumentParser(description="reCamera prediction receiver")
parser.add_argument("--save-images", action="store_true", help="Save JPEG frames to an images/ folder")
args = parser.parse_args()

LOG_FILE = "predictions.csv"
IMAGE_DIR = "images" if args.save_images else None

if IMAGE_DIR:
    os.makedirs(IMAGE_DIR, exist_ok=True)
    print(f"Image saving enabled → ./{IMAGE_DIR}/")

FIELDNAMES = ["timestamp", "label", "x", "y", "w", "h", "score", "class_index", "image_file"]

if not os.path.isfile(LOG_FILE):
    with open(LOG_FILE, "w", newline="") as f:
        csv.DictWriter(f, fieldnames=FIELDNAMES).writeheader()

class PredictionHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            self.send_response(200); self.end_headers(); return

        data = json.loads(self.rfile.read(length))
        timestamp  = data.get("timestamp", datetime.now().isoformat())
        labels     = data.get("labels", [])
        boxes      = data.get("boxes", [])
        image_b64  = data.get("image", None)

        # Save image if flag is set and image data exists
        image_file = ""
        if IMAGE_DIR and image_b64:
            # Filename matches timestamp exactly so CSV rows link back to it
            safe_ts = timestamp.replace(":", "-").replace(".", "-")
            image_file = os.path.join(IMAGE_DIR, f"{safe_ts}.jpg")
            with open(image_file, "wb") as f:
                f.write(base64.b64decode(image_b64))

        # One CSV row per detected object in this frame
        rows = []
        for i, box in enumerate(boxes):
            if len(box) >= 6:
                x, y, w, h, score, class_index = box[:6]
                rows.append({
                    "timestamp":   timestamp,
                    "label":       labels[i] if i < len(labels) else f"class_{int(class_index)}",
                    "x":           round(x, 2),
                    "y":           round(y, 2),
                    "w":           round(w, 2),
                    "h":           round(h, 2),
                    "score":       round(score, 4),
                    "class_index": int(class_index),
                    "image_file":  image_file
                })

        if rows:
            with open(LOG_FILE, "a", newline="") as f:
                csv.DictWriter(f, fieldnames=FIELDNAMES).writerows(rows)
            print(f"[{timestamp}] {len(rows)} object(s): {[r['label'] for r in rows]}"
                  + (f" → {image_file}" if image_file else ""))
        else:
            print(f"[{timestamp}] No detections")

        self.send_response(200); self.end_headers()

    def log_message(self, *args):
        pass

print("Listening on 0.0.0.0:5000 ...")
HTTPServer(("0.0.0.0", 5000), PredictionHandler).serve_forever()