import json
import time
from datetime import datetime
from pathlib import Path

from uploader.drive_client import upload_file

LOG_DIR = Path("logs")
ILLUM_DIR = LOG_DIR / "illumination"
INTER_DIR = LOG_DIR / "interaction"

ILLUM_DIR.mkdir(parents=True, exist_ok=True)
INTER_DIR.mkdir(parents=True, exist_ok=True)

def make_log(experiment_type: str, index: int):
    payload = {
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "experiment_type": experiment_type,
        "run_id": index,
        "value": index * 10,
        "status": "ok"
    }

    folder = ILLUM_DIR if experiment_type == "illumination" else INTER_DIR
    file_path = folder / f"{experiment_type}_{index}.json"

    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    return file_path

def main():
    for i in range(3):
        p1 = make_log("illumination", i)
        upload_file(str(p1), "illumination")

        p2 = make_log("interaction", i)
        upload_file(str(p2), "interaction")

        time.sleep(1)

if __name__ == "__main__":
    main()