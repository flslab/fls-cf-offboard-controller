# Experiment Drive Uploader

This project provides a Python module that automatically uploads experiment JSON log files to Google Drive.

It supports two experiment types:

* `illumination`
* `interaction`

Each log file is uploaded to the corresponding Google Drive folder.

---

## Overview

This uploader is designed to be **plug-and-play**:

1. Your experiment generates a JSON log file
2. You call a single function
3. The file is uploaded to the correct Google Drive folder


---

## Project Structure

```
experiment_drive_uploader/
├─ pyproject.toml
├─ README.md
├─ credentials/
│  └─ service-account.json
├─ config/
│  └─ folders.json
├─ uploader/
│  ├─ __init__.py
│  ├─ drive_client.py
│  └─ retry.py
├─ dummy_experiment/
│  └─ generate_dummy_logs.py
```

---

## Requirements

* Python 3.10 or higher
* Google Drive folders already created
* Service account JSON key

---

## Step 1: Unzip the Project

Extract the zip file:

```bash
unzip experiment_drive_uploader.zip
cd experiment_drive_uploader
```

---

## Step 2: Create Virtual Environment

### macOS / Linux

```bash
python -m venv .venv
source .venv/bin/activate
```

### Windows

```bash
python -m venv .venv
.venv\Scripts\activate
```

---

## Step 3: Install the Package

```bash
python -m pip install -U pip
python -m pip install -e .
```

This installs the uploader in **editable mode** (best for development/testing).

---

## Step 4: Add Credentials

Place the service account JSON file here:

```
credentials/service-account.json
```

Do NOT rename this file unless you update the code.


## Step 5: Test with Dummy Experiment

Run:

```bash
python dummy_experiment/generate_dummy_logs.py
```

### What happens:

* JSON files are created locally
* Files are uploaded to Google Drive
* Files appear in:

  * `illumination` folder
  * `interaction` folder

If this works → setup is correct 

---

## Step 6: Use in Your Experiment Code

### Example integration:

```python
import json
from datetime import datetime
from uploader import upload_log

def write_and_upload_log(experiment_type, data):
    filename = f"{experiment_type}_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}.json"

    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    upload_log(filename, experiment_type)
```

### Example usage:

```python
write_and_upload_log("illumination", {"trial": 1, "status": "ok"})
write_and_upload_log("interaction", {"trial": 2, "status": "ok"})
```

---

## Step 7: Important Rules

* `experiment_type` must be exactly:

  * `"illumination"` OR
  * `"interaction"`

* The log file **must exist before upload**

* Do NOT delete the file before upload completes

---

## Step 8: What the Uploader Does Internally

When you call:

```python
upload_log("file.json", "illumination")
```

It:

1. Reads folder ID from config
2. Authenticates using service account
3. Creates file in Google Drive
4. Uploads content (resumable upload)
5. Places file in correct folder

---


## Step 9: Recommended Workflow

1. Setup environment
2. Test dummy experiment
3. Integrate uploader into experiment code
4. Run experiment → logs auto-upload

---

## Final Notes

* This module is designed to be **simple to integrate**
* Only one function is needed: `upload_log()`
* No manual upload steps required

Gmail : flsexp123@gmail.com
Password :  "IlluminationInteraction"
