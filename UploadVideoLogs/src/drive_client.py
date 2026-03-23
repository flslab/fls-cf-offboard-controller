import json
import os
from pathlib import Path

from google.oauth2 import service_account
from googleapiclient.discovery import build
from googleapiclient.http import MediaFileUpload
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
import pickle


SCOPES = ["https://www.googleapis.com/auth/drive.file"]

BASE_DIR = Path(__file__).resolve().parents[1]
CREDENTIALS_PATH = BASE_DIR / "credentials" / "service-account.json"
FOLDERS_PATH = BASE_DIR / "config" / "folders.json"

TOKEN_PATH = BASE_DIR / "credentials" / "token.pkl"
OAUTH_CLIENT_PATH = BASE_DIR / "credentials" / "oauth-client.json"

def load_folder_ids():
    with open(FOLDERS_PATH, "r", encoding="utf-8") as f:
        return json.load(f)

def get_drive_service():
    creds = None

    # Load saved token
    if TOKEN_PATH.exists():
        with open(TOKEN_PATH, "rb") as token:
            creds = pickle.load(token)

    # If no valid creds → login flow
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                str(OAUTH_CLIENT_PATH),
                SCOPES,
            )
            creds = flow.run_local_server(port=0)

        # Save token
        with open(TOKEN_PATH, "wb") as token:
            pickle.dump(creds, token)

    return build("drive", "v3", credentials=creds)

def upload_file(file_path: str, experiment_type: str):
    folder_ids = load_folder_ids()
    if experiment_type not in folder_ids:
        raise ValueError("experiment_type must be 'illumination' or 'interaction'")

    folder_id = folder_ids[experiment_type]
    file_path = Path(file_path)

    service = get_drive_service()

    metadata = {
        "name": file_path.name,
        "parents": [folder_id],
    }

    media = MediaFileUpload(
        str(file_path),
        mimetype="application/json",
        resumable=True,
    )

    request = service.files().create(
        body=metadata,
        media_body=media,
        fields="id, name"
    )

    response = None
    while response is None:
        status, response = request.next_chunk()
        if status:
            print(f"{file_path.name}: {int(status.progress() * 100)}%")

    return response["id"]