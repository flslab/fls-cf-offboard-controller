import json

import joblib
import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures


def extract_stopping_data(file_path, low_speed_threshold=5):
    try:
        with open(file_path, 'r') as f:
            logs = json.load(f)
    except FileNotFoundError:
        print(f"Error: File {file_path} not found.")
        return pd.DataFrame()

    extracted_data = []

    # Iterate through logs to find 'User Disengage' events
    for i, entry in enumerate(logs):
        if entry.get("type") == "events" and entry.get("name") == "User Disengage":
            start_time = entry["data"]["time"]

            # 1. Find the closest previous Roll and Pitch (State)
            roll, pitch = None, None
            for j in range(i, -1, -1):
                if logs[j].get("type") == "state" and "stateEstimate.roll" in logs[j]["data"]:
                    roll = logs[j]["data"]["stateEstimate.roll"]
                    pitch = logs[j]["data"]["stateEstimate.pitch"]
                    break

            # 2. Find the closest previous Velocity (Frames)
            init_speed = None
            for j in range(i, -1, -1):
                if logs[j].get("type") == "frames":
                    v_vec = logs[j]["data"]["vel"]
                    init_speed = np.linalg.norm(v_vec)
                    break

            if roll is not None and init_speed is not None:
                # Calculate Angle: arccos(cos(roll)*cos(pitch))
                angle_rad = np.arccos(np.cos(np.deg2rad(roll)) * np.cos(np.deg2rad(pitch)))
                angle_deg = np.degrees(angle_rad)

                # 3. Look forward for the 'End Time'
                # First time speed < 0.1 before the next 'User Pushing' event
                end_time = None
                low_speed_count = 0
                for j in range(i + 1, len(logs)):
                    if logs[j].get("type") == "events" and logs[j].get("name") == "User Pushing":
                        if low_speed_count > 1:
                            end_time = logs[j]["data"]["time"]
                        break

                    if logs[j].get("type") == "frames":
                        v_vec = logs[j]["data"]["vel"]
                        curr_speed = np.linalg.norm(v_vec)
                        if curr_speed < 0.1:
                            low_speed_count += 1

                        if low_speed_count > low_speed_threshold:
                            end_time = logs[j]["data"]["time"]
                            break

                if end_time:
                    duration = end_time - start_time
                    extracted_data.append({
                        "A": angle_deg,  # Angle
                        "V": init_speed,  # Speed
                        "T": duration  # Duration
                    })

    return pd.DataFrame(extracted_data)


# --- Main Execution ---
# Update this filename to your actual log path
file_name = '../../logs/pos_translation_2026-03-16_17-05-05.json'
df = extract_stopping_data(file_name)

if not df.empty:
    print(f"Successfully extracted {len(df)} events.\n")

    # --- 1. Power Law Model: T = k * V^a * A^b ---
    # Ensure values are > 0 for log transformation
    df_log = df[(df['T'] > 0) & (df['V'] > 0) & (df['A'] > 0)].copy()

    if not df_log.empty:
        X_log = np.log(df_log[['A', 'V']])
        y_log = np.log(df_log['T'])

        model_log = LinearRegression().fit(X_log, y_log)
        k = np.exp(model_log.intercept_)
        b_angle, a_speed = model_log.coef_

        print("--- Power Law Model ---")
        print(f"Equation: T = {k:.4f} * V^{a_speed:.4f} * A^{b_angle:.4f}")
        print(f"R^2: {model_log.score(X_log, y_log):.4f}\n")

        # joblib.dump(model_log, 'power_law_model.pkl')

#     # --- 2. Polynomial Model (Degree 2) ---
#     poly = PolynomialFeatures(degree=5, include_bias=False)
#     X_poly = poly.fit_transform(df[['A', 'V']])
#     y = df['T']
#
#     model_poly = LinearRegression().fit(X_poly, y)
#
#     # Generate the equation string dynamically
#     features = poly.get_feature_names_out(['A', 'V'])
#     eqn_parts = [f"{model_poly.intercept_:.4f}"]
#     for coef, name in zip(model_poly.coef_, features):
#         eqn_parts.append(f"({coef:.4f} * {name})")
#
#     print("--- Polynomial Model (Degree 2) ---")
#     print(f"Poly Equation: T = {' + '.join(eqn_parts)}")
#     print(f"R^2: {model_poly.score(X_poly, y):.4f}")
#     # print(f"{model_poly.coef_}")
#     joblib.dump({'model': model_poly, 'poly': poly}, 'stabilizing_time.pkl')
# else:
#     print("No valid stopping events were found in the log.")