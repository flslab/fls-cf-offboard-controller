import pandas as pd
import numpy as np

# === Load CSV ===
df = pd.read_csv("HQ_Thrust.csv")

# Extract columns
pwm = df["thrust_cmd"].to_numpy(dtype=float)     # motor terminal voltage [V]
t_N = df["loadcell_mean"].to_numpy(dtype=float) * 0.009800665
t_M = t_N # use total thrust

# Keep only where thrust is positive and not saturated
mask = (t_M > 0) & (pwm > 0)
pwm, t_M = pwm[mask], t_M[mask]

# Fit cubic: t = a pwm^3 + b pwm^2 + c pwm + d
a, b, c, d = np.polyfit(pwm, t_M, deg=3)

print("Thrust = a * PWM^3 + b * PWM^2 + c * PWM + d")
print("a =", a)
print("b =", b)
print("c =", c)
print("d =", d)


# Sanity checks
pt = np.linspace(pwm.min(), pwm.max(), 200)
thrust_pred = ((a*pt + b)*pt + c)*pt + d
assert np.all(thrust_pred >= 0), "Negative thrust in range—trim fit/range."

pwm = 20000
thrust_pred = ((a*pwm + b)*pwm + c)*pwm + d
print(f"With PWM: {pwm}, Thrust output is: {thrust_pred}")

print("=================================")

a, b, c, d = np.polyfit(t_M, pwm, deg=3)

print("PWM = a * Thrust^3 + b * Thrust^2 + c * Thrust + d")
print("a =", a)
print("b =", b)
print("c =", c)
print("d =", d)
# Sanity checks
tt = np.linspace(t_M.min(), t_M.max(), 200)
pwm_pred = ((a*tt + b)*tt + c)*tt + d
assert np.all(pwm_pred >= 0), "Negative pwm in range—trim fit/range."
