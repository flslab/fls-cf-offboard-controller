import subprocess
import numpy as np
import sys

w, h = 100, 100
fps = 30
frame = np.zeros((h, w, 4), dtype=np.uint8)
frame[:, :, 0] = 255
frame[:, :, 3] = 127

proc = subprocess.Popen(
    ['ffmpeg', '-y',
     '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', f'{w}x{h}', '-r', str(fps),
     '-i', 'pipe:0',
     '-c:v', 'prores_ks', '-profile:v', '4', '-pix_fmt', 'yuva444p10le',
     '-loglevel', 'error', 'test_prores.mov'],
    stdin=subprocess.PIPE,
)
for _ in range(30):
    proc.stdin.write(frame.tobytes())
proc.stdin.close()
proc.wait()

proc2 = subprocess.Popen(
    ['ffmpeg', '-i', 'test_prores.mov', '-f', 'rawvideo', '-pix_fmt', 'bgra', '-loglevel', 'error', 'pipe:1'],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
)
out = proc2.stdout.read(h * w * 4)
if len(out) == 0:
    print("Empty output!")
else:
    decoded = np.frombuffer(out, dtype=np.uint8).reshape(h, w, 4)
    print("Prores Min alpha:", decoded[:, :, 3].min(), "Max alpha:", decoded[:, :, 3].max())
