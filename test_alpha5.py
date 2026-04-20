import subprocess
import numpy as np
import sys

w, h = 100, 100
fps = 30
frame = np.zeros((h, w, 4), dtype=np.uint8)
frame[:, :, 0] = 255
frame[:, :, 3] = 128

proc = subprocess.Popen(
    ['ffmpeg', '-y',
     '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', f'{w}x{h}', '-r', str(fps),
     '-i', 'pipe:0',
     '-c:v', 'libvpx-vp9', '-pix_fmt', 'yuva420p', '-auto-alt-ref', '0',
     sys.argv[1] if len(sys.argv)>1 else '-y', sys.argv[2] if len(sys.argv)>2 else '-y',
     '-loglevel', 'error', 'test5.webm'],
    stdin=subprocess.PIPE,
)
for _ in range(30):
    proc.stdin.write(frame.tobytes())
proc.stdin.close()
proc.wait()

proc2 = subprocess.Popen(
    ['ffmpeg', '-i', 'test5.webm', '-f', 'rawvideo', '-pix_fmt', 'bgra', '-loglevel', 'error', 'pipe:1'],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
)
out = proc2.stdout.read(h * w * 4)
decoded = np.frombuffer(out, dtype=np.uint8).reshape(h, w, 4)
print("Min alpha:", decoded[:, :, 3].min(), "Max alpha:", decoded[:, :, 3].max())
