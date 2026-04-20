import subprocess
import numpy as np

w, h = 100, 100
fps = 30
# Create a frame of solid red with 50% transparency: R=255, G=0, B=0, A=128
frame = np.zeros((h, w, 4), dtype=np.uint8)
frame[:, :, 0] = 255
frame[:, :, 3] = 128

proc = subprocess.Popen(
    ['ffmpeg', '-y',
     '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', f'{w}x{h}', '-r', str(fps),
     '-i', 'pipe:0',
     '-c:v', 'libvpx-vp9',
     '-pix_fmt', 'yuva420p',
     '-metadata:s:v:0', 'alpha_mode=1',
     '-auto-alt-ref', '0',
     '-loglevel', 'warning', 'test3.webm'],
    stdin=subprocess.PIPE,
)
for _ in range(30):
    proc.stdin.write(frame.tobytes())
proc.stdin.close()
proc.wait()

proc2 = subprocess.run(
    ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
     '-show_entries', 'stream=width,height,pix_fmt', '-of', 'csv=s=x:p=0', 'test3.webm'],
    capture_output=True, text=True, check=True,
)
print("Probe output test3:", proc2.stdout.strip())
