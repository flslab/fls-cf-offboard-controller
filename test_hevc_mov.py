import subprocess
import numpy as np

w, h = 100, 100
fps = 30
frame = np.zeros((h, w, 4), dtype=np.uint8)
frame[:, :, 0] = 255
frame[:, :, 3] = 128

proc = subprocess.Popen(
    ['ffmpeg', '-y',
     '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', f'{w}x{h}', '-r', str(fps),
     '-i', 'pipe:0',
     '-c:v', 'hevc_videotoolbox', '-alpha_quality', '0.75', '-vtag', 'hvc1',
     '-loglevel', 'error', 'test_hevc2.mov'],
    stdin=subprocess.PIPE,
)
for _ in range(30):
    proc.stdin.write(frame.tobytes())
proc.stdin.close()
proc.wait()

proc2 = subprocess.run(
    ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
     '-show_entries', 'stream=width,height,pix_fmt', '-of', 'csv=s=x:p=0', 'test_hevc2.mov'],
    capture_output=True, text=True, check=True,
)
print("Probe output:", proc2.stdout.strip())
