import subprocess
import numpy as np
from PIL import Image, ImageDraw

w, h = 100, 100
fps = 30
img = Image.new('RGBA', (w, h), (0, 0, 0, 0))
draw = ImageDraw.Draw(img)
draw.rectangle([10, 10, 90, 90], fill=(255, 0, 0, 128))
frame = np.array(img)

proc = subprocess.Popen(
    ['ffmpeg', '-y',
     '-f', 'rawvideo', '-pix_fmt', 'rgba', '-s', f'{w}x{h}', '-r', str(fps),
     '-i', 'pipe:0',
     '-c:v', 'libvpx-vp9', '-pix_fmt', 'yuva420p', '-auto-alt-ref', '0',
     '-loglevel', 'error', 'test.webm'],
    stdin=subprocess.PIPE,
)
for _ in range(10):
    proc.stdin.write(frame.tobytes())
proc.stdin.close()
proc.wait()

print("Done encoding")

proc2 = subprocess.run(
    ['ffprobe', '-v', 'error', '-select_streams', 'v:0',
     '-show_entries', 'stream=width,height,pix_fmt', '-of', 'csv=s=x:p=0', 'test.webm'],
    capture_output=True, text=True, check=True,
)
print("Probe output:", proc2.stdout.strip())
