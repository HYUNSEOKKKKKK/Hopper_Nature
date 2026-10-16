# Demos and media

## Recorded hardware trials

The project maintainer supplied the source recordings for publication. The README displays silent GIF excerpts at **1× playback** and links to the full, uncut MP4 recordings. The MP4s preserve the original sound and duration, use browser-compatible H.264/AAC at 1280 × 720, and omit camera metadata. Original camera files remain outside Git.

| Trial | Source | Full recording | README preview |
| --- | --- | --- | --- |
| Outdoor trial on a running track | `IMG_8769.MP4` | [MP4, 20.75 s](../assets/demos/outdoor-trial.mp4) | [GIF](../assets/demos/outdoor-trial.gif), source 2–10 s, 1×, 10 fps |

The outdoor preview is cropped to the robot and nearby operator. The full recording retains the complete camera view, including the fall at the end of the trial and operator recovery. An operator is present in the recording. The clip documents a physical trial; a specific policy checkpoint, command trace and independently measured tracking error have not been supplied.

The machine-readable [media manifest](../assets/demos/manifest.json) records source/output hashes, intervals, crop, playback speed and encoding arguments.

## Lab YouTube presentation

A full presentation is planned for the lab's YouTube channel. Once the public video/channel URL is supplied, add it below the README demo previews and here. Keep the in-repository recordings available as direct alternatives.

<!-- LAB_YOUTUBE_VIDEO: awaiting the maintainer's public URL. -->

## Reproduce the published assets

Install FFmpeg and run from the repository root. Set `SOURCE_DIR` to the folder containing the original camera recordings. The source hashes in the manifest identify the exact inputs.

```bash
SOURCE_DIR=/path/to/recordings
mkdir -p assets/demos

ffmpeg -i "$SOURCE_DIR/IMG_8769.MP4" \
  -map 0:v:0 -map '0:a:0?' -map_metadata -1 -map_chapters -1 \
  -vf 'scale=1280:-2' -c:v libx264 -preset medium -crf 22 \
  -pix_fmt yuv420p -c:a aac -b:a 128k -movflags +faststart \
  assets/demos/outdoor-trial.mp4

ffmpeg -ss 2 -t 8 -i "$SOURCE_DIR/IMG_8769.MP4" \
  -filter_complex '[0:v]crop=1200:674:600:350,fps=10,scale=600:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3' \
  -loop 0 assets/demos/outdoor-trial.gif
```

The crop is expressed as width 1200, height 674, left 600, top 350 in the 1920 × 1080 input. No speed change or interpolation is applied. GIF playback is muted by format; the full MP4 contains audio.

Only the curated MP4s under `assets/demos/` are exempted from the video ignore rules. Keep additional raw recordings outside the repository. Record source filename/hash, interval, crop, playback multiplier and trial context whenever adding a clip.

## Model cover image

[hopper-overview.png](../assets/images/hopper-overview.png) is a static render of the active URDF and its DAE meshes in the environment's nominal joint configuration. Reproduce it with NumPy, Pillow and DejaVu Sans fonts:

```bash
python scripts/render_robot.py
```

Pillow is needed only for this optional utility, not for training. Ubuntu provides the fonts through `fonts-dejavu-core`.
