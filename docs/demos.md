# Demos and media

## Current assets

[hopper-overview.png](../assets/images/hopper-overview.png) is a static render of the active URDF and its DAE meshes in the environment's nominal joint configuration. It is not a simulation screenshot or hardware photograph. Reproduce it with NumPy, Pillow and DejaVu Sans fonts:

```bash
python scripts/render_robot.py
```

Pillow is needed only for this optional illustration utility, not for training. On Ubuntu, the font files are provided by `fonts-dejavu-core`.

## Recorded demos to add

The maintainer will supply original recordings. Suggested presentation:

| Clip | What it should show | Required context |
| --- | --- | --- |
| Simulation overview | Standing, hopping and a clear camera view of the mechanism | Checkpoint, model revision, commands, resets and playback speed |
| Command sequence | Transitions between zero and nonzero velocity commands | Command plot or overlay, simulation time and real playback speed |
| Hardware demo, if supplied | Recorded physical behavior | Hardware setup, operator/support conditions, recording date and scope |
| Robot photograph | Actual assembled robot and parallel ankle detail | Photograph credit and model/hardware revision |

Keep labels faithful to the recording. A clip with automatic episode resets should say so. Do not infer tracking accuracy, robustness or autonomous operation from an edited preview alone.

For each final clip, record the source filename, date, selected interval, crop, playback multiplier, encoding command, associated checkpoint hash, seed, command sequence, source commit and model revision. Add measured metrics only when their extraction and scope are documented.

## Make a lightweight preview

Install FFmpeg, then choose an actual interval from the supplied recording:

```bash
mkdir -p assets/demos
ffmpeg -ss 0 -t 8 -i /path/to/original.mp4 \
  -filter_complex "[0:v]fps=12,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" \
  -loop 0 assets/demos/simulation.gif
```

This example keeps 1× timing. If a speed change is used, document it in the caption. Keep the original recording outside Git; upload full videos as release assets or use a GitHub-hosted video link. GIF previews and photographs can live under `assets/`. Raw MP4/MOV/WebM files are ignored by Git.

After creating a real preview, add an image link in the root README's Demos section, for example `![Recorded simulation](assets/demos/simulation.gif)`, followed by its scope and full video link. Do not add the link until the asset exists.

No video, physical photograph or selected pretrained result has been supplied for the initial documentation update.
