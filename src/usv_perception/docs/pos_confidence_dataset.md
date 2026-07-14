# Position-confidence dataset

`generate_pos_confidence_dataset.py` records the noisy outputs of the active
Gazebo Classic sensors and uses `/model_states` only to make offline labels.
It never writes a world target coordinate or Gazebo model name into an input
sample, so the resulting training input cannot accidentally use ground truth.

## Output layout

```text
pos_confidence_sim_2000/
  metadata.json
  train/samples/000000.npz
  val/samples/001400.npz
  test/samples/001700.npz
  train.txt
  val.txt
  test.txt
```

Each `.npz` contains the raw modality arrays expected by
`~/c3_pos_confidence/convert.py` plus the two tensors expected by
`total_loss` in `pos_confidence.py`:

| Key | Shape | Meaning |
| --- | --- | --- |
| `dehaze_camera` | `[N,3]` | base-frame xyz point cloud |
| `gated_camera` | `[N,3]` | gated detector xyz points |
| `sonar` | `[N,3]` | delayed/noisy sonar xyz points |
| `radar` | `[N,4]` | xyz and radial velocity |
| `target_heatmap` | `[1,H,W]` | one positive cell per simulated target |
| `target_offset` | `[2,H,W]` | target offset from that cell center, in metres |

The default grid is `x=[0,150)`, `y=[-75,75)`, `voxel_size=1`, therefore
`H=W=150`.  Create `PillarGridConfig(0, 150, -75, 75, voxel_size=1.0)` when
training this dataset.

## Recording 2000 samples

Launch the simulation in terminal 1.  In terminal 2, source the same workspace
and run:

```bash
ros2 run usv_perception generate_pos_confidence_dataset.py --ros-args \
  -p use_sim_time:=true \
  -p output_dir:=~/c3/usv_ws/data/pos_confidence_sim_2000 \
  -p sample_count:=2000
```

The recorder appends to the target directory and retains at most 2000 samples.
When a new sample would become number 2001, it deletes only the chronologically
oldest sample, then writes the new one with a higher sequence number. It
refreshes `moving_vessel` once every roughly 0.5 seconds within 100 m of the
USV, then saves one sample after the sensor-settling window. Whole scenes are
assigned to one split only, with a 70/15/15 balancing target. It exits after
collecting the requested number of *new* samples; the default takes about
17 minutes of running simulation.

Start the simulator with its normal dynamic target controller disabled; the
recorder is the sole controller of `moving_vessel` while data is collected:

```bash
ros2 launch usv_bringup sim.launch.py dynamic_targets:=false
```

## Training conversion

For a loaded sample `sample`, pass only the four modality arrays to the existing
conversion helper.  Labels are not part of `raw_modalities`.

```python
import numpy as np
from convert import pack_single_frame

sample = np.load('samples/000000.npz')
raw_modalities = {key: sample[key] for key in
                  ('dehaze_camera', 'gated_camera', 'sonar', 'radar')}
batch = pack_single_frame(raw_modalities)
target_heatmap = sample['target_heatmap']
target_offset = sample['target_offset']
```

Positive labels are intentionally one cell wide.  The network bounds offsets to
half a voxel, so widening a target to a 3x3 block without recomputing offsets
would create invalid offset supervision.

Sonar points are accumulated for 0.8 seconds within the current scene. This
retains the simulator's delayed/missed-detection behavior but avoids reducing a
four-sector sonar to only the last received detection. Half of the generated
scenes are sampled within 52 m, inside the modeled 60 m sonar range; the other
half cover 52 to 100 m so the model also learns valid sonar absence at range.

## Training, validation, and test

The companion adapters are in `~/c3_pos_confidence`. Install their Python
dependencies once, then train from that directory:

```bash
cd ~/c3_pos_confidence
python3 -m pip install --user -r requirements.txt
python3 train.py \
  --dataset ~/c3/usv_ws/data/pos_confidence_sim_2000 \
  --epochs 40 \
  --batch-size 2
```

`train.py` trains on `train`, chooses `best.pt` by `val`, then reports the final
loss on the untouched `test` set. The model grid is read from `metadata.json`.
