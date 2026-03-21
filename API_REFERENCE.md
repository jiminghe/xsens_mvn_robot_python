# Xsens MVN Robot SDK — Python API Reference

Version: **0.1.3**
Platform: Linux x86_64 / aarch64
Python: 3.10, 3.13 (3.8, 3.11, 3.12 supported when interpreter is available)

---

## Table of Contents

1. [Installation](#installation)
2. [Quick Start](#quick-start)
3. [XsensWrapper Class](#xsenswrapper-class)
   - [Lifecycle Methods](#lifecycle-methods)
   - [Discovery Methods](#discovery-methods)
   - [Session & Timing Methods](#session--timing-methods)
   - [Link Methods](#link-methods)
   - [Joint Methods](#joint-methods)
   - [Tracker Methods](#tracker-methods)
   - [Center of Mass Methods](#center-of-mass-methods)
   - [Scale Data Methods](#scale-data-methods)
4. [Body Segment Reference](#body-segment-reference)
5. [Coordinate System](#coordinate-system)
6. [Common Patterns](#common-patterns)
7. [Error Handling](#error-handling)

---

## Installation

Install the pre-built wheel for your Python version:

```bash
# Python 3.10
pip install xsens_mvn_robot-0.1.3-cp310-none-linux_x86_64.whl

# Python 3.13
pip install xsens_mvn_robot-0.1.3-cp313-none-linux_x86_64.whl
```

Verify the installation:

```python
from xsens_mvn_robot import XsensWrapper
print("Import OK")
```

---

## Quick Start

```python
import time
from xsens_mvn_robot import XsensWrapper

# 1. Create the wrapper (default port: 9763)
device = XsensWrapper(port=9763)

# 2. Initialize — blocks until the first quaternion datagram arrives
if not device.init():
    raise RuntimeError("Failed to initialize Xsens device")

# 3. Discover available body segments
link_names = device.get_link_names()
print("Links:", link_names)

# 4. Start streaming
device.start()
time.sleep(0.5)  # let data stabilize

# 5. Main loop
try:
    last_counter = -1
    while True:
        counter = int(device.get_sample_counter())
        if counter == last_counter:
            time.sleep(0.001)
            continue
        last_counter = counter

        pos = device.get_link_position("pelvis")
        ori = device.get_link_orientation("pelvis")
        print(f"[{counter}] pelvis pos={pos}  ori={ori}")

except KeyboardInterrupt:
    pass
finally:
    # 6. Stop streaming
    device.stop()
```

---

## XsensWrapper Class

```python
from xsens_mvn_robot import XsensWrapper
```

`XsensWrapper` is the **only public class** in this SDK. All data access goes through it.

---

### Lifecycle Methods

#### `__init__(port=9763)`

Create the wrapper instance.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port`    | `int` | `9763` | UDP port that Xsens MVN Studio streams to |

```python
device = XsensWrapper(port=9763)
```

Raises `ValueError` if `port` is not a valid integer in range 1–65535.
Raises `RuntimeError` if the underlying device object cannot be created.

---

#### `init() -> bool`

Initialize the device and build the internal body model. **Must be called before any data access.**

Blocks until a quaternion datagram is received from Xsens MVN Studio, up to ~1 second (100 attempts × 10 ms).

```python
if not device.init():
    raise RuntimeError("Failed to connect to Xsens MVN")
```

Returns `True` on success, `False` if no data was received in time.

> **Note:** Xsens MVN Studio must be running and streaming to the configured port before calling `init()`.

---

#### `start()`

Begin receiving and processing UDP data frames. Call this after `init()`.

```python
device.start()
```

---

#### `stop()`

Stop receiving data. Should always be called before exiting, even on error.

```python
device.stop()
```

---

#### `is_active() -> bool`

Check whether the device is currently streaming.

```python
if device.is_active():
    print("Streaming is active")
```

---

#### `has_finger_data() -> bool`

Returns `True` if the current session includes finger tracking segments (requires MVN Animate Plus/Pro with gloves).

```python
if device.has_finger_data():
    print("Finger tracking enabled")
```

---

#### `get_prop_count() -> int`

Returns the number of prop trackers attached in the current session (0 if none).

```python
n_props = device.get_prop_count()
```

---

### Discovery Methods

Call these after `init()` to find out which segments/joints are available in the current MVN session.

#### `get_link_names() -> list[str]`

Returns a list of all body segment (link) names.

```python
links = device.get_link_names()
# e.g. ['pelvis', 'l5', 'l3', 't12', 't8', 'neck', 'head',
#        'right_shoulder', 'right_upper_arm', ...]
```

---

#### `get_joint_names() -> list[str]`

Returns a list of all joint names (body + finger joints if available).

```python
joints = device.get_joint_names()
# e.g. ['l5_s1', 'l4_l3', 'right_shoulder', 'right_elbow', ...]
```

---

#### `get_tracker_names() -> list[str]`

Returns a list of raw IMU tracker names.

```python
trackers = device.get_tracker_names()
```

---

### Session & Timing Methods

#### `get_sample_counter() -> int`

The monotonically increasing frame counter from MVN Studio. Use this to detect new frames.

```python
counter = int(device.get_sample_counter())
```

**Pattern — poll only on new frames:**

```python
last = -1
while True:
    counter = int(device.get_sample_counter())
    if counter != last:
        last = counter
        # process new frame ...
    time.sleep(0.001)
```

---

#### `get_frame_time() -> int`

Returns the time of the current frame in milliseconds (from MVN's internal clock).

```python
frame_ms = int(device.get_frame_time())
```

---

#### `get_time_code() -> tuple[int, int, int, int]`

Returns the SMPTE time code of the current frame as `(hour, minute, second, nanosecond)`.

```python
h, m, s, ns = device.get_time_code()
print(f"Time: {h:02d}:{m:02d}:{s:02d}.{ns // 1_000_000:03d}")
```

---

#### `get_received_datagram_types() -> list[int]`

Returns a list of datagram type IDs received in the current frame. Useful for debugging what MVN is streaming.

```python
types = device.get_received_datagram_types()
for t in types:
    print(f"  datagram type: 0x{t:02x}")
```

---

#### `get_metadata() -> dict[str, str]`

Returns session metadata from MVN Studio (e.g. subject name, session ID).

```python
meta = device.get_metadata()
for key, value in meta.items():
    print(f"  {key}: {value}")
```

---

### Link Methods

A **link** is a rigid body segment (e.g. `"pelvis"`, `"right_upper_arm"`). All link data is in the **global world frame** unless noted otherwise.

#### `get_link_info(link_name) -> dict`

Returns static info about a link.

```python
info = device.get_link_info("pelvis")
# {'name': 'pelvis', 'parent_joint': ''}
```

---

#### `get_link_position(link_name) -> np.ndarray`

Global 3D position of the link origin, in **meters**.
Returns `np.ndarray` of shape `(3,)` as `[x, y, z]`.

```python
pos = device.get_link_position("pelvis")
# e.g. array([0.012, 0.953, -0.004])
```

---

#### `get_link_orientation(link_name) -> np.ndarray`

Global orientation as a unit quaternion.
Returns `np.ndarray` of shape `(4,)` as **`[w, x, y, z]`** (scalar-first).

```python
quat = device.get_link_orientation("pelvis")
# e.g. array([ 0.999,  0.012, -0.003,  0.021])
```

Convert to rotation matrix with scipy:

```python
from scipy.spatial.transform import Rotation as R
rot = R.from_quat(quat, scalar_first=True)
matrix = rot.as_matrix()  # (3, 3)
```

---

#### `get_link_linear_velocity(link_name) -> np.ndarray`

Linear velocity of the link in **m/s**, global frame.
Returns `np.ndarray` of shape `(3,)` as `[x, y, z]`.

```python
vel = device.get_link_linear_velocity("pelvis")
```

---

#### `get_link_angular_velocity(link_name) -> np.ndarray`

Angular velocity in **rad/s**, global frame.
Returns `np.ndarray` of shape `(3,)` as `[x, y, z]`.

```python
ang_vel = device.get_link_angular_velocity("right_lower_leg")
```

---

#### `get_link_linear_acceleration(link_name) -> np.ndarray`

Linear acceleration in **m/s²**, global frame.
Returns `np.ndarray` of shape `(3,)`.

```python
acc = device.get_link_linear_acceleration("pelvis")
```

---

#### `get_link_angular_acceleration(link_name) -> np.ndarray`

Angular acceleration in **rad/s²**, global frame.
Returns `np.ndarray` of shape `(3,)`.

```python
ang_acc = device.get_link_angular_acceleration("pelvis")
```

---

### Joint Methods

A **joint** connects two adjacent links. Joint angle data is available but **requires joint angle datagrams** to be enabled in MVN Studio (network streaming settings). If only quaternion datagrams are enabled, joint angle data will be zero.

#### `get_joint_info(joint_name) -> dict`

Returns static info about a joint.

```python
info = device.get_joint_info("right_elbow")
# {'name': 'right_elbow', 'parent_link': 'right_upper_arm', 'child_link': 'right_forearm'}
```

---

#### `get_joint_angles(joint_name) -> np.ndarray`

Joint angles as Euler angles in **degrees**, ISB convention.
Returns `np.ndarray` of shape `(3,)` as `[abduction/adduction, internal/external_rotation, flexion/extension]`.

```python
angles = device.get_joint_angles("right_elbow")
# e.g. array([  2.1,  -5.3,  87.4])
```

> **Note:** Requires joint angle datagram to be enabled in MVN Studio.

---

#### `get_joint_computed_angles(joint_name) -> np.ndarray`

Computed joint angles derived from the quaternion datagram (computed by the SDK, not MVN Studio).
Same format as `get_joint_angles` — `[aa, ie, fe]` in **degrees**.

```python
computed = device.get_joint_computed_angles("right_elbow")
```

This works even when joint angle streaming is disabled in MVN Studio.

---

### Tracker Methods

A **tracker** is a raw IMU sensor unit. Tracker data is in the **sensor frame** (not globally fused).

#### `get_tracker_info(tracker_name) -> dict`

Returns static info about a tracker.

```python
info = device.get_tracker_info("Pelvis")
# {'name': 'Pelvis', 'segment_id': 1}
```

---

#### `get_tracker_orientation(tracker_name) -> np.ndarray`

Sensor orientation as a unit quaternion `[w, x, y, z]` (scalar-first).

```python
quat = device.get_tracker_orientation("Pelvis")
```

---

#### `get_tracker_acceleration(tracker_name) -> np.ndarray`

Raw accelerometer reading in **m/s²**, sensor frame.
Returns `np.ndarray` of shape `(3,)`.

```python
acc = device.get_tracker_acceleration("Pelvis")
```

---

#### `get_tracker_angular_velocity(tracker_name) -> np.ndarray`

Gyroscope reading in **rad/s**, sensor frame.
Returns `np.ndarray` of shape `(3,)`.

```python
gyro = device.get_tracker_angular_velocity("Pelvis")
```

---

#### `get_tracker_magnetic_field(tracker_name) -> np.ndarray`

Magnetometer reading in **arbitrary units**, sensor frame.
Returns `np.ndarray` of shape `(3,)`.

```python
mag = device.get_tracker_magnetic_field("Pelvis")
```

---

#### `get_tracker_free_acceleration(tracker_name) -> np.ndarray`

Acceleration with gravity removed, in **m/s²**, global frame.
Returns `np.ndarray` of shape `(3,)`.

```python
free_acc = device.get_tracker_free_acceleration("Pelvis")
```

---

### Center of Mass Methods

Center of mass is computed by MVN Studio from the full body model.

#### `get_center_of_mass_position() -> np.ndarray`

Whole-body center of mass position in **meters**, global frame.
Returns `np.ndarray` of shape `(3,)`, dtype `float32`.

```python
com_pos = device.get_center_of_mass_position()
# e.g. array([0.013, 0.981, -0.002], dtype=float32)
```

---

#### `get_center_of_mass_velocity() -> np.ndarray`

Whole-body center of mass velocity in **m/s**.
Returns `np.ndarray` of shape `(3,)`, dtype `float32`.

```python
com_vel = device.get_center_of_mass_velocity()
```

---

#### `get_center_of_mass_acceleration() -> np.ndarray`

Whole-body center of mass acceleration in **m/s²**.
Returns `np.ndarray` of shape `(3,)`, dtype `float32`.

```python
com_acc = device.get_center_of_mass_acceleration()
```

---

### Scale Data Methods

Scale data contains subject body dimension measurements (segment lengths, joint positions) captured during the MVN calibration.

#### `get_scale_data_info() -> dict`

Returns a dictionary with two keys:

- `segments` — list of dicts, each with `name` (str) and `position` (np.ndarray `[x,y,z]` in meters)
- `points` — list of dicts, each with `name`, `segment_id`, `point_id`, `characteristics`, and `position`

```python
scale = device.get_scale_data_info()

for seg in scale['segments']:
    print(f"Segment: {seg['name']:20s}  pos={seg['position']}")

for pt in scale['points']:
    print(f"Point:   {pt['name']:20s}  seg={pt['segment_id']}  pos={pt['position']}")
```

---

## Body Segment Reference

The following link names are used across MVN sessions. Use `get_link_names()` to confirm availability for your specific setup.

### Torso & Head

| Link Name | Description         |
|-----------|---------------------|
| `pelvis`  | Pelvis (root)       |
| `l5`      | Lumbar spine (L5)   |
| `l3`      | Lumbar spine (L3)   |
| `t12`     | Thoracic spine (T12)|
| `t8`      | Thoracic spine (T8) |
| `neck`    | Neck (C7)           |
| `head`    | Head                |

### Left Arm

| Link Name         | Description       |
|-------------------|-------------------|
| `left_shoulder`   | Left shoulder     |
| `left_upper_arm`  | Left upper arm    |
| `left_forearm`    | Left forearm      |
| `left_hand`       | Left hand         |

### Right Arm

| Link Name          | Description        |
|--------------------|--------------------|
| `right_shoulder`   | Right shoulder     |
| `right_upper_arm`  | Right upper arm    |
| `right_forearm`    | Right forearm      |
| `right_hand`       | Right hand         |

### Left Leg

| Link Name          | Description        |
|--------------------|--------------------|
| `left_upper_leg`   | Left upper leg     |
| `left_lower_leg`   | Left lower leg     |
| `left_foot`        | Left foot          |
| `left_toe`         | Left toe           |

### Right Leg

| Link Name           | Description        |
|---------------------|--------------------|
| `right_upper_leg`   | Right upper leg    |
| `right_lower_leg`   | Right lower leg    |
| `right_foot`        | Right foot         |
| `right_toe`         | Right toe          |

### Body Joint Reference

| Joint Name           | Parent Link       | Child Link         |
|----------------------|-------------------|--------------------|
| `l5_s1`              | pelvis            | l5                 |
| `l4_l3`              | l5                | l3                 |
| `l1_t12`             | l3                | t12                |
| `t9_t8`              | t12               | t8                 |
| `t1_c7`              | t8                | neck               |
| `c1_head`            | neck              | head               |
| `right_c7_shoulder`  | t8                | right_shoulder     |
| `right_shoulder`     | right_shoulder    | right_upper_arm    |
| `right_elbow`        | right_upper_arm   | right_forearm      |
| `right_wrist`        | right_forearm     | right_hand         |
| `left_c7_shoulder`   | t8                | left_shoulder      |
| `left_shoulder`      | left_shoulder     | left_upper_arm     |
| `left_elbow`         | left_upper_arm    | left_forearm       |
| `left_wrist`         | left_forearm      | left_hand          |
| `right_hip`          | pelvis            | right_upper_leg    |
| `right_knee`         | right_upper_leg   | right_lower_leg    |
| `right_ankle`        | right_lower_leg   | right_foot         |
| `right_ballfoot`     | right_foot        | right_toe          |
| `left_hip`           | pelvis            | left_upper_leg     |
| `left_knee`          | left_upper_leg    | left_lower_leg     |
| `left_ankle`         | left_lower_leg    | left_foot          |
| `left_ballfoot`      | left_foot         | left_toe           |

---

## Coordinate System

- **World frame**: Y-up, right-handed
- **Positions**: meters (float64)
- **Quaternions**: scalar-first `[w, x, y, z]` (float64), unit-normalized
- **Euler joint angles**: degrees (float64), ISB convention `[aa, ie, fe]`
  - `aa` — abduction / adduction
  - `ie` — internal / external rotation
  - `fe` — flexion / extension
- **Velocities**: m/s (linear), rad/s (angular)
- **Accelerations**: m/s² (linear), rad/s² (angular)

---

## Common Patterns

### Detect new frames and read full body pose

```python
import time
import numpy as np
from xsens_mvn_robot import XsensWrapper

device = XsensWrapper(port=9763)
device.init()
device.start()

BODY_LINKS = [
    "pelvis", "l5", "l3", "t12", "t8", "neck", "head",
    "left_shoulder", "left_upper_arm", "left_forearm", "left_hand",
    "right_shoulder", "right_upper_arm", "right_forearm", "right_hand",
    "left_upper_leg", "left_lower_leg", "left_foot",
    "right_upper_leg", "right_lower_leg", "right_foot",
]

last_counter = -1
try:
    while True:
        counter = int(device.get_sample_counter())
        if counter == last_counter:
            time.sleep(0.001)
            continue
        last_counter = counter

        frame = {}
        for name in BODY_LINKS:
            pos = device.get_link_position(name)    # [x, y, z] in meters
            ori = device.get_link_orientation(name)  # [w, x, y, z] quaternion
            frame[name] = (pos, ori)

        # Use frame ...
finally:
    device.stop()
```

---

### Build a GMR-compatible human_frame

The `XsensToGMR` adapter (in `general_motion_retargeting/utils/xsens_vendor/`) wraps the above pattern into a ready-to-use format for the GMR retargeting pipeline:

```python
from general_motion_retargeting.utils.xsens_vendor.xsens_to_gmr_adapter import XsensToGMR

adapter = XsensToGMR(port=9763, verbose=True)
adapter.initialize()
adapter.start()

while True:
    human_frame = adapter.get_human_frame()
    if human_frame is None:
        continue
    # human_frame = {"Pelvis": (pos_array, quat_array), "Head": (...), ...}
    # Feed directly to GeneralMotionRetargeting
```

---

### Read center of mass for balance monitoring

```python
device.start()
while True:
    com = device.get_center_of_mass_position()   # [x, y, z] meters
    vel = device.get_center_of_mass_velocity()   # [x, y, z] m/s
    print(f"CoM: {com}  velocity: {vel}")
    time.sleep(0.01)
```

---

### Read raw IMU data from a specific tracker

```python
tracker_names = device.get_tracker_names()

for name in tracker_names:
    orientation = device.get_tracker_orientation(name)   # [w,x,y,z]
    free_acc    = device.get_tracker_free_acceleration(name)  # [x,y,z] m/s²
    gyro        = device.get_tracker_angular_velocity(name)   # [x,y,z] rad/s
    print(f"{name}: quat={orientation}  free_acc={free_acc}  gyro={gyro}")
```

---

### Measure actual streaming rate

```python
import time
from collections import deque

history = deque(maxlen=100)
last_counter = -1

while True:
    counter = int(device.get_sample_counter())
    if counter != last_counter:
        history.append(time.perf_counter())
        last_counter = counter

    if len(history) >= 2:
        dt = (history[-1] - history[0]) / (len(history) - 1)
        fps = 1.0 / dt if dt > 0 else 0.0
        print(f"FPS: {fps:.1f} Hz", end="\r")
    time.sleep(0.001)
```

---

## Error Handling

**Device fails to initialize** — MVN Studio is not streaming, wrong port, or firewall blocking UDP:

```python
if not device.init():
    print("No data received. Check that MVN Studio is streaming to the correct port.")
    # Verify: MVN Studio > Options > Network Streamer > UDP Port
```

**Accessing a link that does not exist** — returns zeros, does not raise an exception. Always validate link names with `get_link_names()` first:

```python
available = set(device.get_link_names())
if "left_hand" in available:
    pos = device.get_link_position("left_hand")
```

**Graceful shutdown on Ctrl+C:**

```python
import signal, sys

def handler(sig, frame):
    device.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)
```

**Port validation** — the constructor validates the port before creating the device:

```python
try:
    device = XsensWrapper(port=99999)  # raises ValueError
except ValueError as e:
    print(e)  # "Port 99999 is out of valid range (1-65535)"
```
