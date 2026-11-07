# Xsens MVN UDP/TCP Data Parsing Python Libraries

The code was written in C++ and wrapped with SWIG as a Python library. It parses
the Xsens MVN network streaming UDP/TCP data.

## Available wheels

Version **0.1.7**, for Python 3.8, 3.10, 3.11, 3.12 and 3.13:

| Platform | Wheel file |
|---|---|
| Linux x86_64 | `xsens_mvn_robot-0.1.7-cpXY-none-linux_x86_64.whl` |
| Linux aarch64 (e.g. NVIDIA Jetson) | `xsens_mvn_robot-0.1.7-cpXY-none-linux_aarch64.whl` |
| Windows x64 | `xsens_mvn_robot-0.1.7-cpXY-none-win_amd64.whl` |

`cpXY` is the Python version: `cp38`, `cp310`, `cp311`, `cp312` or `cp313`. The
only dependency is NumPy (1.x or 2.x), which pip installs automatically.

## Installation

Linux:

```bash
python3 --version        # pick the wheel that matches, e.g. 3.10 -> cp310
pip install xsens_mvn_robot-0.1.7-cp310-none-linux_x86_64.whl
```

Windows (PowerShell or Command Prompt):

```powershell
python --version         # pick the wheel that matches, e.g. 3.10 -> cp310
python -m pip install xsens_mvn_robot-0.1.7-cp310-none-win_amd64.whl
```

The Windows wheel is self-contained: no Visual C++ redistributable or other
DLLs are needed. It requires 64-bit Python from python.org, conda or uv.

Check the installation:

```bash
python -c "import xsens_mvn_robot as x; print(x.__version__)"
```

This prints `0.1.7`. If it prints another version, or the import fails, the
wheel was installed into a different Python environment than the one you are
running (e.g. a conda env vs. a project `.venv`).

### Using with uv

```toml
[project]
dependencies = [
    "xsens_mvn_robot; sys_platform == 'win32' and python_version == '3.10'",
]

[tool.uv.sources]
xsens_mvn_robot = { path = "wheels/xsens_mvn_robot-0.1.7-cp310-none-win_amd64.whl" }
```

The wheel only installs on the Python version in its name, so pin the project's
Python version to match (e.g. `uv python pin 3.10`).

### Windows firewall

On Windows, allow Python through the firewall for UDP port 9763 (or your
chosen port), or the data from MVN Studio will not arrive when it is streamed
from another PC.

## MVN Studio setup

In MVN Studio, open **Options → Network Streamer**, add a destination with the
IP address of the PC running your Python program, port **9763** and protocol
**UDP**, and enable at least **Position + Orientation (Quaternion)**.

## Documentation

- `API_REFERENCE.md`: full Python API reference.
- `realtime_motion_demo.py`: live example printing the body pose.
