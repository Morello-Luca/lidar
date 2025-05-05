# Lidar Motion Tracker Documentation

## Overview

This Python project connects to an RPLidar device, processes 2D scan data, and detects whether any object enters a predefined "danger zone" on a 2D plane. It provides a real-time visualization of the Lidar points using PyQtGraph, applies a DBSCAN clustering algorithm, and raises an alert if a detected object is inside the defined rectangular area.

---

## Block Diagram (Textual)

```
[Start]
   |
   V
[Connect to Lidar]
   |
   V
[Read Lidar Scan Data]
   |
   V
[Preprocess Scan]
   |
   V
[Convert Polar to Cartesian Coordinates]
   |
   V
[Detect Clusters using DBSCAN]
   |
   V
[Check if any cluster is in the danger zone]
   |
   V
[Visualize points]
   |
   V
[If object in zone -> Print alarm]
   |
   V
[Repeat]
```

---

## Key Components

### 1. MotionTracker class

Handles all data processing:
- **Preprocessing**: Filters points with low quality or zero distance.
- **Conversion**: Converts polar coordinates (angle, distance) to Cartesian (x, y).
- **Smoothing**: (Optional) Median filtering over the latest scans.
- **Clustering**: Uses DBSCAN to find clusters in the scan data.
- **Danger Zone Detection**: Checks if any cluster falls inside a rectangle defined by user parameters.

### 2. LidarApp class

Handles the GUI and data acquisition:
- Initializes a Qt window with a PyQtGraph plot.
- Connects to the RPLidar device on `COM5` (can be changed).
- Reads new scans and processes them using the `MotionTracker`.
- Displays clusters in real-time on a 2D plot.
- Alerts the user if a cluster is inside the danger zone.

---

## Key Parameters

```python
PORT_NAME = 'COM5'
BAUDRATE = 115200
TIMEOUT = 3
DMAX = 4000

X_MIN, X_MAX = 500, 1500
Y_MIN, Y_MAX = -500, 500

QUALITY_MIN = 3
CLUSTER_EPS = 200
CLUSTER_MIN_POINTS = 6
FRAME_HISTORY = 3
```

- `PORT_NAME`: Serial port of the LiDAR.
- `DMAX`: Maximum distance displayed on the plot.
- `X_MIN`, `X_MAX`, `Y_MIN`, `Y_MAX`: Rectangle defining the danger zone in millimeters.
- `QUALITY_MIN`: Ignore measurements below this quality.
- `CLUSTER_EPS`: Max distance between points in the same cluster.
- `CLUSTER_MIN_POINTS`: Minimum number of points for a cluster to be considered valid.
- `FRAME_HISTORY`: Number of frames for smoothing (currently disabled).

---

## How to Run

1. **Install Dependencies**:
   ```bash
   pip install numpy pyqtgraph rplidar scikit-learn
   ```

2. **Connect your RPLidar** via USB and find its COM port.

3. **Run the script**:
   ```bash
   python scriptname.py
   ```

4. **Watch the GUI** to see live point cloud. When a cluster enters the danger zone, a message is printed.

---

## How to Modify

- **Change the danger zone**:
  Modify `X_MIN`, `X_MAX`, `Y_MIN`, `Y_MAX` at the top.

- **Filter by quality**:
  Adjust `QUALITY_MIN` to filter out noisy points.

- **Change clustering behavior**:
  Tweak `CLUSTER_EPS` and `CLUSTER_MIN_POINTS`.

- **Change the smoothing**:
  Re-enable the `update_frame()` function and pass its output to clustering.

---

## Example Output (Console)

```
Connessione al LiDAR...
LiDAR connesso.
Sistema avviato. Ctrl+C per uscire.
Scansione ricevuta: [...]
Punti grezzi: [[x1, y1], [x2, y2], ...]
Punti estratti: [[x1, y1], [x2, y2], ...]
!!! ALLARME: oggetto in movimento nella zona pericolosa !!!
```

---

## Notes

- This code assumes a Windows environment with the RPLidar connected to `COM5`. Adjust as needed.
- DBSCAN parameters may need tuning depending on the environment (e.g., indoor vs outdoor, noise levels).
