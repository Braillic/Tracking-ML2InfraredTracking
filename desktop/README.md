# Magic Leap 2 depth receiver

The Unity app is the TCP server and the PC initiates the connection. Frames are
sent as their original `float32` depth values in metres. The server keeps only
the newest waiting frame, so a slow connection does not stall the Unity sensor
loop or build an unbounded queue.

## Unity / Magic Leap setup

The demo scene's `IRToolManager` now has a **Depth Frame Tcp Server** component.
Its defaults are:

- Bind address: `0.0.0.0` (accept Wi-Fi and ADB-forwarded connections)
- Port: `50777`
- Maximum rate: `15` frames/s (`0` means every sensor frame)
- On-screen status: enabled

The fallback overlay displays the Magic Leap Wi-Fi address and port. For a
reliable XR-world display, create a world-space Canvas with a TextMeshPro
`Text - TextMeshPro` component and assign its `TMP_Text` to **Status Text**. You
may assign a second TMP component to **Sensor Info Text** for the live pose and
intrinsics. If that field is empty, sensor information is appended to Status
Text automatically. The fallback overlay can then be disabled.

`ML2DepthRawStream` also adds a server with these defaults at runtime if none is
assigned, which makes the streaming change usable from other scenes without a
scene edit.

## PC setup

Create a virtual environment and install the receiver dependencies:

```powershell
cd desktop
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

### Connect over Wi-Fi

Put the PC and Magic Leap on a mutually reachable network. Use the address shown
in the headset/app:

```powershell
python depth_stream_receiver.py --host 192.168.1.123 --port 50777
```

If this times out, verify that client isolation is disabled on the Wi-Fi network
and that the PC firewall permits the Python process. Android does not need a
runtime permission prompt for `INTERNET`; the permission is included in the app
manifest.

### Connect over USB with ADB forwarding

With the headset visible in `adb devices`, forward the PC port to the listening
port inside the Magic Leap:

```powershell
adb forward tcp:50777 tcp:50777
python depth_stream_receiver.py --host 127.0.0.1 --port 50777
```

To remove the forwarding rule later:

```powershell
adb forward --remove tcp:50777
```

If the Unity port is changed, use the same port in both arguments to `adb
forward` and in the Python command. `0.0.0.0` is the recommended Unity bind
address; `127.0.0.1` may be selected to allow only ADB-forwarded access.

Press `Q` or Escape to quit the viewer. Press `S` to save the current unmodified
depth matrix as `.npy`; use `--save-dir PATH` to select its directory. The
default `unity-raw` view matches `DepthRawMat`: grayscale using the shader's
`_RawMin=5` and `_RawMax=3000`. Override them with `--raw-min` and `--raw-max`
if the Unity material changes. It also applies Unity's Linear-to-sRGB display
conversion; use `--unity-color-space gamma` if the project switches to Gamma.
The earlier inverted Turbo visualization remains
available with `--view turbo --near 0.2 --far 5`. Visualization is display-only;
the received NumPy array is never modified.
The receiver automatically reconnects if a deployment or app restart leaves a
stale TCP connection; adjust this with `--frame-timeout` if needed.

## Wire protocol

Each frame is a 72-byte little-endian header (`<4sHHIIIIQd3f4fI`) followed by
`width * height` little-endian `float32` values and, when present, a fixed
88-byte numeric intrinsics block (`<11d`):

1. Magic `ML2D`
2. Protocol version (`3`) and header size (`72`)
3. Width, height, pixel format (`1` = FLOAT32 metres), payload byte count
4. Frame number and UTC Unix timestamp in seconds
5. Sensor position `x,y,z` in metres and rotation quaternion `x,y,z,w`, both in
   the Unity world coordinate system
6. Numeric intrinsics byte count (`0` or `88`)

The sensor pose is synchronized with every depth frame. Intrinsics are sent once
per TCP connection (on the first frame after metadata becomes available), then
the intrinsics byte count is zero for later frames. In Python, `receive_frame()`
returns a `DepthFrame` containing `depth_metres`, `sensor_position`,
`sensor_rotation`, and the optional one-time `CameraIntrinsics`. The numeric
intrinsics order is `fx, fy, cx, cy, fov_x, fov_y, k1, k2, p1, p2, k3`.
`CameraIntrinsics.camera_matrix` and `.distortion_coefficients` are immediately
usable with OpenCV.
