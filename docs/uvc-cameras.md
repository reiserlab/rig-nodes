# USB (UVC) cameras: `rig-uvc-camera`

A USB camera for viewing and recording alongside a rig graph. ffmpeg reads the camera through
DirectShow and `rig_nodes.uvc_camera_node` publishes what comes out of its pipe: the camera's own
JPEGs as `sensor_msgs/CompressedImage` (default), or 8-bit luma as `sensor_msgs/Image`. No OpenCV,
no `cv_bridge`; `usb_cam` is V4L2 and does not exist on Windows.

```
pixi run uvc-camera -- --list                                   # cameras on this PC
pixi run uvc-camera -- --device "Arducam OV9281 USB Camera" --list   # its formats, sizes, rate ranges
pixi run uvc-camera -- --device "Arducam OV9281 USB Camera" --src ov9281 --size 1280x800 --fps 120
pixi run uvc-camera -- --device "Arducam ezBOX VGA GS Mono" --src ezbox --size 640x480 --fps 200
```

Needs an `ffmpeg` executable: `--ffmpeg`, `$RIG_FFMPEG`, `PATH`, then `C:\msys64\ucrt64\bin`.
One node per camera, each with its own `--src` (CONTRACT §2: one node per `<src>`).

## What was measured (ww18, 2026-11-10)

Two Arducam global-shutter monochrome cameras on the USB 2.0 side of one hub, Microsoft's inbox
UVC driver, frames counted against the host clock over 6 to 8 s.

| Camera | Mode asked | Delivered |
|---|---|---|
| ezBOX VGA GS Mono (0C40:0632) | MJPEG 640x480 @30 / 60 / 200 | 30 / 61.4 / 200 fps |
| | MJPEG 640x480 @90, @100, @120 | 101 fps each |
| | MJPEG 640x480 @150 | 200 fps |
| | MJPEG 320x240 @240, 160x120 @240 | 438, 881 fps |
| | YUYV 640x480 @60, 320x240 @160, 160x120 @200 | 61.5, 162, 402 fps |
| OV9281 USB Camera (0C45:6366) | MJPEG 1280x800 / 1280x720 / 800x600 / 640x480 / 320x240 @120 | 120.3 to 120.8 fps |
| | MJPEG @100 | 101 fps |
| | MJPEG @110 | 120.7 fps |
| | MJPEG below 100, YUYV above 10 | refused: the device does not open |
| | YUYV 1280x800 @10 | 10.7 fps |

Through the node, both cameras at once (Lyrical, rmw_zenoh, loopback): `/ezbox/image/compressed`
200 Hz at 26 KB per frame and `/ov9281/image/compressed` 121 Hz at 43 KB per frame, no malformed
JPEG; `/ezbox/image_raw` 61 Hz.

## What to know

- **The rate you get is not always the rate you ask for, and nothing says so.** The camera picks a
  rate it has; ffmpeg reports the one you asked for. The node's `/<src>/status` carries
  `fps_measured`, and the node warns once when it is more than 5 % off.
- **Driver timestamps are synthetic**: they step by exactly the nominal interval, whatever arrives.
  They cannot show drops or the real rate, so the node does not use them.
- **Stamps are host arrival times, in bursts.** ffmpeg's DirectShow reader hands frames over about
  every 16 ms on Windows. Use `header.stamp` for ordering and rough alignment, not as exposure time.
- **OV9281: uncompressed only at 10 fps.** Anything faster is MJPEG, with JPEG artefacts.
- **Uncompressed 640x480 @60 is 302 Mbit/s** on a USB 2.0 link; MJPEG at 200 fps is 50 Mbit/s.
- **Gray mode needs `-fps_mode passthrough`** (the node sets it): otherwise ffmpeg resamples to the
  nominal rate and drops the extra frames of a camera that runs a little fast.

## Not covered

Exposure and gain (ffmpeg's DirectShow input cannot set them; the camera keeps its defaults),
exposure-accurate timestamps, Linux and macOS, runs longer than a few seconds, and a recorded bag
with these topics.
