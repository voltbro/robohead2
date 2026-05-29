# robohead_web

FastAPI-based web control panel for Robohead.

## Runtime architecture

- `server.py` owns the FastAPI/uvicorn application, HTTP routes, MJPEG stream, and WebSocket endpoints.
- `ros_bridge.py` owns the ROS2 node, publishers, subscribers, service clients, and hot-path frame/audio buffers.
- `image_utils.py` contains fast ROS Image conversion helpers that avoid `cv_bridge` in the web process.
- `static/` contains the browser UI.

## Web endpoints

- `/` serves the control panel.
- `/camera.mjpg` streams the robot camera as low-latency MJPEG.
- `/ws` carries controls, drawing commands, status updates, and optional client-camera binary frames.
- `/audio_ws` streams ReSpeaker PCM audio to the browser.
- `/audio_in_ws` accepts browser microphone PCM and publishes it to `/robohead/media_driver/audio_stream`.

## Notes

The display canvas is kept server-side as a 1080x1080 RGB numpy array. Drawing publishes rate-limited frames while the user draws and flushes the final canvas frame at stroke end for stable display output.
