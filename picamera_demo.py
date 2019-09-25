import picamera
import os

camera = picamera.PiCamera()
# camera.resolution = (640, 480)
camera.resolution = (1280, 720)

# https://picamera.readthedocs.io/en/release-1.13/api_camera.html#picamera.PiCamera.start_recording
recording_suffix = 0
while os.path.isfile("recordings/" + recording_suffix + ".mp4"):
    recording_suffix += 1

print("Started recording...")
camera.start_recording(
    "recordings/" + recording_suffix + ".mp4", format="h264")
# 60s recording
camera.wait_recording(60)
camera.stop_recording()
print("Finished recording.")
