import picamera as picam
import os
import time

cam = picam.PiCamera()
# cam.resolution = (640, 480)
cam.resolution = (1280, 720)


# https://picamera.readthedocs.io/en/release-1.13/api_camera.html#picamera.PiCamera.start_recording
video_folder = "recordings/"
video_file = "payload_recording"
video_file_suffix = 0
if not os.path.exists(video_folder):
    os.makedirs(video_folder)
while os.path.isfile(video_folder + video_file + str(video_file_suffix) + ".h264"):
    video_file_suffix += 1

cam.start_recording(video_folder + video_file +
                    str(video_file_suffix) + ".h264", format="h264")
time.sleep(1)
print("Started recording to file: " + video_folder +
      video_file + str(video_file_suffix) + ".h264 ...")

# 60s recording
count = 0
while(count < 10):
    count += 1
    time.sleep(1)
cam.stop_recording()
print("Finished recording.")
