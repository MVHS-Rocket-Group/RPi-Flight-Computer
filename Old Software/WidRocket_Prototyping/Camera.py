import picamera

camera = picamera.PiCamera()
camera.resolution = (640,480)
camera.start_recording('videotest.h264')
camera.wait_recording(60)
camera.stop_recording()
print("finished recording")