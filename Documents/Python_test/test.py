from picamera import PiCamera
from time import sleep

# Initialize the camera
camera = PiCamera()

# Wait for the camera to warm up
sleep(2)

# Capture an image
camera.capture('test_image.jpg')

# Clean up
camera.close()
