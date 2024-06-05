import io
import time
import requests
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

# Replace these with your server details
REMOTE_SERVER_URL = 'http://your_remote_server.com/upload'
BOUNDARY = 'frameboundary'

def capture_and_send_frame():
    with Picamera2() as camera:
        stream = io.BytesIO()
        
        # Capture an MJPEG frame
        camera.start_preview()
        time.sleep(2)  # Allow camera to warm up
        camera.capture(stream, format='jpeg')
        stream.seek(0)

        # Prepare the HTTP POST request
        headers = {'Content-Type': 'multipart/form-data; boundary=' + BOUNDARY}
        files = {'file': ('frame.jpg', stream.read(), 'image/jpeg')}
        data = '--' + BOUNDARY + '\r\nContent-Disposition: form-data; name="file"; filename="frame.jpg"\r\nContent-Type: image/jpeg\r\n\r\n'

        # Send the frame to the remote server
        try:
            response = requests.post(REMOTE_SERVER_URL, headers=headers, data=data, files=files)
            print('Frame sent successfully. Server response:', response.text)
        except requests.exceptions.RequestException as e:
            print('Error sending frame:', e)

if __name__ == "__main__":
    capture_and_send_frame()
