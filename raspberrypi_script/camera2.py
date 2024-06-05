#!/usr/bin/python3

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:8000
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import io
import logging
import socketserver
from http import server
from threading import Condition
import requests
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
REMOTE_SERVER_URL = 'http://your_remote_server.com/upload'
BOUNDARY = 'frameboundary'
PAGE = """\
<html>
<head>
<title>picamera2 MJPEG streaming demo</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" height="480" />
</body>
</html>
"""


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
		

            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

capture_and_send_frame()
#picam2 = Picamera2()
#picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
#output = StreamingOutput()
#picam2.start_recording(JpegEncoder(), FileOutput(output))

#try:
#    address = ('', 8000)
#    server = StreamingServer(address, StreamingHandler)
#    server.serve_forever()
#finally:
#    picam2.stop_recording()


def capture_and_send_frame():
	url = 'http://httpbin.org/post'
	picam2 = Picamera2()
	picam2.start_and_capture_file("test.jpg")
	files = {'file': open('test.jpg', 'rb')}

	requests.post(url, files=files)
	print('test image sent')
	# Prepare the HTTP POST request
	#headers = {'Content-Type': 'multipart/form-data; boundary=' + BOUNDARY}
	#files = {'file': ('frame.jpg', stream.read(), 'image/jpeg')}
	#data = '--' + BOUNDARY + '\r\nContent-Disposition: form-data; name="file"; filename="frame.jpg"\r\nContent-Type: image/jpeg\r\n\r\n'

	# Send the frame to the remote server
	#try:
	#   response = requests.post(REMOTE_SERVER_URL, headers=headers, data=data, files=files)
	#    print('Frame sent successfully. Server response:', response.text)
	#except requests.exceptions.RequestException as e:
	#    print('Error sending frame:', e)
