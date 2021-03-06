#!/usr/bin/env python
import io
import socket
import struct
import cv2
import numpy as np
from PIL import Image as Img
from cv_bridge import CvBridge, CvBridgeError


import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
# all interfaces)
rospy.init_node('streamer')
server_socket = socket.socket()
server_socket.bind(('192.168.1.126', 7777))
server_socket.listen(0)

# Accept a single connection and make a file-like object out of it
connection = server_socket.accept()[0].makefile('rb')
image_pub = rospy.Publisher("/usb_cam/image_raw",Image, queue_size = 1)
bridge = CvBridge()

try:
   while not rospy.is_shutdown():
       # Read the length of the image as a 32-bit unsigned int. If the
       # length is zero, quit the loop
       image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
       if not image_len:
           break
       # Construct a stream to hold the image data and read the image
       # data from the connection
       image_stream = io.BytesIO()
       image_stream.write(connection.read(image_len))
       # Rewind the stream, open it as an image with opencv and do some
       # processing on it
       image_stream.seek(0)
       image = Img.open(image_stream)

       data = np.fromstring(image_stream.getvalue(), dtype=np.uint8)
       imagedisp = cv2.imdecode(data, 1)
       try:
           image_pub.publish(bridge.cv2_to_imgmsg(imagedisp, "bgr8"))
       except CvBridgeError as e:
           print(e)

       # cv2.imshow("Frame",imagedisp)
       # cv2.waitKey(1)  #imshow will not output an image if you do not use waitKey
       # cv2.destroyAllWindows() #cleanup windows 
finally:
   connection.close()
   server_socket.close()
