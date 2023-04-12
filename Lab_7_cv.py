import math
import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import numpy as np
import cv2 as cv
# from picamera2 import Picamera2
# picam2 = Picamera2()
# picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
# picam2.start()

positions = {}
rotations = {}

# Record Coordinates--------------------------------
Coordinate_X = []
Coordinate_Y = []

# When detect duck store the coordinates of robot and the rotation
Coordinate_X_detection = []
Coordinate_Y_detection = []
rotation_detection = []
u = []
v = []
Sb = []
Global_Duck_X = []
Global_Duck_Y = []
# End here-------------------------

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz


def compute_object_position(x, y, alpha, D, theta):
    alpha = math.radians(alpha)
    R = np.array([[math.cos(theta), -math.sin(theta), x],[math.sin(theta), math.cos(theta), y],[0, 0, 1]])
    target_local = np.array([[D * math.cos(alpha)], [D * math.sin(alpha)], [1]])
    target_global = np.matmul(R, target_local)
    Duck_x_global = target_global[0][0]
    Duck_y_global = target_global[1][0]
    return [Duck_x_global, Duck_y_global]


# IP addresses
IP_ADDRESS = "192.168.0.206"
clientAddress = "192.168.0.41"
optitrackServerAddress = "192.168.0.4"
robot_id = 206

# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

# This will create a new NatNet client
streaming_client = NatNetClient()
streaming_client.set_client_address(clientAddress)
streaming_client.set_server_address(optitrackServerAddress)
streaming_client.set_use_multicast(True)
# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streaming_client.rigid_body_listener = receive_rigid_body_frame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
is_running = streaming_client.run()

while True:
    if robot_id in positions:
        Initial_Position_X = positions[robot_id][0]
        Initial_Position_Y = positions[robot_id][1]
        break

t = 0

try:
    while True:
        if robot_id in positions and t <= 1.5:
            cap = cv.VideoCapture('http://192.168.0.206:3000/stream.mjpg')
            if not cap.isOpened():
                print("Error opening video stream or file")
            exit()
            # Camera Part
            hsv = cv.cvtColor(cap, cv.COLOR_BGR2HSV)
            # Threshold of blue in HSV space
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            # preparing the mask to overlay
            mask = cv.inRange(hsv, lower_yellow, upper_yellow)
            # cv.imshow("m", mask)
            # The black region in the mask has the value of 0,
            # so when multiplied with original image removes all non-blue regions
            result = cv.bitwise_and(cap, cap, mask=mask)
            gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
            gray = cv.medianBlur(gray, 5)
            # gray = cv.GaussianBlur(gray,(7,7),2)
            cv.imshow("G", gray)
            params = cv.SimpleBlobDetector_Params()
            # Change thresholds
            params.minThreshold = 50
            params.maxThreshold = 150
            # Filter by Area.
            params.filterByArea = True
            params.minArea = 400
            params.maxArea = 900000
            # color filter
            params.filterByColor = True
            params.blobColor = 255
            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.3
            # Filter by Convexity
            params.filterByConvexity = False
            params.minConvexity = 0.5
            # Filter by Inertia
            params.filterByInertia = False
            params.minInertiaRatio = 0.01
            # Create a detector with the parameters
            detector = cv.SimpleBlobDetector_create(params)
            # set up keypoints
            keypoints = detector.detect(gray)
            print(keypoints)
            im_with_keypoints = cv.drawKeypoints(gray, keypoints, np.array([]), (0, 0, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # Show keypoints
            cv.imshow("Keypoints", im_with_keypoints)
            # record and print key points
            for keyPoint in keypoints:
                if bool(keyPoint):
                    x = keyPoint.pt[0]
                    y = keyPoint.pt[1]
                    s = keyPoint.size
                    u = u.append(x)
                    v = v.append(y)
                    Sb = Sb.append(s)
                    D = 324.519 - s*1.2413
                    theta = 28.41 - x*0.1
                    Coordinate_X_detection = Coordinate_X_detection.append(positions[robot_id][0])
                    Coordinate_Y_detection = Coordinate_Y_detection.append(positions[robot_id][1])
                    rotation_detection = rotation_detection.append(rotations[robot_id])
                    Global_Duck_X = Global_Duck_X.append(compute_object_position(positions[robot_id][0], positions[robot_id][1], rotations[robot_id], D, theta)[0])
                    Global_Duck_Y = Global_Duck_Y.append(compute_object_position(positions[robot_id][0], positions[robot_id][1], rotations[robot_id], D, theta)[1])
                    print(x, y, s)
                else:
                    pass
            # Our operations on the frame come here
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
            t += 0.1
        else:
            break

except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    streaming_client.shutdown()

# Terminate the Robot
command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
streaming_client.shutdown()
# Close the connection
s.shutdown(2)
s.close()