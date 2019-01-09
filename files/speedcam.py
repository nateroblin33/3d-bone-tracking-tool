###############################################
##     Three-Point Bone Tracking Script      ##
###############################################

# Import packages
from __future__ import print_function
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import pyrealsense2 as rs
from datetime import datetime
import math
import sys
from io import BytesIO
import pytest
from plyfile import (PlyData, PlyElement, make2d, PlyHeaderParseError, PlyElementParseError, PlyProperty)

##############################################
##                                          ##
##             Color Code Chart             ##
##                                          ##
## Black = K (1)                            ##
## White = I (2)                            ##
##                                          ##
## Orange = O                               ##
## Yellow = Y                               ##
## Grellow = W                              ##
## Green = G                                ##
## Turquoise = T                            ##
## Blue = B                                 ##
## Purple = U                               ##
## Pink = P                                 ##
## Red = R                                  ##
##                                          ##
## NOTE: Currently unsupported colors       ##
##       also have codes designated         ##
##                                          ##
## Brown = N                                ##
## Gray = A                                 ##
##                                          ##
##############################################

# Construct the argument parse and parse the arguments
# Gives option to use "python mbts.py --video [FILENAME.EXTENSION]" in Terminal
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=4, help="max buffer size")
args = vars(ap.parse_args())

# Sets up streaming settings for Intel RealSense D435
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Starts stream with configured settings (DOES NOT DISPLAY THE STREAM)
profile = pipeline.start(config)

# Define bounds for each color supported in HSV/HSB values
# NOTE: HSV/HSB is NOT the same as HSL
# NOTE: HSV/HSB values typically range from (0,0%,0%) to (360,100%,100%), but they are scaled here to be from (0,0,0) to (180,255,255)
#       Conversion from real HSV/HSB to this scale can be done by halving the Hue and then taking the appropriate percentage (%) of 255 for Saturation and Value/Brightness

# BLACK (k)
blackLower = (0, 0, 0)
blackUpper = (180, 255, 49)

# WHITE (i)
whiteLower = (0, 0, 240)
whiteUpper = (255, 20, 255)



# GREEN (g)
greenLower = (61, 86, 50)
greenUpper = (80, 255, 255)

# BLUE (b)
blueLower = (101, 86, 50)
blueUpper = (115, 255, 255)

# YELLOW (y)
yellowLower = (16, 108, 50)
yellowUpper = (40, 255, 255)

# RED (r)
# NOTE: Red wraps from upper end of Hue range to lower end (Hue 360 = Hue 0 in standard HSV/HSB, Hue 180 = Hue 0 in this scale)
redLower = (171, 86, 50)
redUpper = (180, 255, 255)
redLower2 = (0, 100, 50)
redUpper2 = (6, 255, 255)

# ORANGE (o)
orangeLower = (7, 100, 50)
orangeUpper = (15, 255, 255)

# PINK (p)
pinkLower = (155, 30, 50)
pinkUpper = (170, 255, 255)

# PURPLE (u)
purpleLower = (116, 51, 40)
purpleUpper = (154, 255, 255)

# TURQUOISE (t)
turquoiseLower = (81, 100, 60)
turquoiseUpper = (100, 255, 255)

# GRELLOW (w)
grellowLower = (41, 100, 50)
grellowUpper = (60, 255, 255)

# Unsupported color ranges (INTERFERE WITH OTHER COLORS)
"""# GRAY (a)
#grayLower = (0, 0, 64)
#grayUpper = (180, 20, 192)

# BROWN (n)
#brownLower = (0, 50, 50)
#brownUpper = (40, 107, 49)"""

# Allows Intel RealSense D435 stream to be accessed if no video file (will not allow video file option from line 47 unless this is changed to "0")
vsbool = 1

# Initialize a list of tracked points for each color
ptsk = deque(maxlen=args["buffer"])

ptsi = deque(maxlen=args["buffer"])

ptsg1 = deque(maxlen=args["buffer"])
ptsg2 = deque(maxlen=args["buffer"])

ptsb1 = deque(maxlen=args["buffer"])
ptsb2 = deque(maxlen=args["buffer"])

ptsy1 = deque(maxlen=args["buffer"])
ptsy2 = deque(maxlen=args["buffer"])

ptsr1 = deque(maxlen=args["buffer"])
ptsr2 = deque(maxlen=args["buffer"])

ptso1 = deque(maxlen=args["buffer"])
ptso2 = deque(maxlen=args["buffer"])

ptsp1 = deque(maxlen=args["buffer"])
ptsp2 = deque(maxlen=args["buffer"])

ptsu1 = deque(maxlen=args["buffer"])
ptsu2 = deque(maxlen=args["buffer"])

ptst1 = deque(maxlen=args["buffer"])
ptst2 = deque(maxlen=args["buffer"])

ptsw1 = deque(maxlen=args["buffer"])
ptsw2 = deque(maxlen=args["buffer"])

#ptsa = deque(maxlen=args["buffer"])
#ptsn = deque(maxlen=args["buffer"])

# Starts coordinates at (0,0) for each color
xk = yk = xi = yi = 0
xg1 = yg1 = xb1 = yb1 = xy1 = yy1 = xr1 = yr1 = xo1 = yo1 = xp1 = yp1 = xu1 = yu1 = xt1 = yt1 = xw1 = yw1 = 0
xg2 = yg2 = xb2 = yb2 = xy2 = yy2 = xr2 = yr2 = xo2 = yo2 = xp2 = yp2 = xu2 = yu2 = xt2 = yt2 = xw2 = yw2 = 0
#xa = ya = xn = yn = 0

# Counters to determine which frames while have Depth/Coord printed
# ptrue is the counter used for frames in which there is something to track
# pfalse is the counter used for frames in which there is nothing to track
ptrue = 1
pfalse = 1

# Counter to have first 5 saved frames used for Roll/Yaw/Pitch baseline values, and arrays for actual baseline and temporary baseline calculation
ryp = 1
lunateBaseline = [0,0,0]
lunateBasecalc = [0,0,0]
capitateBaseline = [0,0,0]
capitateBasecalc = [0,0,0]
hamateBaseline = [0,0,0]
hamateBasecalc = [0,0,0]
scaphoidBaseline = [0,0,0]
scaphoidBasecalc = [0,0,0]
trapezoidBaseline = [0,0,0]
trapezoidBasecalc = [0,0,0]
triquetrumBaseline = [0,0,0]
triquetrumBasecalc = [0,0,0]

# Counter to name Point Cloud files for each frame
framePC = 1

# Boolean to determine if a frame is bad (missing at least one dot)
badframe = 0

# If a video path was not supplied, grab the reference to the Intel RealSense D435 stream
if not args.get("video", False):
    vs = VideoStream(src=0).start()
 
    # Otherwise, grab a reference to the video file
else:
    vsbool = 1
    
    # OPTIONAL code for using Default Webcam/Facetime Camera
    # NOTE: If used, comment out the later "vs = color_image" in line ################################################134 (this line number has changed and is no longer correct)
    #vs = cv2.VideoCapture(args["video"])
    #vs = cv2.VideoCapture(-1)
 
    # Allow the camera or video file to warm up
time.sleep(2.0)

# Print start time for log
dt = datetime.now()
print("Time Started: " + str(dt))

# Aligns streams
align_to = rs.stream.color
align = rs.align(align_to)

# Function to normalize vector
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

# Function to calculate Roll
def roll(p1, p2, p3, baseline):
    vecx = [(p1[0]+p2[0])/2 - p3[0], (p1[1]+p2[1])/2 - p3[1], (p1[2]+p2[2])/2 - p3[2]]
    v1 = [p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]]
    v2 = [p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]]
    X = normalize(vecx)
    vecz = [(v1[1]*v2[2])-(v1[2]*v2[1]), (v1[2]*v2[0])-(v1[0]*v2[2]), (v1[0]*v2[1])-(v1[1]*v2[0])]
    Z = normalize(vecz)
    Y = [(Z[1]*X[2])-(Z[2]*X[1]), (Z[2]*X[0])-(Z[0]*X[2]), (Z[0]*X[1])-(Z[1]*X[0])]
    roll = math.atan2(-Y[0], X[0]) * 90
    # Baseline correction
    print(roll)
    print(baseline[0])
    roll = roll - baseline[0]
    print(roll)
    print("")
    return roll

# Function to calculate Yaw
def yaw(p1, p2, p3, baseline):
    vecx = [(p1[0]+p2[0])/2 - p3[0], (p1[1]+p2[1])/2 - p3[1], (p1[2]+p2[2])/2 - p3[2]] 
    v1 = [p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]]
    v2 = [p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]]
    X = normalize(vecx)
    vecz = [(v1[1]*v2[2])-(v1[2]*v2[1]), (v1[2]*v2[0])-(v1[0]*v2[2]), (v1[0]*v2[1])-(v1[1]*v2[0])]
    Z = normalize(vecz)
    Y = [(Z[1]*X[2])-(Z[2]*X[1]), (Z[2]*X[0])-(Z[0]*X[2]), (Z[0]*X[1])-(Z[1]*X[0])]
    yaw = math.asin(Z[0]) * 90 * 3 / 2
    # Baseline correction
    yaw = yaw - baseline[1]
    return yaw

# Function to calculate Pitch
def pitch(p1, p2, p3, baseline):
    vecx = [(p1[0]+p2[0])/2 - p3[0], (p1[1]+p2[1])/2 - p3[1], (p1[2]+p2[2])/2 - p3[2]]
    v1 = [p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]]
    v2 = [p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]]
    X = normalize(vecx)
    vecz = [(v1[1]*v2[2])-(v1[2]*v2[1]), (v1[2]*v2[0])-(v1[0]*v2[2]), (v1[0]*v2[1])-(v1[1]*v2[0])]
    Z = normalize(vecz)
    Y = [(Z[1]*X[2])-(Z[2]*X[1]), (Z[2]*X[0])-(Z[0]*X[2]), (Z[0]*X[1])-(Z[1]*X[0])]
    pitch = math.atan2(-Z[1], Z[2]) * 90 * 3 / 2
    # Baseline correction
    pitch = pitch - baseline[2]
    return pitch

# Function to calculate Euclidean Distance between two points
def euclideanDist(p1, p2):
    dist = np.sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2))
    # Scale correction
    dist *= 2
    dist /= 3
    # * 39.3701 (for inches)
    return dist

# Function to export Point Cloud files from data
def createPC(pointg1, pointg2, pointb1, pointb2, pointy1, pointy2, pointr1, pointr2, pointo1, pointo2, pointp1, pointp2, pointu1, pointu2, pointt1, pointt2, pointw1, pointw2, bone1FileName, bone2FileName, bone3FileName, bone4FileName, bone5FileName, bone6FileName):
    # Plane #1 is defined by grellow, pink, and blue (arbitrary) on black
    vertex1 = np.array([(-pointw1[2]*2/3, pointw1[0]*2/3, -pointw1[1]*2/3),
                  (-pointp1[2]*2/3, pointp1[0]*2/3, -pointp1[1]*2/3),
                  (-pointb1[2]*2/3, pointb1[0]*2/3, -pointb1[1]*2/3),
                  (((-pointw1[2]*2/3)+(-pointp1[2]*2/3)+(-pointb1[2]*2/3))/3 - 0.001,((pointw1[0]*2/3)+(pointp1[0]*2/3)+(pointb1[0]*2/3))/3,((-pointw1[1]*2/3)+(-pointp1[1]*2/3)+(-pointb1[1]*2/3))/3)],
                 dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    face1 = np.array([([0, 1, 2], 0, 0, 0),#black face
                ([0, 2, 3], 213,   255,   0),
                ([0, 1, 3],   243, 84,   255),
                ([1, 2, 3],   0,   0, 255)],
               dtype=[('vertex_indices', 'i4', (3,)),
                      ('red', 'u1'), ('green', 'u1'),
                      ('blue', 'u1')])

    # Plane #2 is defined by grellow, pink, and blue (arbitrary) on white
    vertex2 = np.array([(-pointw2[2]*2/3, pointw2[0]*2/3, -pointw2[1]*2/3),
                  (-pointp2[2]*2/3, pointp2[0]*2/3, -pointp2[1]*2/3),
                  (-pointb2[2]*2/3, pointb2[0]*2/3, -pointb2[1]*2/3),
                  (((-pointw2[2]*2/3)+(-pointp2[2]*2/3)+(-pointb2[2]*2/3))/3 - 0.001,((pointw2[0]*2/3)+(pointp2[0]*2/3)+(pointb2[0]*2/3))/3,((-pointw2[1]*2/3)+(-pointp2[1]*2/3)+(-pointb2[1]*2/3))/3)],
                 dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    face2 = np.array([([0, 1, 2], 255, 255, 255),#white face
                ([0, 2, 3], 213,   255,   0),
                ([0, 1, 3],   243, 84,   255),
                ([1, 2, 3],   0,   0, 255)],
               dtype=[('vertex_indices', 'i4', (3,)),
                      ('red', 'u1'), ('green', 'u1'),
                      ('blue', 'u1')])

    # Plane #3 is defined by turquoise, red, and orange (arbitrary) on white
    vertex3 = np.array([(-pointt2[2]*2/3, pointt2[0]*2/3, -pointt2[1]*2/3),
                  (-pointr2[2]*2/3, pointr2[0]*2/3, -pointr2[1]*2/3),
                  (-pointo2[2]*2/3, pointo2[0]*2/3, -pointo2[1]*2/3),
                  (((-pointt2[2]*2/3)+(-pointr2[2]*2/3)+(-pointo2[2]*2/3))/3 - 0.001,((pointt2[0]*2/3)+(pointr2[0]*2/3)+(pointo2[0]*2/3))/3,((-pointt2[1]*2/3)+(-pointr2[1]*2/3)+(-pointo2[1]*2/3))/3)],
                 dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    face3 = np.array([([0, 1, 2], 255, 255, 255),#white face
                ([0, 2, 3], 0,   255,   255),
                ([0, 1, 3],   255, 0,   0),
                ([1, 2, 3],   255,   140, 0)],
               dtype=[('vertex_indices', 'i4', (3,)),
                      ('red', 'u1'), ('green', 'u1'),
                      ('blue', 'u1')])

    # Plane #4 is defined by purple, yellow, and green (arbitrary) on black
    vertex4 = np.array([(-pointu1[2]*2/3, pointu1[0]*2/3, -pointu1[1]*2/3),
                  (-pointy1[2]*2/3, pointy1[0]*2/3, -pointy1[1]*2/3),
                  (-pointg1[2]*2/3, pointg1[0]*2/3, -pointg1[1]*2/3),
                  (((-pointu1[2]*2/3)+(-pointy1[2]*2/3)+(-pointg1[2]*2/3))/3 - 0.001,((pointu1[0]*2/3)+(pointy1[0]*2/3)+(pointg1[0]*2/3))/3,((-pointu1[1]*2/3)+(-pointy1[1]*2/3)+(-pointg1[1]*2/3))/3)],
                 dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    face4 = np.array([([0, 1, 2], 0, 0, 0),#black face
                ([0, 2, 3], 134,   0,   102),
                ([0, 1, 3],   255, 255,   0),
                ([1, 2, 3],   0,   255, 0)],
               dtype=[('vertex_indices', 'i4', (3,)),
                      ('red', 'u1'), ('green', 'u1'),
                      ('blue', 'u1')])
    
    # Plane #5 is defined by purple, yellow, and green (arbitrary) on white
    vertex5 = np.array([(-pointu2[2]*2/3, pointu2[0]*2/3, -pointu2[1]*2/3),
                  (-pointy2[2]*2/3, pointy2[0]*2/3, -pointy2[1]*2/3),
                  (-pointg2[2]*2/3, pointg2[0]*2/3, -pointg2[1]*2/3),
                  (((-pointu2[2]*2/3)+(-pointy2[2]*2/3)+(-pointg2[2]*2/3))/3 - 0.001,((pointu2[0]*2/3)+(pointy2[0]*2/3)+(pointg2[0]*2/3))/3,((-pointu2[1]*2/3)+(-pointy2[1]*2/3)+(-pointg2[1]*2/3))/3)],
                 dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    face5 = np.array([([0, 1, 2], 255, 255, 255),#white face
                ([0, 2, 3], 134,   0,   102),
                ([0, 1, 3],   255, 255,   0),
                ([1, 2, 3],   0,   255, 0)],
               dtype=[('vertex_indices', 'i4', (3,)),
                      ('red', 'u1'), ('green', 'u1'),
                      ('blue', 'u1')])

    # Plane #6 is defined by turquoise, red, and orange (arbitrary) on black
    vertex6 = np.array([(-pointt1[2]*2/3, pointt1[0]*2/3, -pointt1[1]*2/3),
                  (-pointr1[2]*2/3, pointr1[0]*2/3, -pointr1[1]*2/3),
                  (-pointo1[2]*2/3, pointo1[0]*2/3, -pointo1[1]*2/3),
                  (((-pointt1[2]*2/3)+(-pointr1[2]*2/3)+(-pointo1[2]*2/3))/3 - 0.001,((pointt1[0]*2/3)+(pointr1[0]*2/3)+(pointo1[0]*2/3))/3,((-pointt1[1]*2/3)+(-pointr1[1]*2/3)+(-pointo1[1]*2/3))/3)],
                 dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

    face6 = np.array([([0, 1, 2], 0, 0, 0),#black face
                ([0, 2, 3], 0,   255,   255),
                ([0, 1, 3],   255, 0,   0),
                ([1, 2, 3],   255,   140, 0)],
               dtype=[('vertex_indices', 'i4', (3,)),
                      ('red', 'u1'), ('green', 'u1'),
                      ('blue', 'u1')])

    # Saves Point Cloud files under the given names (I have been changing it every time I want to make a new set of Point Clouds, otherwise it overwrites the previous file with the same name)
    PlyData([PlyElement.describe(vertex1, 'vertex'),PlyElement.describe(face1, 'face')]).write(bone1FileName)
    PlyData([PlyElement.describe(vertex2, 'vertex'),PlyElement.describe(face2, 'face')]).write(bone2FileName)
    PlyData([PlyElement.describe(vertex3, 'vertex'),PlyElement.describe(face3, 'face')]).write(bone3FileName)
    PlyData([PlyElement.describe(vertex4, 'vertex'),PlyElement.describe(face4, 'face')]).write(bone4FileName)
    PlyData([PlyElement.describe(vertex5, 'vertex'),PlyElement.describe(face5, 'face')]).write(bone5FileName)
    PlyData([PlyElement.describe(vertex6, 'vertex'),PlyElement.describe(face6, 'face')]).write(bone6FileName)

# Keep looping
while True:
    # Grab the current frame if no video file, then set current frame as the frame from enabled video source frame
    if vsbool == 1:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Set Intel RealSense D435 RGB frame as enabled video source frame
        vs = color_image

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))

        # OPTIONAL code for displaying RGB and Depth frames in a separate window
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)
        #cv2.waitKey(1)
        
        # OPTIONAL code for using Default Webcam/Facetime Camera
        # NOTE: If used, comment out the earlier "frame = vs" in line ######################################################147 (this line number has changed and is no longer correct)
        #frame = vs.read()
    
    # Setting up for getting Euclidean Real-World distances
    #device = profile.get_device()
    #projection = device.projection()
    #print(projection.query_vertices())
    ##stream = profile.get_stream(rs.stream.depth)
    ##vids = stream.as_video_stream_profile()
    ##intrinsics = vids.get_intrinsics()
    
    # Intrinsics & Extrinsics
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)

    # Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    
    # Define frame as current enabled video source frame
    frame = vs
   
    # Handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame
 
    # If we are viewing a video and we did not grab a frame, then we have reached the end of the video
    if frame is None:
        break
 
    # Resize the frame, blur it, and convert it to the HSV color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    ###############################################
    ##                                           ##
    ##      Mask For Each Supported Color        ##
    ##     (Code is Same For Each Section)       ##
    ##                                           ##
    ###############################################
    
    # construct a mask for the color "BLACK", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maskk = cv2.inRange(hsv, blackLower, blackUpper)
    maskk = cv2.erode(maskk, None, iterations=2)
    maskk = cv2.dilate(maskk, None, iterations=2)
    
    """# find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsk = cv2.findContours(maskk.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsk = cntsk[0] if imutils.is_cv2() else cntsk[1]
    centerk = None
 
    # only proceed if at least one contour was found
    if len(cntsk) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        ck = max(cntsk, key=cv2.contourArea)
        ((xk, yk), radiusk) = cv2.minEnclosingCircle(ck)
        Mk = cv2.moments(ck)
        centerk = (int(Mk["m10"] / Mk["m00"]), int(Mk["m01"] / Mk["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusk > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xk), int(yk)), int(radiusk),
                (0, 0, 0), 2)
            cv2.circle(frame, centerk, 5, (0, 0, 0), -1)
 
    # update the points queue
    ptsk.appendleft(centerk)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsk)):
        # if either of the tracked points are None, ignore
        # them
        if ptsk[i - 1] is None or ptsk[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessk = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsk[i - 1], ptsk[i], (0, 0, 0), thicknessk)"""
    
    # construct a mask for the color "WHITE", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maski = cv2.inRange(hsv, whiteLower, whiteUpper)
    maski = cv2.erode(maski, None, iterations=2)
    maski = cv2.dilate(maski, None, iterations=2)
    
    """# find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsi = cv2.findContours(maskk.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsi = cntsi[0] if imutils.is_cv2() else cntsi[1]
    centeri = None
 
    # only proceed if at least one contour was found
    if len(cntsi) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        ci = max(cntsi, key=cv2.contourArea)
        ((xi, yi), radiusi) = cv2.minEnclosingCircle(ci)
        Mi = cv2.moments(ci)
        centeri = (int(Mi["m10"] / Mi["m00"]), int(Mi["m01"] / Mi["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusi > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xi), int(yi)), int(radiusi),
                (0, 0, 0), 2)
            cv2.circle(frame, centeri, 5, (0, 0, 0), -1)
 
    # update the points queue
    ptsi.appendleft(centeri)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsi)):
        # if either of the tracked points are None, ignore
        # them
        if ptsi[i - 1] is None or ptsi[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessi = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsi[i - 1], ptsi[i], (0, 0, 0), thicknessi)"""
    
    # construct a mask for the color "GREEN", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maskg = cv2.inRange(hsv, greenLower, greenUpper)
    maskg = cv2.erode(maskg, None, iterations=2)
    maskg = cv2.dilate(maskg, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsg = cv2.findContours(maskg.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsg = cntsg[0] if imutils.is_cv2() else cntsg[1]
    centerg1 = None
    centerg2 = None
 
    # only proceed if at least one contour was found
    if len(cntsg) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cgt1 = max(cntsg, key=cv2.contourArea)
        cgt2 = min(cntsg, key=cv2.contourArea)
        
        ((xgt1, ygt1), radiusgt1) = cv2.minEnclosingCircle(cgt1)
        ((xgt2, ygt2), radiusgt2) = cv2.minEnclosingCircle(cgt2)
        
        if int(ygt1) - int(radiusgt1) - 2 > 0 and int(xgt1) + int(radiusgt1) + 2 < 599:
            if maskk[int(ygt1) - int(radiusgt1) - 2][int(xgt1) + int(radiusgt1) + 2] != 0:
                cg1 = cgt1
                cg2 = cgt2
            else:
                cg1 = cgt2
                cg2 = cgt1
        elif int(ygt1) - int(radiusgt1) - 2 < 0 and int(xgt1) + int(radiusgt1) + 2 < 599:
            if maskk[int(ygt1) + int(radiusgt1) + 2][int(xgt1) + int(radiusgt1) + 2] != 0:
                cg1 = cgt1
                cg2 = cgt2
            else:
                cg1 = cgt2
                cg2 = cgt1
        elif int(xgt1) + int(radiusgt1) + 2 > 599 and int(ygt1) - int(radiusgt1) - 2 > 0:
            if maskk[int(ygt1) - int(radiusgt1) - 2][int(xgt1) - int(radiusgt1) - 2] != 0:
                cg1 = cgt1
                cg2 = cgt2
            else:
                cg1 = cgt2
                cg2 = cgt1
        else:
            if maskk[int(ygt1) + int(radiusgt1) + 2][int(xgt1) - int(radiusgt1) - 2] != 0:
                cg1 = cgt1
                cg2 = cgt2
            else:
                cg1 = cgt2
                cg2 = cgt1
        
        ((xg1, yg1), radiusg1) = cv2.minEnclosingCircle(cg1)
        Mg1 = cv2.moments(cg1)
        centerg1 = (int(Mg1["m10"] / Mg1["m00"]), int(Mg1["m01"] / Mg1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusg1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xg1), int(yg1)), int(radiusg1),
                (0, 255, 0), 2)
            cv2.circle(frame, centerg1, 5, (0, 0, 0), -1)
        
        ((xg2, yg2), radiusg2) = cv2.minEnclosingCircle(cg2)
        Mg2 = cv2.moments(cg2)
        centerg2 = (int(Mg2["m10"] / Mg2["m00"]), int(Mg2["m01"] / Mg2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusg2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xg2), int(yg2)), int(radiusg2),
                (0, 255, 0), 2)
            cv2.circle(frame, centerg2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptsg1.appendleft(centerg1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsg1)):
        # if either of the tracked points are None, ignore
        # them
        if ptsg1[i - 1] is None or ptsg1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessg1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsg1[i - 1], ptsg1[i], (0, 255, 0), thicknessg1)
        
    # update the points queue
    ptsg2.appendleft(centerg2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsg2)):
        # if either of the tracked points are None, ignore
        # them
        if ptsg2[i - 1] is None or ptsg2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessg2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsg2[i - 1], ptsg2[i], (0, 255, 0), thicknessg2)
 
    # construct a mask for the color "BLUE", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maskb = cv2.inRange(hsv, blueLower, blueUpper)
    maskb = cv2.erode(maskb, None, iterations=2)
    maskb = cv2.dilate(maskb, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsb = cv2.findContours(maskb.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsb = cntsb[0] if imutils.is_cv2() else cntsb[1]
    centerb1 = None
    centerb2 = None
 
    # only proceed if at least one contour was found
    if len(cntsb) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cbt1 = max(cntsb, key=cv2.contourArea)
        cbt2 = min(cntsb, key=cv2.contourArea)
        
        ((xbt1, ybt1), radiusbt1) = cv2.minEnclosingCircle(cbt1)
        ((xbt2, ybt2), radiusbt2) = cv2.minEnclosingCircle(cbt2)
        
        if int(ybt1) + int(radiusbt1) + 2 < 449 and int(xbt1) + int(radiusbt1) + 2 < 599:
            if maskk[int(ybt1) + int(radiusbt1) + 2][int(xbt1)] != 0:
                cb1 = cbt1
                cb2 = cbt2
            else:
                cb1 = cbt2
                cb2 = cbt1
        elif int(ybt1) + int(radiusbt1) + 2 > 449 and int(xbt1) + int(radiusbt1) + 2 < 599:
            if maskk[int(ybt1) - int(radiusbt1) - 2][int(xbt1)] != 0:
                cb1 = cbt1
                cb2 = cbt2
            else:
                cb1 = cbt2
                cb2 = cbt1
        elif int(xbt1) + int(radiusbt1) + 2 > 599 and int(ybt1) + int(radiusbt1) + 2 < 449:
            if maskk[int(ybt1) + int(radiusbt1) + 2][int(xbt1)] != 0:
                cb1 = cbt1
                cb2 = cbt2
            else:
                cb1 = cbt2
                cb2 = cbt1
        else:
            if maskk[int(ybt1) - int(radiusbt1) - 2][int(xbt1)] != 0:
                cb1 = cbt1
                cb2 = cbt2
            else:
                cb1 = cbt2
                cb2 = cbt1
        
        ((xb1, yb1), radiusb1) = cv2.minEnclosingCircle(cb1)
        Mb1 = cv2.moments(cb1)
        centerb1 = (int(Mb1["m10"] / Mb1["m00"]), int(Mb1["m01"] / Mb1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusb1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xb1), int(yb1)), int(radiusb1),
                (255, 0, 0), 2)
            cv2.circle(frame, centerb1, 5, (0, 0, 0), -1)
        
        ((xb2, yb2), radiusb2) = cv2.minEnclosingCircle(cb2)
        Mb2 = cv2.moments(cb2)
        centerb2 = (int(Mb2["m10"] / Mb2["m00"]), int(Mb2["m01"] / Mb2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusb2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xb2), int(yb2)), int(radiusb2),
                (255, 0, 0), 2)
            cv2.circle(frame, centerb2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptsb1.appendleft(centerb1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsb1)):
        # if either of the tracked points are None, ignore
        # them
        if ptsb1[i - 1] is None or ptsb1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessb1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsb1[i - 1], ptsb1[i], (255, 0, 0), thicknessb1)
        
    # update the points queue
    ptsb2.appendleft(centerb2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsb2)):
        # if either of the tracked points are None, ignore
        # them
        if ptsb2[i - 1] is None or ptsb2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessb2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsb2[i - 1], ptsb2[i], (255, 0, 0), thicknessb2)
    
    # construct a mask for the color "YELLOW", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    masky = cv2.inRange(hsv, yellowLower, yellowUpper)
    masky = cv2.erode(masky, None, iterations=2)
    masky = cv2.dilate(masky, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsy = cv2.findContours(masky.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsy = cntsy[0] if imutils.is_cv2() else cntsy[1]
    centery1 = None
    centery2 = None
 
    # only proceed if at least one contour was found
    if len(cntsy) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cyt1 = max(cntsy, key=cv2.contourArea)
        cyt2 = min(cntsy, key=cv2.contourArea)
        
        ((xyt1, yyt1), radiusyt1) = cv2.minEnclosingCircle(cyt1)
        ((xyt2, yyt2), radiusyt2) = cv2.minEnclosingCircle(cyt2)
        
        if int(yyt1) + int(radiusyt1) + 2 < 449 and int(xyt1) - int(radiusyt1) - 2 > 0:
            if maskk[int(yyt1) + int(radiusyt1) + 2][int(xyt1) - int(radiusyt1) - 2] != 0:
                cy1 = cyt1
                cy2 = cyt2
            else:
                cy1 = cyt2
                cy2 = cyt1
        elif int(yyt1) + int(radiusyt1) + 2 > 449 and int(xyt1) - int(radiusyt1) - 2 > 0:
            if maskk[int(yyt1) - int(radiusyt1) - 2][int(xyt1) - int(radiusyt1) - 2] != 0:
                cy1 = cyt1
                cy2 = cyt2
            else:
                cy1 = cyt2
                cy2 = cyt1
        elif int(xyt1) - int(radiusyt1) - 2 < 0 and int(yyt1) + int(radiusyt1) + 2 < 449:
            if maskk[int(yyt1) + int(radiusyt1) + 2][int(xyt1) + int(radiusyt1) + 2] != 0:
                cy1 = cyt1
                cy2 = cyt2
            else:
                cy1 = cyt2
                cy2 = cyt1
        else:
            if maskk[int(yyt1) - int(radiusyt1) - 2][int(xyt1) + int(radiusyt1) + 2] != 0:
                cy1 = cyt1
                cy2 = cyt2
            else:
                cy1 = cyt2
                cy2 = cyt1
        
        ((xy1, yy1), radiusy1) = cv2.minEnclosingCircle(cy1)
        My1 = cv2.moments(cy1)
        centery1 = (int(My1["m10"] / My1["m00"]), int(My1["m01"] / My1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusy1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xy1), int(yy1)), int(radiusy1),
                (0, 255, 255), 2)
            cv2.circle(frame, centery1, 5, (0, 0, 0), -1)
        
        ((xy2, yy2), radiusy2) = cv2.minEnclosingCircle(cy2)
        My2 = cv2.moments(cy2)
        centery2 = (int(My2["m10"] / My2["m00"]), int(My2["m01"] / My2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusy2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xy2), int(yy2)), int(radiusy2),
                (0, 255, 255), 2)
            cv2.circle(frame, centery2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptsy1.appendleft(centery1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsy1)):
        # if either of the tracked points are None, ignore
        # them
        if ptsy1[i - 1] is None or ptsy1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessy1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsy1[i - 1], ptsy1[i], (0, 255, 255), thicknessy1)
        
    # update the points queue
    ptsy2.appendleft(centery2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsy2)):
        # if either of the tracked points are None, ignore
        # them
        if ptsy2[i - 1] is None or ptsy2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessy2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsy2[i - 1], ptsy2[i], (0, 255, 255), thicknessy2)
        
    # construct a mask for the color "RED", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maskr = cv2.inRange(hsv, redLower, redUpper) + cv2.inRange(hsv, redLower2, redUpper2)
    maskr = cv2.erode(maskr, None, iterations=2)
    maskr = cv2.dilate(maskr, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsr = cv2.findContours(maskr.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsr = cntsr[0] if imutils.is_cv2() else cntsr[1]
    centerr1 = None
    centerr2 = None
 
    # only proceed if at least one contour was found
    if len(cntsr) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        crt1 = max(cntsr, key=cv2.contourArea)
        crt2 = min(cntsr, key=cv2.contourArea)
        
        ((xrt1, yrt1), radiusrt1) = cv2.minEnclosingCircle(crt1)
        ((xrt2, yrt2), radiusrt2) = cv2.minEnclosingCircle(crt2)
        
        if int(yrt1) - int(radiusrt1) - 2 > 0 and int(xrt1) - int(radiusrt1) - 2 > 0:
            if maskk[int(yrt1) - int(radiusrt1) - 2][int(xrt1) - int(radiusrt1) - 2] != 0:
                cr1 = crt1
                cr2 = crt2
            else:
                cr1 = crt2
                cr2 = crt1
        elif int(yrt1) - int(radiusrt1) - 2 < 0 and int(xrt1) - int(radiusrt1) - 2 > 0:
            if maskk[int(yrt1) + int(radiusrt1) + 2][int(xrt1) - int(radiusrt1) - 2] != 0:
                cr1 = crt1
                cr2 = crt2
            else:
                cr1 = crt2
                cr2 = crt1
        elif int(xrt1) - int(radiusrt1) - 2 < 0 and int(yrt1) - int(radiusrt1) - 2 > 0:
            if maskk[int(yrt1) - int(radiusrt1) - 2][int(xrt1) + int(radiusrt1) + 2] != 0:
                cr1 = crt1
                cr2 = crt2
            else:
                cr1 = crt2
                cr2 = crt1
        else:
            if maskk[int(yrt1) + int(radiusrt1) + 2][int(xrt1) + int(radiusrt1) + 2] != 0:
                cr1 = crt1
                cr2 = crt2
            else:
                cr1 = crt2
                cr2 = crt1
        
        ((xr1, yr1), radiusr1) = cv2.minEnclosingCircle(cr1)
        Mr1 = cv2.moments(cr1)
        centerr1 = (int(Mr1["m10"] / Mr1["m00"]), int(Mr1["m01"] / Mr1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusr1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xr1), int(yr1)), int(radiusr1),
                (0, 0, 255), 2)
            cv2.circle(frame, centerr1, 5, (0, 0, 0), -1)
        
        ((xr2, yr2), radiusr2) = cv2.minEnclosingCircle(cr2)
        Mr2 = cv2.moments(cr2)
        centerr2 = (int(Mr2["m10"] / Mr2["m00"]), int(Mr2["m01"] / Mr2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusr2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xr2), int(yr2)), int(radiusr2),
                (0, 0, 255), 2)
            cv2.circle(frame, centerr2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptsr1.appendleft(centerr1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsr1)):
        # if either of the tracked points are None, ignore
        # them
        if ptsr1[i - 1] is None or ptsr1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessr1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsr1[i - 1], ptsr1[i], (0, 0, 255), thicknessr1)
        
    # update the points queue
    ptsr2.appendleft(centerr2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsr2)):
        # if either of the tracked points are None, ignore
        # them
        if ptsr2[i - 1] is None or ptsr2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessr2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsr2[i - 1], ptsr2[i], (0, 0, 255), thicknessr2)
    
    # construct a mask for the color "ORANGE", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    masko = cv2.inRange(hsv, orangeLower, orangeUpper)
    masko = cv2.erode(masko, None, iterations=2)
    masko = cv2.dilate(masko, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntso = cv2.findContours(masko.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntso = cntso[0] if imutils.is_cv2() else cntso[1]
    centero1 = None
    centero2 = None
 
    # only proceed if at least one contour was found
    if len(cntso) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cot1 = max(cntso, key=cv2.contourArea)
        cot2 = min(cntso, key=cv2.contourArea)
        
        ((xot1, yot1), radiusot1) = cv2.minEnclosingCircle(cot1)
        ((xot2, yot2), radiusot2) = cv2.minEnclosingCircle(cot2)
        
        if int(yot1) + int(radiusot1) + 2 < 449 and int(xot1) - int(radiusot1) - 2 > 0:
            if maskk[int(yot1) + int(radiusot1) + 2][int(xot1) - int(radiusot1) - 2] != 0:
                co1 = cot1
                co2 = cot2
            else:
                co1 = cot2
                co2 = cot1
        elif int(yot1) + int(radiusot1) + 2 > 449 and int(xot1) - int(radiusot1) - 2 > 0:
            if maskk[int(yot1) - int(radiusot1) - 2][int(xot1) - int(radiusot1) - 2] != 0:
                co1 = cot1
                co2 = cot2
            else:
                co1 = cot2
                co2 = cot1
        elif int(xot1) - int(radiusot1) - 2 < 0 and int(yot1) + int(radiusot1) + 2 < 449:
            if maskk[int(yot1) + int(radiusot1) + 2][int(xot1) + int(radiusot1) + 2] != 0:
                co1 = cot1
                co2 = cot2
            else:
                co1 = cot2
                co2 = cot1
        else:
            if maskk[int(yot1) - int(radiusot1) - 2][int(xot1) + int(radiusot1) + 2] != 0:
                co1 = cot1
                co2 = cot2
            else:
                co1 = cot2
                co2 = cot1
        
        ((xo1, yo1), radiuso1) = cv2.minEnclosingCircle(co1)
        Mo1 = cv2.moments(co1)
        centero1 = (int(Mo1["m10"] / Mo1["m00"]), int(Mo1["m01"] / Mo1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiuso1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xo1), int(yo1)), int(radiuso1),
                (0, 140, 255), 2)
            cv2.circle(frame, centero1, 5, (0, 0, 0), -1)
        
        ((xo2, yo2), radiuso2) = cv2.minEnclosingCircle(co2)
        Mo2 = cv2.moments(co2)
        centero2 = (int(Mo2["m10"] / Mo2["m00"]), int(Mo2["m01"] / Mo2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiuso2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xo2), int(yo2)), int(radiuso2),
                (0, 140, 255), 2)
            cv2.circle(frame, centero2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptso1.appendleft(centero1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptso1)):
        # if either of the tracked points are None, ignore
        # them
        if ptso1[i - 1] is None or ptso1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknesso1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptso1[i - 1], ptso1[i], (0, 140, 255), thicknesso1)
        
    # update the points queue
    ptso2.appendleft(centero2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptso2)):
        # if either of the tracked points are None, ignore
        # them
        if ptso2[i - 1] is None or ptso2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknesso2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptso2[i - 1], ptso2[i], (0, 140, 255), thicknesso2)
        
    
    # construct a mask for the color "PINK", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maskp = cv2.inRange(hsv, pinkLower, pinkUpper)
    maskp = cv2.erode(maskp, None, iterations=2)
    maskp = cv2.dilate(maskp, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsp = cv2.findContours(maskp.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsp = cntsp[0] if imutils.is_cv2() else cntsp[1]
    centerp1 = None
    centerp2 = None
 
    # only proceed if at least one contour was found
    if len(cntsp) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cpt1 = max(cntsp, key=cv2.contourArea)
        cpt2 = min(cntsp, key=cv2.contourArea)
        
        ((xpt1, ypt1), radiuspt1) = cv2.minEnclosingCircle(cpt1)
        ((xpt2, ypt2), radiuspt2) = cv2.minEnclosingCircle(cpt2)
        
        if int(ypt1) + int(radiuspt1) + 2 < 449 and int(xpt1) - int(radiuspt1) - 2 > 0:
            if maskk[int(ypt1)][int(xpt1) - int(radiuspt1) - 2] != 0:
                cp1 = cpt1
                cp2 = cpt2
            else:
                cp1 = cpt2
                cp2 = cpt1
        elif int(ypt1) + int(radiuspt1) + 2 > 449 and int(xpt1) - int(radiuspt1) - 2 > 0:
            if maskk[int(ypt1)][int(xpt1) - int(radiuspt1) - 2] != 0:
                cp1 = cpt1
                cp2 = cpt2
            else:
                cp1 = cpt2
                cp2 = cpt1
        elif int(xpt1) - int(radiuspt1) - 2 < 0 and int(ypt1) + int(radiuspt1) + 2 < 449:
            if maskk[int(ypt1)][int(xpt1) + int(radiuspt1) + 2] != 0:
                cp1 = cpt1
                cp2 = cpt2
            else:
                cp1 = cpt2
                cp2 = cpt1
        else:
            if maskk[int(ypt1)][int(xpt1) + int(radiuspt1) + 2] != 0:
                cp1 = cpt1
                cp2 = cpt2
            else:
                cp1 = cpt2
                cp2 = cpt1
        
        ((xp1, yp1), radiusp1) = cv2.minEnclosingCircle(cp1)
        Mp1 = cv2.moments(cp1)
        centerp1 = (int(Mp1["m10"] / Mp1["m00"]), int(Mp1["m01"] / Mp1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusp1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xp1), int(yp1)), int(radiusp1),
                (255, 84, 243), 2)
            cv2.circle(frame, centerp1, 5, (0, 0, 0), -1)
        
        ((xp2, yp2), radiusp2) = cv2.minEnclosingCircle(cp2)
        Mp2 = cv2.moments(cp2)
        centerp2 = (int(Mp2["m10"] / Mp2["m00"]), int(Mp2["m01"] / Mp2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusp2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xp2), int(yp2)), int(radiusp2),
                (255, 84, 243), 2)
            cv2.circle(frame, centerp2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptsp1.appendleft(centerp1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsp1)):
        # if either of the tracked points are None, ignore
        # them
        if ptsp1[i - 1] is None or ptsp1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessp1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsp1[i - 1], ptsp1[i], (255, 84, 243), thicknessp1)
        
    # update the points queue
    ptsp2.appendleft(centerp2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsp2)):
        # if either of the tracked points are None, ignore
        # them
        if ptsp2[i - 1] is None or ptsp2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessp2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsp2[i - 1], ptsp2[i], (255, 84, 243), thicknessp2)
    
    # construct a mask for the color "PURPLE", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    masku = cv2.inRange(hsv, purpleLower, purpleUpper)
    masku = cv2.erode(masku, None, iterations=2)
    masku = cv2.dilate(masku, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsu = cv2.findContours(masku.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsu = cntsu[0] if imutils.is_cv2() else cntsu[1]
    centeru1 = None
    centeru2 = None
 
    # only proceed if at least one contour was found
    if len(cntsu) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cut1 = max(cntsu, key=cv2.contourArea)
        cut2 = min(cntsu, key=cv2.contourArea)
        
        ((xut1, yut1), radiusut1) = cv2.minEnclosingCircle(cut1)
        ((xut2, yut2), radiusut2) = cv2.minEnclosingCircle(cut2)
        
        if int(yut1) + int(radiusut1) + 2 < 449 and int(xut1) + int(radiusut1) + 2 < 599:
            if maskk[int(yut1) + int(radiusut1) + 2][int(xut1)] != 0:
                cu1 = cut1
                cu2 = cut2
            else:
                cu1 = cut2
                cu2 = cut1
        elif int(yut1) + int(radiusut1) + 2 > 449 and int(xut1) + int(radiusut1) + 2 < 599:
            if maskk[int(yut1) - int(radiusut1) - 2][int(xut1)] != 0:
                cu1 = cut1
                cu2 = cut2
            else:
                cu1 = cut2
                cu2 = cut1
        elif int(xut1) + int(radiusut1) + 2 > 599 and int(yut1) + int(radiusut1) + 2 < 449:
            if maskk[int(yut1) + int(radiusut1) + 2][int(xut1)] != 0:
                cu1 = cut1
                cu2 = cut2
            else:
                cu1 = cut2
                cu2 = cut1
        else:
            if maskk[int(yut1) - int(radiusut1) - 2][int(xut1)] != 0:
                cu1 = cut1
                cu2 = cut2
            else:
                cu1 = cut2
                cu2 = cut1
        
        ((xu1, yu1), radiusu1) = cv2.minEnclosingCircle(cu1)
        Mu1 = cv2.moments(cu1)
        centeru1 = (int(Mu1["m10"] / Mu1["m00"]), int(Mu1["m01"] / Mu1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusu1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xu1), int(yu1)), int(radiusu1),
                (102, 0, 134), 2)
            cv2.circle(frame, centeru1, 5, (0, 0, 0), -1)
        
        ((xu2, yu2), radiusu2) = cv2.minEnclosingCircle(cu2)
        Mu2 = cv2.moments(cu2)
        centeru2 = (int(Mu2["m10"] / Mu2["m00"]), int(Mu2["m01"] / Mu2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusu2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xu2), int(yu2)), int(radiusu2),
                (102, 0, 134), 2)
            cv2.circle(frame, centeru2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptsu1.appendleft(centeru1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsu1)):
        # if either of the tracked points are None, ignore
        # them
        if ptsu1[i - 1] is None or ptsu1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessu1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsu1[i - 1], ptsu1[i], (102, 0, 134), thicknessu1)
        
    # update the points queue
    ptsu2.appendleft(centeru2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsu2)):
        # if either of the tracked points are None, ignore
        # them
        if ptsu2[i - 1] is None or ptsu2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessu2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsu2[i - 1], ptsu2[i], (102, 0, 134), thicknessu2)
        
    # construct a mask for the color "GRELLOW", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maskw = cv2.inRange(hsv, grellowLower, grellowUpper)
    maskw = cv2.erode(maskw, None, iterations=2)
    maskw = cv2.dilate(maskw, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntsw = cv2.findContours(maskw.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntsw = cntsw[0] if imutils.is_cv2() else cntsw[1]
    centerw1 = None
    centerw2 = None
 
    # only proceed if at least one contour was found
    if len(cntsw) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        cwt1 = max(cntsw, key=cv2.contourArea)
        cwt2 = min(cntsw, key=cv2.contourArea)
        
        ((xwt1, ywt1), radiuswt1) = cv2.minEnclosingCircle(cwt1)
        ((xwt2, ywt2), radiuswt2) = cv2.minEnclosingCircle(cwt2)
        
        if int(ywt1) + int(radiuswt1) + 2 < 449 and int(xwt1) + int(radiuswt1) + 2 < 599:
            if maskk[int(ywt1) + int(radiuswt1) + 2][int(xwt1)] != 0:
                cw1 = cwt1
                cw2 = cwt2
            else:
                cw1 = cwt2
                cw2 = cwt1
        elif int(ywt1) + int(radiuswt1) + 2 > 449 and int(xwt1) + int(radiuswt1) + 2 < 599:
            if maskk[int(ywt1) - int(radiuswt1) - 2][int(xwt1)] != 0:
                cw1 = cwt1
                cw2 = cwt2
            else:
                cw1 = cwt2
                cw2 = cwt1
        elif int(xwt1) + int(radiuswt1) + 2 > 599 and int(ywt1) + int(radiuswt1) + 2 < 449:
            if maskk[int(ywt1) + int(radiuswt1) + 2][int(xwt1)] != 0:
                cw1 = cwt1
                cw2 = cwt2
            else:
                cw1 = cwt2
                cw2 = cwt1
        else:
            if maskk[int(ywt1) - int(radiuswt1) - 2][int(xwt1)] != 0:
                cw1 = cwt1
                cw2 = cwt2
            else:
                cw1 = cwt2
                cw2 = cwt1
        
        ((xw1, yw1), radiusw1) = cv2.minEnclosingCircle(cw1)
        Mw1 = cv2.moments(cw1)
        centerw1 = (int(Mw1["m10"] / Mw1["m00"]), int(Mw1["m01"] / Mw1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusw1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xw1), int(yw1)), int(radiusw1),
                (0, 255, 213), 2)
            cv2.circle(frame, centerw1, 5, (0, 0, 0), -1)
        
        ((xw2, yw2), radiusw2) = cv2.minEnclosingCircle(cw2)
        Mw2 = cv2.moments(cw2)
        centerw2 = (int(Mw2["m10"] / Mw2["m00"]), int(Mw2["m01"] / Mw2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiusw2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xw2), int(yw2)), int(radiusw2),
                (0, 255, 213), 2)
            cv2.circle(frame, centerw2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptsw1.appendleft(centerw1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsw1)):
        # if either of the tracked points are None, ignore
        # them
        if ptsw1[i - 1] is None or ptsw1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessw1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsw1[i - 1], ptsw1[i], (0, 255, 213), thicknessw1)
        
    # update the points queue
    ptsw2.appendleft(centerw2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptsw2)):
        # if either of the tracked points are None, ignore
        # them
        if ptsw2[i - 1] is None or ptsw2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknessw2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptsw2[i - 1], ptsw2[i], (0, 255, 213), thicknessw2)
        
    # construct a mask for the color "TURQUOISE", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    maskt = cv2.inRange(hsv, turquoiseLower, turquoiseUpper)
    maskt = cv2.erode(maskt, None, iterations=2)
    maskt = cv2.dilate(maskt, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cntst = cv2.findContours(maskt.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cntst = cntst[0] if imutils.is_cv2() else cntst[1]
    centert1 = None
    centert2 = None
 
    # only proceed if at least one contour was found
    if len(cntst) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        ctt1 = max(cntst, key=cv2.contourArea)
        ctt2 = min(cntst, key=cv2.contourArea)
        
        ((xtt1, ytt1), radiustt1) = cv2.minEnclosingCircle(ctt1)
        ((xtt2, ytt2), radiustt2) = cv2.minEnclosingCircle(ctt2)
        
        if int(ytt1) + int(radiustt1) + 2 < 449 and int(xtt1) - int(radiustt1) - 2 > 0:
            if maskk[int(ytt1) + int(radiustt1) + 2][int(xtt1) - int(radiustt1) - 2] != 0:
                ct1 = ctt1
                ct2 = ctt2
            else:
                ct1 = ctt2
                ct2 = ctt1
        elif int(ytt1) + int(radiustt1) + 2 > 449 and int(xtt1) - int(radiustt1) - 2 > 0:
            if maskk[int(ytt1) - int(radiustt1) - 2][int(xtt1) - int(radiustt1) - 2] != 0:
                ct1 = ctt1
                ct2 = ctt2
            else:
                ct1 = ctt2
                ct2 = ctt1
        elif int(xtt1) - int(radiustt1) - 2 < 0 and int(ytt1) + int(radiustt1) + 2 < 449:
            if maskk[int(ytt1) + int(radiustt1) + 2][int(xtt1) + int(radiustt1) + 2] != 0:
                ct1 = ctt1
                ct2 = ctt2
            else:
                ct1 = ctt2
                ct2 = ctt1
        else:
            if maskk[int(ytt1) - int(radiustt1) - 2][int(xtt1) + int(radiustt1) + 2] != 0:
                ct1 = ctt1
                ct2 = ctt2
            else:
                ct1 = ctt2
                ct2 = ctt1
        
        ((xt1, yt1), radiust1) = cv2.minEnclosingCircle(ct1)
        Mt1 = cv2.moments(ct1)
        centert1 = (int(Mt1["m10"] / Mt1["m00"]), int(Mt1["m01"] / Mt1["m00"]))

        # only proceed if the radius meets a minimum size
        if radiust1 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xt1), int(yt1)), int(radiust1),
                (255, 255, 0), 2)
            cv2.circle(frame, centert1, 5, (0, 0, 0), -1)
        
        ((xt2, yt2), radiust2) = cv2.minEnclosingCircle(ct2)
        Mt2 = cv2.moments(ct2)
        centert2 = (int(Mt2["m10"] / Mt2["m00"]), int(Mt2["m01"] / Mt2["m00"]))

        # only proceed if the radius meets a minimum size
        if radiust2 > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(xt2), int(yt2)), int(radiust2),
                (255, 255, 0), 2)
            cv2.circle(frame, centert2, 5, (255, 255, 255), -1)
 
    # update the points queue
    ptst1.appendleft(centert1)
    
    # loop over the set of tracked points
    for i in range(1, len(ptst1)):
        # if either of the tracked points are None, ignore
        # them
        if ptst1[i - 1] is None or ptst1[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknesst1 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptst1[i - 1], ptst1[i], (255, 255, 0), thicknesst1)
        
    # update the points queue
    ptst2.appendleft(centert2)
    
    # loop over the set of tracked points
    for i in range(1, len(ptst2)):
        # if either of the tracked points are None, ignore
        # them
        if ptst2[i - 1] is None or ptst2[i] is None:
            continue
 
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thicknesst2 = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, ptst2[i - 1], ptst2[i], (255, 255, 0), thicknesst2)
        
    ###############################################
    ##                                           ##
    ##     System Printing of Calculations       ##
    ##      And Exporting of Point Clouds        ##
    ##                                           ##
    ###############################################
    
    # Display the frame in a window titled "Frame"
    cv2.imshow("Frame", frame)
    # Display the mask for the color Black
    cv2.imshow("Mask: Black", maskk)
    #cv2.imshow("Mask: Grellow", maskw)
    #cv2.imshow("Mask: Purple", masku)
    
    key = cv2.waitKey(1) & 0xFF
 
    # Sets the frame rate to 20 FPS
    time.sleep(0.05)

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        pipeline.stop()
        break
    
    if key == ord("c"):
        if badframe == 0:
            # Calculate Roll values
            rollLunate = roll(pointw1, pointp1, pointb1, [0,0,0])
            rollCapitate = roll(pointu1, pointy1, pointg1, [0,0,0])
            rollHamate = roll(pointt2, pointr2, pointo2, [0,0,0])
            rollScaphoid = roll(pointt1, pointo1, pointr1, [0,0,0])
            rollTrapezoid = roll(pointw2, pointp2, pointb2, [0,0,0])
            rollTriquetrum = roll(pointu2, pointy2, pointg2, [0,0,0])

            # Calculate Yaw values
            yawLunate = yaw(pointw1, pointp1, pointb1, [0,0,0])
            yawCapitate = yaw(pointu1, pointy1, pointg1, [0,0,0])
            yawHamate = yaw(pointt2, pointr2, pointo2, [0,0,0])
            yawScaphoid = yaw(pointt1, pointo1, pointr1, [0,0,0])
            yawTrapezoid = yaw(pointw2, pointp2, pointb2, [0,0,0])
            yawTriquetrum = yaw(pointu2, pointy2, pointg2, [0,0,0])

            # Calculate Pitch values
            pitchLunate = pitch(pointw1, pointp1, pointb1, [0,0,0])
            pitchCapitate = pitch(pointu1, pointy1, pointg1, [0,0,0])
            pitchHamate = pitch(pointt2, pointr2, pointo2, [0,0,0])
            pitchScaphoid = pitch(pointt1, pointo1, pointr1, [0,0,0])
            pitchTrapezoid = pitch(pointw2, pointp2, pointb2, [0,0,0])
            pitchTriquetrum = pitch(pointu2, pointy2, pointg2, [0,0,0])

            lunateBaseline[0] = rollLunate
            capitateBaseline[0] = rollCapitate
            hamateBaseline[0] = rollHamate
            scaphoidBaseline[0] = rollScaphoid
            trapezoidBaseline[0] = rollTrapezoid
            triquetrumBaseline[0] = rollTriquetrum

            lunateBaseline[1] = yawLunate
            capitateBaseline[1] = yawCapitate
            hamateBaseline[1] = yawHamate
            scaphoidBaseline[1] = yawScaphoid
            trapezoidBaseline[1] = yawTrapezoid
            triquetrumBaseline[1] = yawTriquetrum

            lunateBaseline[2] = pitchLunate
            capitateBaseline[2] = pitchCapitate
            hamateBaseline[2] = pitchHamate
            scaphoidBaseline[2] = pitchScaphoid
            trapezoidBaseline[2] = pitchTrapezoid
            triquetrumBaseline[2] = pitchTriquetrum

            ryp = 8
        else:
            print("\nCannot calibrate from this frame!")
    
    # If nothing in frame, display a message every 10 seconds
    if xg1 == 0 and yg1 == 0 and xb1 == 0 and yb1 == 0 and xy1 == 0 and yy1 == 0 and xr1 == 0 and yr1 == 0 and xo1 == 0 and yo1 == 0 and xp1 == 0 and yp1 == 0 and xu1 == 0 and yu1 == 0 and xt1 == 0 and yt1 == 0 and xw1 == 0 and yw1 == 0 and xg2 == 0 and yg2 == 0 and xb2 == 0 and yb2 == 0 and xy2 == 0 and yy2 == 0 and xr2 == 0 and yr2 == 0 and xo2 == 0 and yo2 == 0 and xp2 == 0 and yp2 == 0 and xu2 == 0 and yu2 == 0 and xt2 == 0 and yt2 == 0 and xw2 == 0 and yw2 == 0:
        # and xa == 0 and ya == 0 and xn == 0 and yn == 0:
        if pfalse == 1:
            print("Out of Frame!" + "\n")
            pfalse +=1
            ptrue = 1
        elif pfalse == 200:# Prints the message for one frame every ten seconds (while there are no colors in the frame)
            pfalse = 1
            ptrue = 1
        else:
            pfalse +=1
            ptrue = 1
    
    # If any color is being tracked, (for one frame every 2 seconds) display all Depth/Coord values, Euclidean Distance values between Black, Red, and Purple, and Roll/Yaw/Pitch values every 1/5 of a second; then, reset all coordinates to (0,0)
    else:
        if ptrue == 1:
            if badframe == 0:
                dtNOW = datetime.now()
                day = str(dtNOW).split()
                time3 = day[1].split('.')
                time2 = time3[0].split(':')
                dtFix = day[0] + "_" + time2[0] + "_" + time2[1] + "_" + time2[2] + "_" + time3[1]
            
            badframe = 0
            
            # Get center depths and correct with scalar multiple (2/3)
            depthg1 = aligned_depth_frame.get_distance(int(xg1),int(yg1))
            depthg1 = depthg1 * 2 / 3
            depthg2 = aligned_depth_frame.get_distance(int(xg2),int(yg2))
            depthg2 = depthg2 * 2 / 3
            
            depthb1 = aligned_depth_frame.get_distance(int(xb1),int(yb1))
            depthb1 = depthb1 * 2 / 3
            depthb2 = aligned_depth_frame.get_distance(int(xb2),int(yb2))
            depthb2 = depthb2 * 2 / 3
            
            depthy1 = aligned_depth_frame.get_distance(int(xy1),int(yy1))
            depthy1 = depthy1 * 2 / 3
            depthy2 = aligned_depth_frame.get_distance(int(xy2),int(yy2))
            depthy2 = depthy2 * 2 / 3
            
            depthr1 = aligned_depth_frame.get_distance(int(xr1),int(yr1))
            depthr1 = depthr1 * 2 / 3
            depthr2 = aligned_depth_frame.get_distance(int(xr2),int(yr2))
            depthr2 = depthr2 * 2 / 3
            
            deptho1 = aligned_depth_frame.get_distance(int(xo1),int(yo1))
            deptho1 = deptho1 * 2 / 3
            deptho2 = aligned_depth_frame.get_distance(int(xo2),int(yo2))
            deptho2 = deptho2 * 2 / 3
            
            depthp1 = aligned_depth_frame.get_distance(int(xp1),int(yp1))
            depthp1 = depthp1 * 2 / 3
            depthp2 = aligned_depth_frame.get_distance(int(xp2),int(yp2))
            depthp2 = depthp2 * 2 / 3
            
            depthu1 = aligned_depth_frame.get_distance(int(xu1),int(yu1))
            depthu1 = depthu1 * 2 / 3
            depthu2 = aligned_depth_frame.get_distance(int(xu2),int(yu2))
            depthu2 = depthu2 * 2 / 3
            
            deptht1 = aligned_depth_frame.get_distance(int(xt1),int(yt1))
            deptht1 = deptht1 * 2 / 3
            deptht2 = aligned_depth_frame.get_distance(int(xt2),int(yt2))
            deptht2 = deptht2 * 2 / 3
            
            depthw1 = aligned_depth_frame.get_distance(int(xw1),int(yw1))
            depthw1 = depthw1 * 2 / 3
            depthw2 = aligned_depth_frame.get_distance(int(xw2),int(yw2))
            depthw2 = depthw2 * 2 / 3
            
            # Convert from Pixel Coordinates (and actual depth) to RealWorld Coordinates (didn't do this for all of the colors yet, just for red, black, purple, yellow, pink, green, and orange (arbitrary points selected))
            coordg1 = [float(xg1),float(yg1)]
            coordg2 = [float(xg2),float(yg2)]
            
            coordb1 = [float(xb1),float(yb1)]
            coordb2 = [float(xb2),float(yb2)]
            
            coordy1 = [float(xy1),float(yy1)]
            coordy2 = [float(xy2),float(yy2)]
            
            coordr1 = [float(xr1),float(yr1)]
            coordr2 = [float(xr2),float(yr2)]
            
            coordo1 = [float(xo1),float(yo1)]
            coordo2 = [float(xo2),float(yo2)]
            
            coordp1 = [float(xp1),float(yp1)]
            coordp2 = [float(xp2),float(yp2)]
            
            coordu1 = [float(xu1),float(yu1)]
            coordu2 = [float(xu2),float(yu2)]
            
            coordt1 = [float(xt1),float(yt1)]
            coordt2 = [float(xt2),float(yt2)]
            
            coordw1 = [float(xw1),float(yw1)]
            coordw2 = [float(xw2),float(yw2)]

            
            pointg1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordg1, depthg1)
            pointg2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordg2, depthg2)
            
            pointb1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordb1, depthb1)
            pointb2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordb2, depthb2)
            
            pointy1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordy1, depthy1)
            pointy2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordy2, depthy2)
            
            pointr1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordr1, depthr1)
            pointr2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordr2, depthr2)
            
            pointo1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordo1, deptho1)
            pointo2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordo2, deptho2)
            
            pointp1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordp1, depthp1)
            pointp2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordp2, depthp2)
            
            pointu1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordu1, depthu1)
            pointu2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordu2, depthu2)
            
            pointt1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordt1, deptht1)
            pointt2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordt2, deptht2)
            
            pointw1 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordw1, depthw1)
            pointw2 = rs.rs2_deproject_pixel_to_point(depth_intrin, coordw2, depthw2)
            
            # Fixes problem if there is only one dot of a color visible
            if pointg1 == pointg2:
                depthg2 = 0
                coordg2 = [0,0]
                pointg2 = [0,0,0]
                badframe = 1
            if pointb1 == pointb2:
                depthb2 = 0
                coordb2 = [0,0]
                pointb2 = [0,0,0]
                badframe = 1
            if pointy1 == pointy2:
                depthy2 = 0
                coordy2 = [0,0]
                pointy2 = [0,0,0]
                badframe = 1
            if pointr1 == pointr2:
                depthr2 = 0
                coordr2 = [0,0]
                pointr2 = [0,0,0]
                badframe = 1
            if pointo1 == pointo2:
                deptho2 = 0
                coordo2 = [0,0]
                pointo2 = [0,0,0]
                badframe = 1
            if pointp1 == pointp2:
                depthp2 = 0
                coordp2 = [0,0]
                pointp2 = [0,0,0]
                badframe = 1
            if pointu1 == pointu2:
                depthu2 = 0
                coordu2 = [0,0]
                pointu2 = [0,0,0]
                badframe = 1
            if pointt1 == pointt2:
                deptht2 = 0
                coordt2 = [0,0]
                pointt2 = [0,0,0]
                badframe = 1
            if pointw1 == pointw2:
                depthw2 = 0
                coordw2 = [0,0]
                pointw2 = [0,0,0]
                badframe = 1
            
            if badframe == 1 or depthg1 == 0 or depthg2 == 0 or depthb1 == 0 or depthb2 == 0 or depthy1 == 0 or depthy2 == 0 or depthr1 == 0 or depthr2 == 0 or deptho1 == 0 or deptho2 == 0 or depthp1 == 0 or depthp2 == 0 or depthu1 == 0 or depthu2 == 0 or deptht1 == 0 or deptht2 == 0:
                print("\nPlease adjust frame so all dots are visible!")
                badframe = 1
            
            # Print the current frame # to correspond to Point Cloud files (# will be repeated if the Point Cloud files are not saved)
            if ryp == 8:
                print("Frame #" + str(framePC) + ":")
                
            # Displays depths and center pixel coordinates
            #print("[Depth (m)]" + "\tGreen: " + str(depthg) + "\tBlue: " + str(depthb) + "\tYellow: " + str(depthy) + "\tRed: " + str(depthr) + "\tBlack: " + str(depthk) + "\n\t\tOrange: " + str(deptho) + "\tPink: " + str(depthp) + "\tPurple: " + str(depthu) + "\tTurquoise: " + str(deptht) + "\tGrellow: " + str(depthw))
            
            print("[Depth (m)]" + "\n[Lunate (wpb/k)] " + "\tGrellow: " + str(depthw1) + "\tPink: " + str(depthp1) + "\tBlue: " + str(depthb1) + "\n[Capitate (uyg/k)] " + "\tPurple: " + str(depthu1) + "\tYellow: " + str(depthy1) + "\tGreen: " + str(depthg1) + "\n[Hamate (tro/i)] " + "\tTurquoise: " + str(deptht2) + "\tRed: " + str(depthr2) + "\tOrange: " + str(deptho2) + "\n[Scaphoid (tor/k)] " + "\tTurquoise: " + str(deptht1) + "\tOrange: " + str(deptho1) + "\tRed: " + str(depthr1) + "\n[Trapezoid (wpb/i)] " + "\tGrellow: " + str(depthw2) + "\tPink: " + str(depthp2) + "\tBlue: " + str(depthb2) + "\n[Triquetrum (uyg/i)] " + "\tPurple: " + str(depthu2) + "\tYellow: " + str(depthy2) + "\tGreen: " + str(depthg2) + "\n")
            
            #print("[Coord (pixel)]" + "\tGreen: " + str(centerg) + "\tBlue: " + str(centerb) + "\tYellow: " + str(centery) + "\tRed: " + str(centerr) + "\tBlack: " + str(centerk) + "\n\t\tOrange: " + str(centero) + "\tPink: " + str(centerp) + "\tPurple: " + str(centeru) + "\tTurquoise: " + str(centert) + "\tGrellow: " + str(centerw))
            
            # Calculate Roll values
            rollLunate = roll(pointw1, pointp1, pointb1, lunateBaseline)
            rollCapitate = roll(pointu1, pointy1, pointg1, capitateBaseline)
            rollHamate = roll(pointt2, pointr2, pointo2, hamateBaseline)
            rollScaphoid = roll(pointt1, pointo1, pointr1, scaphoidBaseline)
            rollTrapezoid = roll(pointw2, pointp2, pointb2, trapezoidBaseline)
            rollTriquetrum = roll(pointu2, pointy2, pointg2, triquetrumBaseline)
            
            # Calculate Yaw values
            yawLunate = yaw(pointw1, pointp1, pointb1, lunateBaseline)
            yawCapitate = yaw(pointu1, pointy1, pointg1, capitateBaseline)
            yawHamate = yaw(pointt2, pointr2, pointo2, hamateBaseline)
            yawScaphoid = yaw(pointt1, pointo1, pointr1, scaphoidBaseline)
            yawTrapezoid = yaw(pointw2, pointp2, pointb2, trapezoidBaseline)
            yawTriquetrum = yaw(pointu2, pointy2, pointg2, triquetrumBaseline)
            
            # Calculate Pitch values
            pitchLunate = pitch(pointw1, pointp1, pointb1, lunateBaseline)
            pitchCapitate = pitch(pointu1, pointy1, pointg1, capitateBaseline)
            pitchHamate = pitch(pointt2, pointr2, pointo2, hamateBaseline)
            pitchScaphoid = pitch(pointt1, pointo1, pointr1, scaphoidBaseline)
            pitchTrapezoid = pitch(pointw2, pointp2, pointb2, trapezoidBaseline)
            pitchTriquetrum = pitch(pointu2, pointy2, pointg2, triquetrumBaseline)
            
            if ryp == 8:
                if rollLunate != 0 and rollCapitate != 0 and rollHamate != 0 and rollScaphoid != 0 and rollTrapezoid != 0 and rollTriquetrum != 0 and yawLunate != 0 and yawCapitate != 0 and yawHamate != 0 and yawScaphoid != 0 and yawTrapezoid != 0 and yawTriquetrum != 0 and pitchLunate != 0 and pitchCapitate != 0 and pitchHamate != 0 and pitchScaphoid != 0 and pitchTrapezoid != 0 and pitchTriquetrum != 0 and badframe == 0: 
                    # Display Roll/Yaw/Pitch values
                    print("Lunate:")
                    print("[Roll]\t" + str(rollLunate) + " degrees")
                    print("[Yaw]\t" + str(yawLunate) + " degrees")
                    print("[Pitch]\t" + str(pitchLunate) + " degrees" + "\n")

                    print("Capitate:")
                    print("[Roll]\t" + str(rollCapitate) + " degrees")
                    print("[Yaw]\t" + str(yawCapitate) + " degrees")
                    print("[Pitch]\t" + str(pitchCapitate) + " degrees" + "\n")

                    print("Hamate:")
                    print("[Roll]\t" + str(rollHamate) + " degrees")
                    print("[Yaw]\t" + str(yawHamate) + " degrees")
                    print("[Pitch]\t" + str(pitchHamate) + " degrees" + "\n")

                    print("Scaphoid:")
                    print("[Roll]\t" + str(rollScaphoid) + " degrees")
                    print("[Yaw]\t" + str(yawScaphoid) + " degrees")
                    print("[Pitch]\t" + str(pitchScaphoid) + " degrees" + "\n")

                    print("Trapezoid:")
                    print("[Roll]\t" + str(rollTrapezoid) + " degrees")
                    print("[Yaw]\t" + str(yawTrapezoid) + " degrees")
                    print("[Pitch]\t" + str(pitchTrapezoid) + " degrees" + "\n")

                    print("Triquetrum:")
                    print("[Roll]\t" + str(rollTriquetrum) + " degrees")
                    print("[Yaw]\t" + str(yawTriquetrum) + " degrees")
                    print("[Pitch]\t" + str(pitchTriquetrum) + " degrees" + "\n")
                else:
                    print("Please adjust frame so all dots are visible!\n")
                    badframe = 1
            
            # Reset all coordinates to (0,0)
            xk = yk = xi = yi = 0
            xg1 = yg1 = xb1 = yb1 = xy1 = yy1 = xr1 = yr1 = xo1 = yo1 = xp1 = yp1 = xu1 = yu1 = xt1 = yt1 = xw1 = yw1 = 0
            xg2 = yg2 = xb2 = yb2 = xy2 = yy2 = xr2 = yr2 = xo2 = yo2 = xp2 = yp2 = xu2 = yu2 = xt2 = yt2 = xw2 = yw2 = 0
            
            # Create Point Cloud files from the first frame that has Roll/Yaw/Pitch displayed with the baseline correction, then each subsequent frame
            if ryp == 8:
                #if xg1 == 0 or yg1 == 0 or xb1 == 0 or yb1 == 0 or xy1 == 0 or yy1 == 0 or xr1 == 0 or yr1 == 0 or xo1 == 0 or yo1 == 0 or xp1 == 0 or yp1 == 0 or xu1 == 0 or yu1 == 0 or xt1 == 0 or yt1 == 0 or xw1 == 0 or yw1 == 0 or xg2 == 0 or yg2 == 0 or xb2 == 0 or yb2 == 0 or xy2 == 0 or yy2 == 0 or xr2 == 0 or yr2 == 0 or xo2 == 0 or yo2 == 0 or xp2 == 0 or yp2 == 0 or xu2 == 0 or yu2 == 0 or xt2 == 0 or yt2 == 0 or xw2 == 0 or yw2 == 0:
                if badframe == 0:
                    createPC(pointg1, pointg2, pointb1, pointb2, pointy1, pointy2, pointr1, pointr2, pointo1, pointo2, pointp1, pointp2, pointu1, pointu2, pointt1, pointt2, pointw1, pointw2, "frame" + str(framePC) + "lunate" + dtFix + ".ply", "frame" + str(framePC) + "capitate" + dtFix + ".ply", "frame" + str(framePC) + "hamate" + dtFix + ".ply", "frame" + str(framePC) + "scaphoid" + dtFix + ".ply", "frame" + str(framePC) + "trapezoid" + dtFix + ".ply", "frame" + str(framePC) + "triquetrum" + dtFix + ".ply")
            
            if ryp == 8 and badframe == 0:
                f = open('frame' + str(framePC) + 'data' + dtFix + '.txt', 'w')
                f.write("[Lunate (wpb/k), roll, yaw, pitch]" + "\n")
                f.write(str(depthw1) + "\n")
                f.write(str(depthp1) + "\n")
                f.write(str(depthb1) + "\n")
                f.write(str(rollLunate) + "\n")
                f.write(str(yawLunate) + "\n")
                f.write(str(pitchLunate) + "\n")
                f.write("[Capitate (uyg/k), roll, yaw, pitch]" + "\n")
                f.write(str(depthu1) + "\n")
                f.write(str(depthy1) + "\n")
                f.write(str(depthg1) + "\n")
                f.write(str(rollCapitate) + "\n")
                f.write(str(yawCapitate) + "\n")
                f.write(str(pitchCapitate) + "\n")
                f.write("[Hamate (tro/i), roll, yaw, pitch]" + "\n")
                f.write(str(deptht2) + "\n")
                f.write(str(depthr2) + "\n")
                f.write(str(deptho2) + "\n")
                f.write(str(rollHamate) + "\n")
                f.write(str(yawHamate) + "\n")
                f.write(str(pitchHamate) + "\n")
                f.write("[Scaphoid (tor/k), roll, yaw, pitch]" + "\n")
                f.write(str(deptht1) + "\n")
                f.write(str(deptho1) + "\n")
                f.write(str(depthr1) + "\n")
                f.write(str(rollScaphoid) + "\n")
                f.write(str(yawScaphoid) + "\n")
                f.write(str(pitchScaphoid) + "\n")
                f.write("[Trapezoid (wpb/i), roll, yaw, pitch]" + "\n")
                f.write(str(depthw2) + "\n")
                f.write(str(depthp2) + "\n")
                f.write(str(depthb2) + "\n")
                f.write(str(rollTrapezoid) + "\n")
                f.write(str(yawTrapezoid) + "\n")
                f.write(str(pitchTrapezoid) + "\n")
                f.write("[Triquetrum (uyg/i), roll, yaw, pitch]" + "\n")
                f.write(str(depthu2) + "\n")
                f.write(str(depthy2) + "\n")
                f.write(str(depthg2) + "\n")
                f.write(str(rollTriquetrum) + "\n")
                f.write(str(yawTriquetrum) + "\n")
                f.write(str(pitchTriquetrum))
                f.close()
                
                framePC += 1
            
            ptrue += 1
            pfalse = 1
        elif ptrue == 2:#Prints all information for ten frames every second (while any color is in frame)
            ptrue = 1
            pfalse = 1
        else:
            ptrue += 1
            pfalse = 1
    
    """# Troubleshooting
    print(deptho1, deptho2, deptht1, deptht2, depthr1, depthr2)
    print(aligned_depth_frame.get_distance(int(xt1),int(yt1)), coordt1)"""
 
    # if we are not using a video file, stop the camera video stream
#if not args.get("video", False):
    #vs.stop()
    #pipeline.stop()
 
    # otherwise, release the camera
#else:
    #vs.release()

if args.get("video", False):
    vs.release()
 
    # close all windows

# Print stop time for log
dt = datetime.now()
print("Time Stopped: " + str(dt))

cv2.destroyAllWindows()
                  
