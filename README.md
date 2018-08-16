![logo3DBTThttps://cdn.pbrd.co/images/HzjQAxU.png](https://cdn.pbrd.co/images/HzjSbKT.png "3D Bone Tracking Tool Logo")
# 3D Bone Tracking Tool

The 3D Bone Tracking Tool is a pair of Python 2.7 programs that interface with Intel® RealSense™ depth cameras (D400 series). The purpose of these is programs is to track the movement of bones in 3D space - tracking both real-world (x,y,z) coordinates and angles of rotation (roll, yaw, and pitch) using colored dots.

|Program|Description|
|:------|:----------|
|manualframe.py|Allows the user to save data from specific frames manually after calibrating (stop-motion tracking)|
|speedcam.py|Automatically saves data from every frame (10 fps) after calibrating (real-time tracking)|

Each program can be run directly from the shell.

## Download

The 3D Bone Tracking Tool programs can be installed [here](https://github.com/nateroblin33/3d-bone-tracking-tool/tree/master/files).

In order to run, they will need to be in the same directory as the [pyrealsense2](https://pypi.python.org/pypi/pyrealsense2) (instructions for installation [here](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python)) and [imutils](https://github.com/jrosebr1/imutils) (download as a .zip folder, then extract and move the "imutils" subfolder to the directory) packages, as well as [plyfile.py](https://github.com/dranjan/python-plyfile/blob/master/plyfile.py).

## Platform & Python Version

*Platform:* The 3D Bone Tracking Tool programs work exclusively on Windows 10, restricted by the requirements for the Intel® RealSense™ D400 series.

*Python:* The 3D Bone Tracking Tool programs work exclusively with Python 2.7 (32-bit), restricted by the requirements for the Intel® RealSense™ D400 series.

## Running the Programs

1. Open a Terminal/Shell
2. Change the directory to where the files are housed (type ***cd DIRECTORY_NAME***)
3. Run the python file (either type ***python manualframe.py*** or ***python speedcam.py***)

## Functions and Parameters

*Vector Normailzation function:*

    normalize(v):
        return v / norm
        
    #v = Initial Vector ([x,y,z])
    #v / norm = Normalized Vector ([Δx,Δy,Δz])

*Angle Calculation functions:*

Roll:
  
    roll(p1, p2, p3, baseline):
        return roll
        
    #roll = Roll angle relative to baseline (in degrees)
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])
    #p3 = Point 3 ([x,y,z])
    #baseline = Baseline roll angle (in degrees)
Yaw:
  
    yaw(p1, p2, p3, baseline):
        return yaw
        
    #yaw = Yaw angle relative to baseline (in degrees)
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])
    #p3 = Point 3 ([x,y,z])
    #baseline = Baseline yaw angle (in degrees)
Pitch:
  
    pitch(p1, p2, p3, baseline):
        return pitch
        
    #pitch = Pitch angle relative to baseline (in degrees)
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])
    #p3 = Point 3 ([x,y,z])
    #baseline = Baseline pitch angle (in degrees)

*Euclidean Distance Calculation function:*

    euclideanDist(p1, p2):
        return dist
        
    #dist = Euclidean distance between two points
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])

*Point Cloud File Creation function:*

    createPC(pointg1, pointg2, pointb1, pointb2, pointy1, pointy2, pointr1, pointr2, pointo1, pointo2, pointp1, pointp2, pointu1, pointu2, pointt1, pointt2, pointw1, pointw2, bone1FileName, bone2FileName, bone3FileName, bone4FileName, bone5FileName, bone6FileName):
    
    #pointg1 = Green point on black
    #pointg2 = Green point on white
    #pointb1 = Blue point on black
    #pointb2 = Blue point on white
    #pointy1 = Yellow point on black
    #pointy2 = Yellow point on white
    #pointr1 = Red point on black
    #pointr2 = Red point on white
    #pointo1 = Orange point on black
    #pointo2 = Orange point on white
    #pointp1 = Pink point on black
    #pointp2 = Pink point on white
    #pointu1 = Purple point on black
    #pointu2 = Purple point on white
    #pointt1 = Turquoise point on black
    #pointt2 = Turquoise point on white
    #pointw1 = Grellow (Green-Yellow) point on black
    #pointw2 = Grellow (Green-Yellow) point on white
    
    #bone1FileName = 'bone1FileName.ply'
    #bone2FileName = 'bone2FileName.ply'
    #bone3FileName = 'bone3FileName.ply'
    #bone4FileName = 'bone4FileName.ply'
    #bone5FileName = 'bone5FileName.ply'
    #bone6FileName = 'bone6FileName.ply'

## Example Uses
*Note: Animated GIFs are composed of Point Cloud files rendered in MeshLab*

### Carpal Bones
This example shows how each program tracks the carpal (wrist) bones when the a hand is closed and opened. Bones tracked are (clockwise from top right) the trapezoid, capitate, hamate, triquetrum, lunate, and scaphoid.

#### manualframe.py
![4frameanimation](https://cdn.pbrd.co/images/HzkiM7o.gif "Carpal Animation - manualframe.py")

#### speedcam.py
![shortRTanimation](https://cdn.pbrd.co/images/HzklWxi.gif "Carpal Animation - speedcam.py")
