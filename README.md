![logo3DBTThttps://cdn.pbrd.co/images/HzjQAxU.png](https://cdn.pbrd.co/images/HzjSbKT.png "3D Bone Tracking Tool Logo")
# 3D Bone Tracking Tool

The 3D Bone Tracking Tool is a pair of Python 2.7 programs that interface with Intel® RealSense™ depth cameras (D400 series). The purpose of these programs is to track the movement of bones in 3D space - tracking both real-world (x,y,z) coordinates and angles of rotation (roll, yaw, and pitch) using colored dots.

|Program|Description|
|:------|:----------|
|manualframe.py|Allows the user to save data from specific frames manually after calibrating (stop-motion tracking)|
|speedcam.py|Automatically saves data from every frame (10 fps) after calibrating (real-time tracking)|

Each program can be run directly from the shell.

## Download

The 3D Bone Tracking Tool programs can be installed [here](https://github.com/nateroblin33/3d-bone-tracking-tool/tree/master/files).

In order to run, they will need to be in the same directory as the [pyrealsense2](https://pypi.python.org/pypi/pyrealsense2) (instructions for installation [here](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python), or **copy pyrealsense2.pyd and pyrealsense2.cp36-win32.pyd from Program Files (x86)/Intel RealSense SDK 2.0/bin/x86 into your directory**) and [imutils](https://github.com/jrosebr1/imutils) ("Clone or download" as a .zip folder, then extract and move the "imutils" *subfolder* to the directory) packages, as well as [plyfile.py](https://github.com/dranjan/python-plyfile/blob/master/plyfile.py).

## Platform & Python Version

*Platform:* The 3D Bone Tracking Tool programs work exclusively on Windows 10, restricted by the requirements for the Intel® RealSense™ D400 series.

*Python:* The 3D Bone Tracking Tool programs work exclusively with Python 2.7 (32-bit), restricted by the requirements for the Intel® RealSense™ D400 series.

## Running the Programs

1. Open a Terminal/Shell.
2. Change the directory to where the files are housed (type ***cd DIRECTORY_NAME***).
3. Run the python file (either type ***python manualframe.py*** or ***python speedcam.py***).
4. Press "c" to calibrate angle measurements when all colored dots are being tracked correctly.
5. *For **manualframe.py**, press "n" to save data from a frame. If all dots are being tracked, the txt file and corresponding Point Cloud file will save correctly (if not, the frame counter won't increase and you can press "n" again to retake that frame). The next time "n" is pressed, the frame counter will increase and a new set of frame data will be saved. If the frame saves correctly but the data shows that not all of the dots were correctly tracked (in the right places), press "f" to retake that frame (overwrite a bad frame), then continue once the frame looks right.*
6. *For **speedcam.py**, frame data will save automatically at 10 fps. If not all dots are being tracked during a frame, that frame will automatically be discarded and the frame counter will not increase until the next success frame.*
7. Press "q" to quit the program and stop the RealSense™ stream.

## Functions and Parameters defined within the programs

*Vector Normailzation function:*

    def normalize(v):
        return v / norm
        
    #v = Initial Vector ([Δx,Δy,Δz])
    #v / norm = Normalized Vector ([Δx,Δy,Δz])

*Angle Calculation functions:*

Roll:
  
    def roll(p1, p2, p3, baseline):
        return roll
        
    #roll = Roll angle relative to baseline (in degrees)
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])
    #p3 = Point 3 ([x,y,z])
    #baseline = Baseline roll angle (in degrees)
Yaw:
  
    def yaw(p1, p2, p3, baseline):
        return yaw
        
    #yaw = Yaw angle relative to baseline (in degrees)
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])
    #p3 = Point 3 ([x,y,z])
    #baseline = Baseline yaw angle (in degrees)
Pitch:
  
    def pitch(p1, p2, p3, baseline):
        return pitch
        
    #pitch = Pitch angle relative to baseline (in degrees)
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])
    #p3 = Point 3 ([x,y,z])
    #baseline = Baseline pitch angle (in degrees)

*Euclidean Distance Calculation function:*

    def euclideanDist(p1, p2):
        return dist
        
    #dist = Euclidean distance between two points
    #p1 = Point 1 ([x,y,z])
    #p2 = Point 2 ([x,y,z])

*Point Cloud File Creation function:*

    def createPC(pointg1, pointg2, pointb1, pointb2, pointy1, pointy2, pointr1, pointr2, pointo1, pointo2, pointp1, pointp2, pointu1, pointu2, pointt1, pointt2, pointw1, pointw2, bone1FileName, bone2FileName, bone3FileName, bone4FileName, bone5FileName, bone6FileName):
    
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

## Color-Code Chart (for dots)
|Color|1 Letter Abbreviation|HSV Lower|HSV Upper|Number Code (Black and White only)|
|:----|:--------------------|:--------|:--------|:---------------------------------|
|Black|k|(0,0,0)|(180,255,49)|1|
|White|i|(0,0,240)|(180,20,255)|2|
|Green|g|(61,86,50)|(80,255,255)||
|Blue|b|(101,86,50)|(115,255,255)||
|Yellow|y|(16,108,50)|(40,255,255)||
|Red|r|(171,86,50)|(180,255,255)||
|"  "|" "|(0,100,50)|(6,255,255)||
|Orange|o|(7,100,50)|(15,255,255)||
|Pink|p|(155,30,50)|(170,255,255)||
|Purple|u|(116,51,40)|(154,255,255)||
|Turquoise|t|(81,100,60)|(100,255,255)||
|Grellow (Green-Yellow)|w|(41,100,50)|(60,255,255)||

*Note: The HSV scale used in the programs is H: 0-180; S: 0-255; V: 0-255. This is different from the traditional scale for HSV, which is H: 0-360; S: 0-100%; V: 0-100%.*

*Note: Red has two separate HSV ranges because red is at both the upper and lower ends of the [hue spectrum](https://upload.wikimedia.org/wikipedia/commons/a/ad/HueScale.svg).*

## Example Uses
*Note: Animated GIFs are composed of Point Cloud files rendered in MeshLab*

### Carpal Bones
![HandChart](https://cdn.pbrd.co/images/Hzsig6o.png "Carpal Color Dots Chart")
This example shows how each program tracks the carpal (wrist) bones when the a hand is closed and opened. Bones tracked are (clockwise from top right) the trapezoid, capitate, hamate, triquetrum, lunate, and scaphoid.

#### manualframe.py
![4frameanimation](https://cdn.pbrd.co/images/HzkiM7o.gif "Carpal Animation - manualframe.py")

#### speedcam.py
![shortRTanimation](https://cdn.pbrd.co/images/HzklWxi.gif "Carpal Animation - speedcam.py")

# Tendon Strain Tool

The Tendon Strain Tool is a Python 2.7 program that interfaces with Intel® RealSense™ depth cameras (D400 series). The purpose of this program is to track the strain in tendons using euclidean distances in 3D space by tracking real-world (x,y,z) coordinates.

|Program|Description|
|:------|:----------|
|tendonstrain.py|Automatically saves data from every frame (1-2 fps) after calibrating (real-time tracking)|

The program can be run directly from the shell.

## Download

The Tendon Strain Tool program can be installed [here](https://github.com/nateroblin33/3d-bone-tracking-tool/tree/master/files).

In order to run, it will need to be in the same directory as the [pyrealsense2](https://pypi.python.org/pypi/pyrealsense2) (instructions for installation [here](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python), or **copy pyrealsense2.pyd and pyrealsense2.cp36-win32.pyd from Program Files (x86)/Intel RealSense SDK 2.0/bin/x86 into your directory**) and [imutils](https://github.com/jrosebr1/imutils) ("Clone or download" as a .zip folder, then extract and move the "imutils" *subfolder* to the directory) packages, as well as [plyfile.py](https://github.com/dranjan/python-plyfile/blob/master/plyfile.py).

## Platform & Python Version

*Platform:* The 3D Bone Tracking Tool programs work exclusively on Windows 10, restricted by the requirements for the Intel® RealSense™ D400 series.

*Python:* The 3D Bone Tracking Tool programs work exclusively with Python 2.7 (32-bit), restricted by the requirements for the Intel® RealSense™ D400 series.

## Running the Program

1. Open a Terminal/Shell.
2. Change the directory to where the files are housed (type ***cd DIRECTORY_NAME***).
3. Run the python file (type ***python tendonstrain.py***).
4. Press "c" to calibrate starting lengths when all 3 colored dots (blue, green, yellow) are being tracked correctly.
5. Frame data will save automatically at somewhere between 1 and 2 fps.
6. Press "q" repeatedly to quit the program and stop the RealSense™ stream. (Keep pressing q over and over until the stream stops).

## Example Use

### Chart Generated
![tendonStrainChart](https://cdn.pbrd.co/images/HVGzVEP.png "Tendon Strain Chart")

### Corresponding Data

    [Time]	[Strain 1 (%)]	[Length 1 (mm)]	[Change in Length 1 (mm)]	[Strain 2 (%)]	[Length 2 (mm)]	[Change in Length 2 (mm)]	
    0	100.0	43.11438583485039	0.0	100.0	99.06767232683295	0.0	
    1.10600018501	100.24652456204875	43.22067338570976	0.10628755085936858	100.45502209959167	99.51845212947111	0.45077980263816264	
    3.04400014877	144.70850962040672	62.39018517360374	19.275799338753355	99.32830149433426	98.40223625221577	-0.6654360746171789	
    4.625	99.20344136022072	42.770954469495116	-0.343431365355272	100.62256986807634	99.68443780374442	0.6167654769114739	
    6.1850001812	104.02927045761304	44.851581046275314	1.7371952114249232	100.16841196489415	99.23451414037346	0.16684181354051364	
    7.78600001335	104.79368008025023	45.18115156033784	2.066765725487453	100.4758444153679	99.53908031303516	0.47140798620221025	
