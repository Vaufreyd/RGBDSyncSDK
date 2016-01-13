# RGBD Sync SDK

## RGBD Sync SDK

This RGBD Sync SDK is designed for file access to data (robot information, depth, infrared, video, bodies and faces from a Kinect2, ...) and easy way to read them synchronously
from separate sources almost like in ROS bag. 

The RGBD Sync SDK has been used to record the [MobileRGBD corpus](http://MobileRGBD.inrialpes.fr/). It is available for people who want
to work on this corpus, and anyone who need to use it within the term of the LICENSE.

## Building the program

### Requirements

#### *ffmpeg*

You must install [ffmpeg](https://www.ffmpeg.org/) version 2.4.7 (or later) to your computer in a way you can call `ffprobe` and `ffmpeg` from
the command line. For older version, please read KnownIssues.md.
In a command shell (even in the standard cmd.exe shell on Windows), you can test `ffprobe` and `ffmpeg` to see if it is working.

    $> ffprobe --version  
    $> ffmpeg --version

#### *OpenCV*

You **DO NOT** need to recompile OpenCV to support specific codecs. You just need to install it. This source code
was tested with OpenCV 2.4.10 (Linux/Windows/Mac OSX) and 3.0.0 (on Windows only for this last version.) Please see the [OpenCV web site](http://opencv.org).

#### *cmake*

In all environment, you must install cmake (2.8 or later).

#### *Visual Studio (Windows users Only)*

On Windows, you must install Visual Studio 2013 (Express, for the free version) or later.

### Linux/Mac OSX compilation

On these environments, you need to call `cmake` to create the make file within this folder:

    $> cmake .

After these step is complete, you can call make:

    $> make

### Windows compilation

You must first create a visual studio solution for the program:

    $> cmake . -DOpenCV_FIND_QUIETLY:BOOLEAN=FALSE -G "Visual Studio 12 Win64" "-DOpenCV_DIR:STRING=PATH_TO_YOUR_OPENCV\\opencv\\build"

It will produce you a complete solution named `GenerateVideoFromRecords.sln`. You can open it in order to compile the program.

## Test the program

### Usage

This program is intended to process synchronously all data recorded with RGBDSyncSDK (Kinect2 streams and robot streams). You can find data example on the [MobileRGBD web site](http://MobileRGBD.inrialpes.fr).
You must call the GenerateVideoFromRecords at least with two parameters: a folder name containing data and a rendering choice (-VideoEdit, -RGBOnly, etc...).

If you put several folders, for each one must can provide one or several rendering choices.
It will process **iteratively** all rendering choices. Options can be set anywere
 and several times without any inconvenient
They will be true for further parameters (until the end of the command line as they is no way to unset a parameter).

The rendering types are:
+ *-VideoEdit*: make a composite video with robot sensors (laser range finder, telemeters, localisation and map), RGB, Depth, Body detection and Infrared streams.
+ *-RGBOnly*: Full HD RGB stream.
+ *-DepthOnly*: only depth stream
+ *-IROnly*: only Infrared stream.
+ *-BodyIndexOnly*: body detection stream;
+ *-RobotOnly*: robot sensors (laser range finder, telemeters, localisation and map.

The options are:
+ *-ShowLiveVideo*: Process data in itneractive mode. In this mode you can start, stop processing, skip frames, go to a specific time, etc. For debug purpose.
+ *-DebugMode*: The program will print output in the console. 

    $> Usage: GenerateVideoFromRecords [-ShowLiveVideo] [-DebugMode] <folder1> <rendering_choice_1> [<rendering_choice_2> ...] [<folder2> <rendering_choice_n> [<rendering_choice_n+1> ...] ...
    
### Test on data

First, you must get retrieve a sample of the MobileRGBD corpus. This file is a **480 MiB** `7z` file that contains a static recording of a dummy ([click here to download](http://mobilergbd.inrialpes.fr/RGBDSyncSDK-TestData.7z)).
You just need to decompress it once: you do not need to decompress 7z files from the folder itself (`7z x RGBDSyncSDK-TestData.7z`). Then, run the program:

    $> ./GenerateVideoFromRecords -ShowLiveVideo $PATH_TO_FOLDER/RGBDSyncSDK-TestData -VideoEdit

As we used `-ShowLiveVideo`, we are in interactive mode. Commands are:
- _escape_: stop current rendering (and start next one if any)
- _space_:  play or pause
- _d_: toggle debug mode
- _x_: when paused, go to next frame
- _s_: skip 10 frames
- _t_: go to console and type a timestamp to reach. Generated video will miss data (use it for debugging purpose mode only).

To start processing the video, you must push _space_.
At the end, you can find a `ResVideo` inside `$PATH_TO_FOLDER/RGBDSyncSDK-TestData` folder containing a `VideoEdit_0001.mp4`. If you relaunch the process, a the file number will be increased to prevent erasing previous computation.
  
## Participate!

You can help us finding bugs, proposing new functionalities and more directly on this website! Click on the "New issue" button in the menu to do that.
You can browse the git repository here on GitHub, submit patches and push requests!

## Licensing

RGBD Sync SDK and submodules are free software; you can redistribute and/or modify them under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
[Consult the license on the FSF website](http://www.gnu.org/licenses/lgpl-3.0.txt).

If you are a researcher and this software helps you, please cite our publication on MobileRGBD:  
*MobileRGBD, An Open Benchmark Corpus for mobile RGB-D Related Algorithms*, Dominique Vaufreydaz, Amaury N&egrave;gre,
13th International Conference on Control, Automation, Robotics and Vision, Dec 2014, Singapore, 2014. [(go to author version)](https://hal.inria.fr/hal-01095667)

Copyright (c) 2015, University of Grenoble Alpes and Inria, All rights reserved.
Dominique Vaufreydaz <dominique.vaufreydaz@inria.fr> 

