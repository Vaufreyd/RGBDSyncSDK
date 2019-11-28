# MobileRGBD Reader (for ROS)

## Installation and compilation

To install this package into your catkin environment, just execute a clone operation of the full RGBD Sync SDK in your *src* folder from your catkin workspace using the following command:

    $> git clone --recursive https://github.com/Vaufreyd/RGBDSyncSDK.git

Then, you can compile it using catkin_make or catkin_build.

## Lauch reader program

The program is dedicated to read a single folder of data. It uses a precomputed matrix to faster compute the 3D point cloud from the depth information (this file is part of this repository).

    $> roslaunch mobile_rgbd_reader mobile_rgbd_reader.launch DataPath:=/Path/To/FolderToOneRecord/ DepthToCameraPath:=/Path/To/DepthToCameraTable.raw

Note that you need to install 7zip in your system to be able to read directly compressed data (you will see logs of 7zip in the console when executing the program).

## RGBD Sync SDK

This RGBD Sync SDK is designed for file access to data (robot information, depth, infrared, video, bodies and faces from a Kinect2, ...) and easy way to read them synchronously
from separate sources almost like in ROS bag. 

The RGBD Sync SDK has been used to record the [MobileRGBD corpus](http://MobileRGBD.inrialpes.fr/). It is available for people who want
to work on this corpus, and anyone who need to use it within the term of the LICENSE.


## Licensing

RGBD Sync SDK and submodules are free software; you can redistribute and/or modify them under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
[Consult the license on the FSF website](http://www.gnu.org/licenses/lgpl-3.0.txt).

If you are a researcher and this software helps you, please cite our publication on MobileRGBD:  
+ *MobileRGBD, An Open Benchmark Corpus for mobile RGB-D Related Algorithms*, Dominique Vaufreydaz, Amaury N&egrave;gre,
13th International Conference on Control, Automation, Robotics and Vision, Dec 2014, Singapore, 2014. [(go to author version)](https://hal.inria.fr/hal-01095667)

Copyright (c) 2015-2019, University of Grenoble Alpes and Inria, All rights reserved.
Dominique Vaufreydaz <dominique.vaufreydaz@inria.fr> 