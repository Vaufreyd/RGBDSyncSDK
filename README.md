# RGBD Sync SDK

## RGBD Sync SDK

This RGBD Sync SDK is designed for file access to data (robot information, depth, infrared, video, bodies and faces from a Kinect2, ...) and easy way to read them synchronously
from separate sources almost like in ROS bag. 

The RGBD Sync SDK has been used to record the [MobileRGBD corpus](http://MobileRGBD.inrialpes.fr/). It is available for people who want
to work on this corpus, and anyone who need to use it within the term of the LICENSE.


## Content

First this repository contains submodules:
+ [_DataManagement_](https://github.com/Vaufreyd/DataManagement): provide classes to handle all data (timestamped files, video files, etc.)
+ [_Drawing_](https://github.com/Vaufreyd/Drawing): classes to draw in OpenCV cv::Mat all data from the corpus. All these classes will resize draws to feat available space.
+ [_Map_](https://github.com/Vaufreyd/Map): dedicated to simple maps build with wall segments.
+ [_Omiscid_](https://github.com/Vaufreyd/Omiscid): Omiscid 3.0b, codename Yggdrasil is a middleware. We only used here its system abstraction layer and its JSON (de)serialization.

On can also find subfolders:
+ [**_mobile_rgbd_reader_**](https://github.com/Vaufreyd/RGBDSyncSDK/tree/master/mobile_rgbd_reader): A ROS package to feed data from MobileRGBD into a ROS architecture.
+ [**__Kinect_**](https://github.com/Vaufreyd/Kinect/tree/2f132dbb1098235230ffc83fb252749b942dcaa3): All classes dedicated to (submodule) Kinect2. Under Windows, there will be classes to read, to record and/or to process data. Under Linux, everything is define to read Kinect2 data from MobileRGBD recordings.
+ [**_GenerateVideoFromRecords_**](https://github.com/Vaufreyd/RGBDSyncSDK/tree/master/GenerateVideoFromRecords): an example to show how to synchronously read data from the corpus and generate a (composite) mp4 video.
+ [**_RecordKinect_**](https://github.com/Vaufreyd/RGBDSyncSDK/tree/master/RecordKinect): a project to make your own records using the Kinect2 device under Windows (RGB, Depth, Infrared, Body, Face, Audio). This project uses native Kinect2 SDK under Windows. Your records will be readable using RGBDSyncSDK under Windows/Linux/Mac OSX.

## Cloning and updating

As we used submodules in our project, in order to clone this repo, you must ask git to work recursively:

    $> git clone --recursive https://github.com/Vaufreyd/RGBDSyncSDK.git
    
For the same reason, in order to update this repo, you must ask git to work recursively with submodules:

    $> git pull --recurse-submodules

## Example

You can start using this SDK using the [_GenerateVideoFromRecords example_](https://github.com/Vaufreyd/RGBDSyncSDK/tree/master/GenerateVideoFromRecords).
    
## Participate!

You can help us finding bugs, proposing new functionalities and more directly on this website! Click on the "New issue" button in the menu to do that.
You can browse the git repository here on GitHub, submit patches and push requests!

## Licensing

RGBD Sync SDK and submodules are free software; you can redistribute and/or modify them under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
[Consult the license on the FSF website](http://www.gnu.org/licenses/lgpl-3.0.txt).

If you are a researcher and this software helps you, please cite our publication on MobileRGBD:  
+ *MobileRGBD, An Open Benchmark Corpus for mobile RGB-D Related Algorithms*, Dominique Vaufreydaz, Amaury N&egrave;gre,
13th International Conference on Control, Automation, Robotics and Vision, Dec 2014, Singapore, 2014. [(go to author version)](https://hal.inria.fr/hal-01095667)

Copyright (c) 2015, University of Grenoble Alpes and Inria, All rights reserved.
Dominique Vaufreydaz <dominique.vaufreydaz@inria.fr> 

