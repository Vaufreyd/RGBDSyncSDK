/**
 * @file mobile_rgbd_reader.cpp
 * @ingroup ROS
 * @author Jean-Alix David, Inria
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @author based on initial version by Amaury Nègre, CNRS, Inria (2014)
 * @copyright All right reserved.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>

#include <rosgraph_msgs/Clock.h>

#include <DrawCameraView.h>
#include <DrawDepthView.h>

#include <stdio.h>
#include <iostream>
#include <fstream>

#define WIDTH 1920
#define HEIGHT 1080

// Omiscid stuff
#include <System/TypedMemoryBuffer.h>

// Pcl stuff
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#define LocalisationFileName "/robulab/Localization.timestamp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

	try
	{
		// ROS init and NodeHandle creation
		ros::init(argc, argv, "MobileRGBD_reader");

		ros::NodeHandle nh;

		// Get param for data folder path and DepthToCameraTable.raw path
		string datapath, filepath;
		ros::param::get("DataPath", datapath);
		cout << datapath << endl;
		ros::param::get("DepthToCameraTablePath", filepath);
		cout << filepath << endl;

		// Read DapthToCameraTable.raw for pointcloud computation
		const char *c = filepath.c_str();
		Omiscid::TypedMemoryBuffer<MobileRGBD::Kinect2::PointF> DepthToCamera;
		DepthToCamera.SetNewNumberOfElementsInBuffer(MobileRGBD::Kinect2::DepthWidth * MobileRGBD::Kinect2::DepthHeight);
		FILE *fdtc = fopen(c, "rb");
		if ( fread((MobileRGBD::Kinect2::PointF *)DepthToCamera, MobileRGBD::Kinect2::DepthWidth * MobileRGBD::Kinect2::DepthHeight * 8, 1, fdtc) != 1)
		{
			fprintf(stderr, "Unable to read DepthToCameraTable.raw\n");
			return -1;
		}
		fclose(fdtc);

		// Read camera parameters file to get camera model
		camera_info_manager::CameraInfoManager cim(nh, "kinect2_rgb");
		cim.loadCameraInfo("package://mobile_rgbd_reader/config/kinect2_rgb.ini");
		// sensor_msgs::CameraInfo cameraInfo = cim.getCameraInfo();
		sensor_msgs::CameraInfoPtr cameraInfoPtr(new sensor_msgs::CameraInfo(cim.getCameraInfo()));;
		image_geometry::PinholeCameraModel cam_model; // init cam_model
		cam_model.fromCameraInfo(cameraInfoPtr);

		// Init publishers
		image_transport::ImageTransport it(nh);
		image_transport::CameraPublisher camera_pub = it.advertiseCamera("kinect/rgb/image", 1);
		image_transport::Publisher depth_pub = it.advertise("kinect/depth/image", 1);

		ros::Publisher clockPub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
		ros::Publisher pointcloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("kinect/depth/points", 1);
		ros::Publisher colouredcloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("kinect/depth/colouredpoints", 1);

		cv_bridge::CvImagePtr cv_ptr;

		KinectImageConverter imageConverter;

		// Scan data folder name for camera tf and speed
		int angle, height;
		float speed;
		char *str;
		str = strrchr(const_cast<char *>(datapath.c_str()), '/');
		cout << "str: " << str << endl;
		int res = sscanf(str, "%*[^_]_%*d_%d_%d_%*[^_]_%f", &height, &angle, &speed);
		cout << "height " << height << "; angle = " << angle << "; speed = " << speed << endl;

		// Tf for kinect position
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Transform tfRobotToKinect;

		tfRobotToKinect.setOrigin(tf::Vector3(0.22, 0.0, height/100.));
		tf::Quaternion qRobotToKinect;
		qRobotToKinect.setRPY(0, -angle * M_PI / 180., 0);
		tfRobotToKinect.setRotation(qRobotToKinect);

		// Prepare Drawer for 2D kinect views
		MobileRGBD::Kinect2::DrawCameraView BGRReader(datapath);
		MobileRGBD::Kinect2::DrawDepthView DepthReader(datapath);

		// Get synchronized localisation file (contains x,y,theta)
		datapath = datapath + LocalisationFileName;
		MobileRGBD::ReadTimestampFile Localization(datapath);

		cv_bridge::CvImage cv_img, cv_depth;
		cv_img.encoding = sensor_msgs::image_encodings::BGR8;
		cv_depth.encoding = sensor_msgs::image_encodings::MONO16;

		bool quit = false;

		// Localization of the robot
		float x, y, o;
		x = y = o = 0.0;

		// DepthCloud: pcl cloud to fill
		pcl::PointCloud<pcl::PointXYZ>::Ptr DepthCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColouredDepthCloud(new pcl::PointCloud<pcl::PointXYZRGB>); //New point cloud for the PCD People app

		while (DepthReader.GetNextTimestamp() && ros::ok() && !quit)
		{
			// Do we have all the data?
			if (BGRReader.LoadFrame(DepthReader.CurrentTimestamp) == false || DepthReader.LoadFrame(DepthReader.CurrentTimestamp) == false)
			{
				continue;
			}

			double timestamp = DepthReader.CurrentTimestamp.time + DepthReader.CurrentTimestamp.millitm / 1000.0;
			cv::Mat MatBGR(MobileRGBD::Kinect2::CamHeight, MobileRGBD::Kinect2::CamWidth, CV_8UC3);
			cv::Mat MatDepth(MobileRGBD::Kinect2::DepthHeight,
							 MobileRGBD::Kinect2::DepthWidth, CV_16UC1,
							 DepthReader.FrameBuffer);

			// Compute pcl point cloud from Depth data, first get point on uint16 depth data starting at end
			uint16_t *LocalData = &((uint16_t *)DepthReader.FrameBuffer)[MobileRGBD::Kinect2::DepthHeight * MobileRGBD::Kinect2::DepthWidth - 1];

			// Draw frame
			BGRReader.Draw(MatBGR, BGRReader.FrameBuffer, 1);

			// empty DepthClouds
			DepthCloud->clear();
			ColouredDepthCloud->clear();
			int NbColoredPoints = 0;
			int NbWhitePoints = 0;

			// Loop over local data
			for (int y = 0; y < MobileRGBD::Kinect2::DepthHeight; y++) //y--)
			{
				for (int x = 0; x < MobileRGBD::Kinect2::DepthWidth; x++) //x--)
				{
					float rx, rz, ry;

					if (*LocalData >= 500) // for data at more than 50 cm from the Depth sensor only
					{
						int CurPixel = MobileRGBD::Kinect2::DepthWidth * y + x; // compute pixel index

						// Compute x,z,y data. Z is in front of depth sensor, let's say its x
						rz = (*LocalData) / 1000.0f; //The depth value from the depth in meters // X

						// and compute yz accordingly using parameter data
						rx = rz * DepthToCamera[CurPixel].X; // Y
						ry = rz * DepthToCamera[CurPixel].Y; // Z

						// cout << "rx: " << rx << ", ry: " << ry <<  ", rz: " << rz << endl;

						cv::Point pixelpoint = cam_model.project3dToPixel(cv::Point3d(-rx, ry, rz));

						// cout << "x: " << pixelpoint.x << ", y: " << pixelpoint.y << endl;

						Vec3b bgr_values;
						if (pixelpoint.x >= 0 && pixelpoint.x < MatBGR.cols &&
							pixelpoint.y >= 0 && pixelpoint.y < MatBGR.rows)
						{
							bgr_values = MatBGR.at<Vec3b>(pixelpoint.y, pixelpoint.x);
							NbColoredPoints++;
						}
						else
						{
							bgr_values[2] = bgr_values[1] = bgr_values[0] = 255;
							NbWhitePoints++;
						}

						pcl::PointXYZ Point;
						pcl::PointXYZRGB ColourPoint;
						Point.x = rz;
						Point.y = -rx;
						Point.z = -ry;

						ColourPoint.x = rz;
						ColourPoint.y = -rx;
						ColourPoint.z = -ry;
						ColourPoint.b = bgr_values[0];
						ColourPoint.g = bgr_values[1];
						ColourPoint.r = bgr_values[2];

						DepthCloud->push_back(Point);
						ColouredDepthCloud->push_back(ColourPoint);
					}

					LocalData--;
				}
			}

			// Here we have the pcl clouds
			// Get and scan localisation
			Localization.GetDataForTimestamp(DepthReader.CurrentTimestamp);

			// if sscanf failed, previous Localization is used
			sscanf(Localization.DataBuffer, "{\"x\":%f,\"y\":%f,\"o\":%f}", &x, &y, &o);

			// Publications to ROS

			// publish Clock
			rosgraph_msgs::Clock clockMsg;
			clockMsg.clock = ros::Time(timestamp);
			clockPub.publish(clockMsg);

			// set tf world to robot
			transform.setOrigin(tf::Vector3(x, y, 0));
			tf::Quaternion q;
			q.setRPY(0, 0, o);
			transform.setRotation(q);

			// Send transform from world to robot and to Kinect
			br.sendTransform(tf::StampedTransform(transform, ros::Time(timestamp), "world", "robot"));
			br.sendTransform(tf::StampedTransform(tfRobotToKinect, ros::Time(timestamp), "robot", "kinect2"));

			// publish images
			cv::flip(MatBGR,cv_img.image,1);
			cv_img.header.stamp = ros::Time(timestamp);
			cv_img.header.frame_id = "kinect2_rgb";

			cv::flip(MatDepth,cv_depth.image,1);
			cv_depth.header.stamp = ros::Time(timestamp);
			cv_depth.header.frame_id = "kinect2";

			cameraInfoPtr->header.stamp = cv_img.header.stamp;
			cameraInfoPtr->header.frame_id = "kinect2_rgb";

			camera_pub.publish(cv_img.toImageMsg(), cameraInfoPtr);
			depth_pub.publish(cv_depth.toImageMsg());

			// publish clouds
			DepthCloud->header.frame_id = "kinect2";
			pcl_conversions::toPCL(ros::Time(timestamp), DepthCloud->header.stamp);
			pointcloudPub.publish(DepthCloud);

			ColouredDepthCloud->header.frame_id = "kinect2";
			pcl_conversions::toPCL(ros::Time(timestamp), DepthCloud->header.stamp);
			colouredcloudPub.publish(ColouredDepthCloud);
		}
	}
	catch ( ros::Exception &e )
    {
     ROS_ERROR("Error occured: %s ", e.what());
    }
	{
		cout << "Exception. Damn" << endl;
	}

	return 0;
}
