/**
 * @file Main.cpp
 * @ingroup MobileRGBD
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 * @copyright All right reserved.
 */

#ifndef _FILE_OFFSET_BITS
#define _FILE_OFFSET_BITS  64
#endif

#include <System/ConfigSystem.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <stdio.h>

#include <sys/stat.h>
#if defined WIN32 || defined WIN64
	#include <direct.h>
#endif

#include "Drawing/DrawLaser.h"
#include "Drawing/DrawBodyIndexView.h"
#include "Drawing/DrawCameraView.h"
#include "Drawing/DrawDepthView.h"
#include "Drawing/DrawSkeleton.h"
#include "Drawing/DrawInfraredView.h"
#include "Drawing/DrawFace.h"
#include "Drawing/DrawMap.h"
#include "Drawing/DrawLocalization.h"

#include "DataManagement/ConstantFpsTimestampFromFile.h"
#include "DataManagement/VideoIO.h"


#define VideoSizeFactor 1.0f			/*!< @brief Resize factor for video. */

#include <System/SimpleList.h>

// Using namespaces
using namespace cv;
using namespace MobileRGBD;

// Internally we have also a MobileRGBD::Kinect1 namespace with classes to handle Kinect for Windows 1 devices
using namespace MobileRGBD::Kinect2;

namespace MobileRGBD {

/**
 * @class DrawingPlace Main.cpp
 * @brief Association class to associate a Drawable (thus reading) object for MobileRGBD data and a drawing place (cv::Mat).
 *
 * @author Dominique Vaufreydaz, Grenoble Alpes University, Inria
 */
class DrawingPlace
{
public:
	/** @brief constructor.
	 *
	 * @param _DrawingObject [in] Reference to the DrawingObject
	 * @param _DrawingZone [in] Reference to the _DrawingZone
	 */
	DrawingPlace( Drawable& _DrawingObject, Mat& _DrawingZone ) : DrawingObject(_DrawingObject), DrawingZone(_DrawingZone)
	{
	}

	Drawable& DrawingObject;	/*!< @brief Reference to store the Drawable reference */
	Mat& DrawingZone;			/*!< @brief Reference to store the cv::Mat reference */
};

} // namespace MobileRGBD


static bool ShowVideo = false;		/*!< @brief Shall we show video while processing */
static bool LocalDebug = false;		/*!< @brief Shall we debug */

/** @enum TimestampFlags
	* @brief Define where to put timestamp info in the video
	*/
typedef enum { NO_TIMESTAMP, TIMESTAMP_HIGH, TIMESTAMP_LOW } TimestampFlags;

bool DoFolderVideo( const std::string& Folder, Omiscid::SimpleList<DrawingPlace*>& DrawingObject, cv::Mat& VideoImg, const std::string& Prefix, bool ShowVideo, TimestampFlags TimestampHigh = TIMESTAMP_HIGH );

/** @brief Prepare all data to make a composite video with most of data stream, then call DoFolderVideo function to process it.
 *
 * @param Folder [in] Folder where we can read all data
 * @param ShowVideo [in] do we show video in interactive mode while processing?
 * @param TimestampHigh [in] Flag to provide information about where to put timestamp information within the video 
 * @return true if everything went fine.
*/
bool VideoEditFromFolderData( const std::string& Folder, bool ShowVideo, TimestampFlags TimestampHigh )
{
	int VideoWidth = 0;
	int VideoHeight = DepthHeight;			// Video size
	
	// Create objects to read data from folder
	DrawCameraView CamDrawer( Folder );	
	VideoWidth += 750;
	DrawDepthView DepthDrawer( Folder );		
	VideoWidth += DepthWidth;
	DrawSkeleton SkeletonDrawer( Folder );
	DrawLaserData TelDrawer( Folder );		// VideoWidth += 512;
	DrawInfraredView InfraredDrawer( Folder );
	DrawBodyIndexView BodyIndexDrawer( Folder );
	DrawLocalization LocalizationDrawer( Folder );
	DrawMap MapDrawer( Folder );
	DrawFace FaceDrawer( Folder, 0, 0, false );

	// Create cv::Mat to draw all synchronized data
	CvSize VideoSize = cvSize(VideoWidth, VideoHeight*2);
	Mat VideoImg( VideoSize, CV_8UC3 );

	// subframe all region for all data streams
	Mat ImgInfrared( VideoImg, Rect(0,0,512,VideoHeight) );
	Mat ImgCamera( VideoImg, Rect(512,0,750,VideoHeight) );
	// Compute ratio for ImgBodyIndex
	int BodyIndexWidth  = VideoWidth-VideoWidth/2-512;
	int BodyIndexHeight = (int)(((float)BodyIndexWidth/(float)DepthWidth)*(float)DepthHeight);
	Mat ImgBodyIndex( VideoImg, Rect(0,VideoHeight,BodyIndexWidth,BodyIndexHeight) );
	Mat ImgDepth( VideoImg, Rect(VideoWidth/2-512,VideoHeight,512,VideoHeight) );
	Mat ImgTelemeter( VideoImg, Rect(VideoWidth/2,VideoHeight,VideoWidth/2,VideoHeight) );
	
	// Create list of associations between data reader and drawing place
	Omiscid::SimpleList<DrawingPlace*> DrawingObject;
	DrawingObject.Add(new DrawingPlace(MapDrawer, ImgTelemeter));
	DrawingObject.Add(new DrawingPlace(LocalizationDrawer, ImgTelemeter));
	DrawingObject.Add(new DrawingPlace(TelDrawer, ImgTelemeter));
	DrawingObject.Add(new DrawingPlace(CamDrawer, ImgCamera));
	DrawingObject.Add(new DrawingPlace(DepthDrawer, ImgDepth));
	DrawingObject.Add(new DrawingPlace(SkeletonDrawer, ImgDepth));
	DrawingObject.Add(new DrawingPlace(InfraredDrawer, ImgInfrared));
	DrawingObject.Add(new DrawingPlace(BodyIndexDrawer, ImgBodyIndex));
	DrawingObject.Add(new DrawingPlace(FaceDrawer, ImgCamera));

	// Call DoFolderVideo to process data with predefined info
	return DoFolderVideo( Folder, DrawingObject, VideoImg, "VideoEdit_", ShowVideo, TimestampHigh );
}

/** @brief Prepare all data to make a robot view from the stream, then call DoFolderVideo function to process it.
 *
 * @param Folder [in] Folder where we can read all data
 * @param ShowVideo [in] do we show video in interactive mode while processing?
 * @param TimestampHigh [in] Flag to provide information about where to put timestamp information within the video 
 * @return true if everything went fine.
*/
bool RobotViewFromFolderData( const std::string& Folder, bool ShowVideo, TimestampFlags TimestampHigh )
{
	// Draw robot at scale Depth
	int VideoWidth = DepthWidth;
	int VideoHeight = DepthHeight;
	
	// Create objects to read data from folder
	DrawLaserData TelDrawer( Folder );
	TelDrawer.CurrentDrawingMode = DrawLaserData::PointCloud;
	DrawLocalization LocalizationDrawer( Folder );
	DrawMap MapDrawer( Folder );

	// Create cv::Mat to draw all synchronized data
	CvSize VideoSize = cvSize(VideoWidth, VideoHeight);
	Mat VideoImg( VideoSize, CV_8UC3 );

	// Create list of associations between data reader and drawing place
	Omiscid::SimpleList<DrawingPlace*> DrawingObject;
	DrawingObject.Add( new DrawingPlace(MapDrawer, VideoImg) );
	DrawingObject.Add( new DrawingPlace(LocalizationDrawer, VideoImg) );
	DrawingObject.Add( new DrawingPlace(TelDrawer, VideoImg) );

	// Call DoFolderVideo to process data with predefined info
	return DoFolderVideo( Folder, DrawingObject, VideoImg, "RobotView_", ShowVideo, TimestampHigh );
}

/** @brief Prepare all data to make an RBG view, then call DoFolderVideo function to process it.
 *
 * @param Folder [in] Folder where we can read all data
 * @param ShowVideo [in] do we show video in interactive mode while processing?
 * @param TimestampHigh [in] Flag to provide information about where to put timestamp information within the video 
 * @return true if everything went fine.
*/
bool RGBFromFolderData( const std::string& Folder, bool ShowVideo, TimestampFlags TimestampHigh )
{
	// RGB wiew at full size
	int VideoWidth = CamWidth;
	int VideoHeight = CamWidth;
	
	// Create objects to read data from folder
	DrawCameraView CamDrawer( Folder );

	// Create Mat for drawing
	CvSize VideoSize = cvSize(VideoWidth, VideoHeight);
	Mat VideoImg( VideoSize, CV_8UC3 );
	
	// Create list of associations between data reader and drawing place (one unique object)
	Omiscid::SimpleList<DrawingPlace*> DrawingObject;
	DrawingObject.Add( new DrawingPlace(CamDrawer, VideoImg) );

	// Call DoFolderVideo to process data with predefined info
	return DoFolderVideo( Folder, DrawingObject, VideoImg, "RGB_", ShowVideo, TimestampHigh );
}

/** @brief Prepare all data to make a depth view, then call DoFolderVideo function to process it.
 *
 * @param Folder [in] Folder where we can read all data
 * @param ShowVideo [in] do we show video in interactive mode while processing?
 * @param TimestampHigh [in] Flag to provide information about where to put timestamp information within the video 
 * @return true if everything went fine.
*/
bool DepthFromFolderData( const std::string& Folder, bool ShowVideo, TimestampFlags TimestampHigh )
{
	int VideoWidth = DepthWidth;
	int VideoHeight = DepthHeight;
	
	// Create objects to read data from folder
	DrawDepthView DepthDrawer( Folder );

	// Create Mat for drawing
	CvSize VideoSize = cvSize(VideoWidth, VideoHeight);
	Mat VideoImg( VideoSize, CV_8UC3 );
	
	// Create list of associations between data reader and drawing place (one unique object)
	Omiscid::SimpleList<DrawingPlace*> DrawingObject;
	DrawingObject.Add( new DrawingPlace(DepthDrawer, VideoImg) );

	// Call DoFolderVideo to process data with predefined info
	return DoFolderVideo( Folder, DrawingObject, VideoImg, "Depth_", ShowVideo, TimestampHigh );
}

/** @brief Prepare all data to make an infrared view, then call DoFolderVideo function to process it.
 *
 * @param Folder [in] Folder where we can read all data
 * @param ShowVideo [in] do we show video in interactive mode while processing?
 * @param TimestampHigh [in] Flag to provide information about where to put timestamp information within the video 
 * @return true if everything went fine.
*/
bool InfraredFromFolderData( const std::string& Folder, bool ShowVideo, TimestampFlags TimestampHigh )
{
	int VideoWidth = InfraredWidth;
	int VideoHeight = InfraredHeight;
	
	// Create objects to read data from folder
	DrawInfraredView InfraredDrawer( Folder );

	// Create Mat for drawing
	CvSize VideoSize = cvSize(VideoWidth, VideoHeight);
	Mat VideoImg( VideoSize, CV_8UC3 );
	
	// Create list of associations between data reader and drawing place (one unique object)
	Omiscid::SimpleList<DrawingPlace*>& DrawingObject = *(new Omiscid::SimpleList<DrawingPlace*>);
	DrawingObject.Add( new DrawingPlace(InfraredDrawer, VideoImg) );

	// Call DoFolderVideo to process data with predefined info
	return DoFolderVideo( Folder, DrawingObject, VideoImg, "IR_", ShowVideo, TimestampHigh );
}

/** @brief Prepare all data to make an body index (body detection, previously player number) view at depth scale, then call DoFolderVideo function to process it.
 *
 * @param Folder [in] Folder where we can read all data
 * @param ShowVideo [in] do we show video in interactive mode while processing?
 * @param TimestampHigh [in] Flag to provide information about where to put timestamp information within the video 
 * @return true if everything went fine.
*/
bool BodyIndexFromFolderData( const std::string& Folder, bool ShowVideo, TimestampFlags TimestampHigh )
{
	int VideoWidth = DepthWidth;
	int VideoHeight = DepthHeight;
	
	// Create objects to read data from folder
	DrawBodyIndexView BodyIndexDrawer( Folder );

	// Create Mat for drawing
	CvSize VideoSize = cvSize(VideoWidth, VideoHeight);
	Mat VideoImg( VideoSize, CV_8UC3 );
	
	// Create list of associations between data reader and drawing place (one unique object)
	Omiscid::SimpleList<DrawingPlace*>& DrawingObject = *(new Omiscid::SimpleList<DrawingPlace*>);
	DrawingObject.Add( new DrawingPlace(BodyIndexDrawer, VideoImg) );

	// Call DoFolderVideo to process data with predefined info
	return DoFolderVideo( Folder, DrawingObject, VideoImg, "BodyI_", ShowVideo, TimestampHigh );
}

/** @brief Prepare all data to make an body view at depth scale, then call DoFolderVideo function to process it.
 *
 * @param Folder [in] Folder where we can read all data
 * @param DrawingObject [in] List containing all Reading/Drawing objects
 * @param VideoImg [in, out] cv::Mat to fill draw in
 * @param Prefix [in] Video file name prefix ("VideoEdit_" for a composite video)
 * @param ShowVideo [in] do we show video in interactive mode while processing?
 * @param TimestampHigh [in] Flag to provide information about where to put timestamp information within the video 
 * @return true if everything went fine.
 */
bool DoFolderVideo( const std::string& Folder, Omiscid::SimpleList<DrawingPlace*>& DrawingObject, cv::Mat& VideoImg, const std::string& Prefix, bool ShowVideo, TimestampFlags TimestampHigh /* = true */ )
{
	// A pointer to video encoder
	VideoIO * Encoder = nullptr;

	std::string sVideoName = Folder + "/ResVideo/" ;

	// Create video and done if needed
	if ( DataFile::FileOrFolderExists( sVideoName.c_str() ) == false )
	{
#ifdef OMISCID_ON_WINDOWS
		mkdir( sVideoName.c_str() );
#else
		mkdir( sVideoName.c_str(), 0777 );
#endif
	}

	// In order to open OpenCV Window
	const char * VideoName = strdup( Folder.c_str() );

	int CurrentFrame = 1;
	int GoToFrame = 0;

	if ( ShowVideo == true )
	{
		// Create window
		cv::namedWindow( VideoName, CV_WINDOW_AUTOSIZE );
	}

	bool paused = true;

	char LineBuffer[10*1024];	// 10KiB for writing information

	ConstantFpsTimestampFromFile RTS( Folder + "/Timestamps.lst", 30.0f );
	while( RTS.GetNextTimestamp() == true )
	{
		// To skip frame, for used in interactive mode, ShowVideo = true
		if ( CurrentFrame < GoToFrame )
		{
			CurrentFrame++;
			continue;
		}

		// If no encoder started, start one
		if ( Encoder == nullptr )
		{
			// Skip to a new video name, i.e. do not erase previous video generation
			int NumVideo = 1;
			do
			{
				sprintf( LineBuffer, "%s%s%.6d.mp4", sVideoName.c_str(), Prefix.c_str(), NumVideo++ );
				if ( LocalDebug == true )
				{
					printf( "Check if file '%s', exists\n", LineBuffer ) ;
				}
			}
			while ( DataFile::FileOrFolderExists( LineBuffer ) == true );
			
			// Ok, here we found a new video name avalaible
			if ( LocalDebug == true )
			{
				printf( "Openning %s\n", LineBuffer );
			}

			// Create it
			Encoder = new VideoIO();
			Encoder->Create( LineBuffer, VideoImg.cols, VideoImg.rows, 30.0, "-y -codec:v libx264 -profile:v high -preset slow -b:v 4000k -maxrate 5000k -bufsize 2000k -threads 0 -pix_fmt yuv420p" );
		}

		// If LocalDebug, print timestamp
		if ( LocalDebug == true )
		{
			fprintf( stderr, "%d.%.3d\r", (int)RTS.CurrentTimestamp.time, (int)RTS.CurrentTimestamp.millitm );
		}

Redraw:
		// Fill it white
		VideoImg = cv::Scalar(255,255,255);

		// Draw all object (depending on the previous call)
		for( DrawingObject.First(); DrawingObject.NotAtEnd(); DrawingObject.Next() )
		{
			DrawingPlace* pTmp = DrawingObject.GetCurrent();
			pTmp->DrawingObject.Draw( pTmp->DrawingZone, RTS.CurrentTimestamp );
		}

		// flip image, as Kinect data are fliped
		flip( VideoImg, VideoImg, 1 );

		// Shall we write the timestamp somewhere?
		if ( TimestampHigh != NO_TIMESTAMP )
		{
			if ( TimestampHigh == TIMESTAMP_HIGH )
			{
				rectangle( VideoImg, Point(0,0), Point(150,20), Scalar(255, 255, 255), -1 );

				// Write TS over the camera view
				sprintf( LineBuffer, "%d.%.3d", (int)RTS.CurrentTimestamp.time, (int)RTS.CurrentTimestamp.millitm );
				putText( VideoImg, LineBuffer, Point(5,15), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0, 0, 0), 1, 8 );
			}
			else
			{
				rectangle( VideoImg, Point(0,VideoImg.rows-20), Point(150,VideoImg.rows-1), Scalar(255, 255, 255), -1 );

				// Write TS over the camera view
				sprintf( LineBuffer, "%d.%.3d", (int)RTS.CurrentTimestamp.time, (int)RTS.CurrentTimestamp.millitm );
				putText( VideoImg, LineBuffer, Point(5,VideoImg.rows-5), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, Scalar(0, 0, 0), 1, 8 );
			}
		}

		// Give this image to encoder
		Encoder->WriteFrame( VideoImg );

		// increase frame number		
		CurrentFrame++;

		// Ok, are we *not* in interactive mode? (i.e. ShowVideo = false), go to next frame
		if ( ShowVideo == false )
		{
			continue;
		}

		// Show image
		imshow(VideoName, VideoImg);

		// track keybord inputs
		for(;;)
		{
			char KeyPressed = cv::waitKey(100);

			// If you press Escape, end of job (video will be stopped at this point)
			if ( KeyPressed == 27 )
			{
				// Goto to end of work, no need for putting a specific bollean in the main loop just for that.
				// The compiler does many jumps, so do I.
				goto EndOfWork;
			}

			// 'd' : toggle debug mode
			if ( KeyPressed == 'd' )
			{
				LocalDebug = !LocalDebug;
				fprintf( stderr, "Debug is %s\n", LocalDebug ? "on" : "off" );
				break;
			}

			// 's' : skip 10 frames
			// for debugging purpose as video will loose 10 frames
			else if ( KeyPressed == 's' )
			{
				GoToFrame = CurrentFrame+10;
				break;
			}
			// ' ' : pause on/off
			else if ( KeyPressed == ' ' )
			{
				paused = !paused;
				break;
			}
			// 't' : ask in console for a timestamp and search for it
			// *ONLY* for debugging purpose as output video will loose data 
			else if ( KeyPressed == 't' )
			{
				TimeB SearchTimeStamp;
				int iTmp;

				// try to parse line
				printf( "Enter timestamp:" );
				int MilliRead = 0;
				if ( scanf( "%d.%d", &iTmp, &MilliRead ) == 2 )
				{
					SearchTimeStamp.time = iTmp;
					SearchTimeStamp.millitm = MilliRead;
					RTS.SearchDataForTimestamp( SearchTimeStamp );
				}
				// Go to redraw, we do not want to seach for the next timestamp
				goto Redraw;
			}

			// If we are *not* anymore in pause mode or we typed 'x' (next frame), we can exit the loop
			if ( paused == false || KeyPressed == 'x' )
			{
				break;
			}

			// else, nothing special, wait for keyboard input again
		}
	}
EndOfWork:

	// Trace it in debug mode
	if ( LocalDebug == true )
	{ 
		fprintf( stderr, "\n" );
	}

	// Is there was an encoder, delete it (this call will close the video file if not done before)
	if ( Encoder != nullptr )
	{
		delete Encoder;
	}

	// If we were in interactive mode, destroy the windows and free the associated name
	if ( ShowVideo == true )
	{
		// Create window
		cv::destroyWindow( VideoName );

		// strdup => free
		free((void*)VideoName);
	}

	return true;
}

/** @enum ParamEnum
 *  @brief Define int value for autorized parameters associated to pParams array. 0 mean no parameter. Last value, NumberOfParams, contains actual number of autorized parameters.
 */
enum { NoParam = 0, ShowLiveVideo, DebugMode, VideoEdit, RGBOnly, DepthOnly, IROnly, BodyIndexOnly, RobotOnly, NumberOfParams } ParamEnum;

/** @brief Array containing string for autorized parameters. In shell call, all parameters must be prefixed by '-'.
 */
const char * pParam[] = { nullptr, "ShowLiveVideo", "DebugMode", "VideoEdit", "RGBOnly", "DepthOnly", "IROnly", "BodyIndexOnly", "RobotOnly", nullptr };

/** @brief Function to chack if the parameter is known or not. If yes, return the associated enum value. If not, return NoParam.
 */
inline int CheckParameter( const char * param )
{
	// Check if params seems good, not null and starting with '-'
	if ( param == nullptr || param[0] != '-' )
	{
		return NoParam;
	}

	// Check if it is an autorized parameter
	for( int pos = 1; pos < NumberOfParams; pos++ )
	{
		if ( strcasecmp(&param[1], pParam[pos]) == 0 )	// Skip first '-', thus &param[1]
		{
			return pos;
		}
	}

	// this parameter is not in the autorized list
	return NoParam;
}

/** @brief You must call the GenerateVideoFromRecords at least with two parameters: a folder name containing data and a rendering choice (-VideoEdit, -RGBOnly, etc...).
 * If you put several folders, for each one you must provide one or several rendering choices. It will process *iteratively* all rendering choices. Options can be set anywere
 * and several times without any inconvenient. They will be true for further parameters (until the end of the command line as they is no way to unset a parameter).
 * Usage: GenerateVideoFromRecords [-ShowLiveVideo] [-DebugMode] <folder1> <rendering_choice_1> [<rendering_choice_2> ...] [<folder2> <rendering_choice_n> [<rendering_choice_n+1> ...] 
 * @return 0 if successful, negative value for failure.
 * @example 
 */
int main( int argc, char *argv[] )
{
	Omiscid::SimpleString CurrentWorkingFolder;
	bool ShowVideo = false;	// Default value for the ShowWindow flag

	// Ask to use first the compressed version (if any) of the data. Useful for networked storage.
	DataFile::OpenCompressedVersionFirst = true;

	// Check Params
	int pos;
	for( pos = 1; pos < argc; pos++ )
	{
		if ( argv[pos][0] == '-' )
		{
			int VideoProcessingType = CheckParameter(argv[pos]);
			if ( VideoProcessingType != NoParam )
			{
				if ( VideoProcessingType == ShowLiveVideo )
				{
					ShowVideo = true;
					continue;
				}

				if ( VideoProcessingType == DebugMode )
				{
					LocalDebug = true;
					continue;
				}

				// Other parameter are process ones
				if ( CurrentWorkingFolder.IsEmpty() )
				{
					fprintf( stderr, "You must specify current working folder before any processing type\n");
					return -1;
				}

				// Here we can process video rendering type
				switch( VideoProcessingType )
				{
					// Video composition
					case VideoEdit:
						if ( VideoEditFromFolderData( CurrentWorkingFolder.GetStr(), ShowVideo, TIMESTAMP_HIGH ) != true ) { return -3; }
						break;

					// Only RGB
					case RGBOnly:
						if ( RGBFromFolderData( CurrentWorkingFolder.GetStr(), ShowVideo, TIMESTAMP_HIGH ) != true ) { return -3; }
						break;

					// Only depth
					case DepthOnly:
						if ( DepthFromFolderData( CurrentWorkingFolder.GetStr(), ShowVideo, TIMESTAMP_HIGH ) != true ) { return -3; }
						break;
					
					// Infrared Only
					case IROnly:
						if ( InfraredFromFolderData( CurrentWorkingFolder.GetStr(), ShowVideo, TIMESTAMP_HIGH ) != true ) { return -3; }
						break;

					// Body Only (aka skeletons)
					case BodyIndexOnly:
						if ( BodyIndexFromFolderData( CurrentWorkingFolder.GetStr(), ShowVideo, TIMESTAMP_HIGH ) != true ) { return -3; }
						break;

					// Robot data
					case RobotOnly:
						if ( RobotViewFromFolderData( CurrentWorkingFolder.GetStr(), ShowVideo, TIMESTAMP_HIGH ) != true ) { return -3; }
						break;
				}

				continue;
			}
			fprintf( stderr, "Unkown parameter '%s'\n", argv[pos] );
			return -2;
		}
		else
		{
			// Current shold be a folder
			CurrentWorkingFolder = argv[pos];

			// Got with '/' as its works under Windows also
			CurrentWorkingFolder.ReplaceAll("\\","/");
			if ( CurrentWorkingFolder.IsEmpty() == true )
			{
				fprintf( stderr, "Wrong empty parameter number %d\n", pos );
				return -3;
			}

			// Remove tailing '/' if any
			if ( CurrentWorkingFolder[CurrentWorkingFolder.GetLength()-1] == '/' )
			{
				CurrentWorkingFolder = CurrentWorkingFolder.SubString( 0, CurrentWorkingFolder.GetLength()-1 );
			}
		}
	}

	return 0;
}
