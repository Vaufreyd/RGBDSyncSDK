#define _WINSOCKAPI_
#include <Windows.h>

#include <Kinect/KinectSensor.h>
#include <Kinect/KinectImageConverter.h>
#include <System/Socket.h>
#include <System/ElapsedTime.h>
#include <System/Mutex.h>
#include <System/Mutex.h>

using namespace MobileRGBD;
using namespace MobileRGBD::Kinect2;

#define EXIT_BAD_PARAMETER (-1)

int main( int argc, char*argv[] )
{
	int RecordingTime = 10;	// 10 seconds
	Omiscid::SimpleString RecordingFolderName = "TestRecord";

	// parse parameters if any
	for ( int CurParam = 1; CurParam < argc; CurParam++ )
	{
		if ( strcasecmp( "-f", argv[CurParam] ) == 0 )
		{
			// expecting a folder
			if ( CurParam == (argc-1) || argv[++CurParam][0] == '-' )
			{
				fprintf( stderr, "Missing argument after -f (folder name) option\n" );
				return EXIT_BAD_PARAMETER;
			}

			// Ok, set the folder name
			RecordingFolderName = argv[CurParam];
			continue;
		}

		if ( strcasecmp( "-t", argv[CurParam] ) == 0 )
		{
			// expecting a folder
			if ( CurParam == (argc-1) )
			{
				fprintf( stderr, "Missing argument after -t (recording time) option\n" );
				return EXIT_BAD_PARAMETER;
			}

			// got to next parameter
			CurParam++;
			if ( argv[CurParam][0] == '-' )
			{
				fprintf( stderr, "Time after -t (cordording time) must not be negative or an option\n" );
				return EXIT_BAD_PARAMETER;
			}

			// Ok, set the recording time, lazy call to atoi()...
			RecordingTime = atoi( argv[CurParam] );

			if ( RecordingTime == 0 )
			{
				fprintf( stderr, "Recording time must not be 0.\n" );
				return EXIT_BAD_PARAMETER;
			}

			continue;
		}

		fprintf( stderr, "Unexpected parameter '%s'\n", argv[CurParam] );
		return EXIT_BAD_PARAMETER;
	}

	// Create Kinect Sensor
	KinectSensor MySensor;

	// Init Kinect sensor asking for Body and Depth information
	if ( MySensor.Init( FrameSourceTypes_All ) == false ) // FrameSourceTypes_Color|FrameSourceTypes_Depth|FrameSourceTypes_Body|FrameSourceTypes_Face|FrameSourceTypes_Infrared ) == false )
	{
		fprintf( stderr, "Unable to start Kinect2 with these frame sources\n" );
		return EXIT_FAILURE;
	}

	// Check if we can start recording (Folder name is ok, ACL are ok, ...)
	if ( MySensor.StartRecording( RecordingFolderName.GetStr() ) == false )
	{
		fprintf( stderr, "Unable to start recording in '%s'\n", RecordingFolderName.GetStr() );
		return EXIT_FAILURE;
	}
	fprintf( stderr, "Start recording in '%s' for %d seconds\n", RecordingFolderName.GetStr(), RecordingTime );

	// Wait for recording Time seconds
	Omiscid::Thread::Sleep( RecordingTime*1000 );

	fprintf( stderr, "Stop record\n" );

	// Stop recording
	MySensor.StopRecording();

	fprintf( stderr, "Exit" );

	// Go away, KinectSensor will stop in its destructor
	return 0;
}

