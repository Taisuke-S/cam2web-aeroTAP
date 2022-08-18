/*
    cam2web - streaming camera to web

    Copyright (C) 2017, cvsandbox, cvsandbox@gmail.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <map>
#include <mutex>
#include <thread>
#include <chrono>

#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <fstream>

#include "XV4LCamera.hpp"
#include "XManualResetEvent.hpp"
#include "aeroTAP_SDK.h"
#include "ZDColorPalette.h"

using namespace std;
using namespace std::chrono;
using namespace aerotap;

aeroTAP camera;

#if 1
typedef struct tagBITMAPINFOHEADER {
  uint32_t biSize;
  uint32_t	biWidth;
  uint32_t	biHeight;
  uint16_t	biPlanes;
  uint16_t	biBitCount;
  uint32_t	biCompression;
  uint32_t	biSizeImage;
  uint32_t	biXPelsPerMeter;
  uint32_t	biYPelsPerMeter;
  uint32_t	biClrUsed;
  uint32_t	biClrImportant;
} BITMAPINFOHEADER, *PBITMAPINFOHEADER;
#else
typedef struct tagBITMAPINFOHEADER {
  DWORD biSize;
  LONG  biWidth;
  LONG  biHeight;
  WORD  biPlanes;
  WORD  biBitCount;
  DWORD biCompression;
  DWORD biSizeImage;
  LONG  biXPelsPerMeter;
  LONG  biYPelsPerMeter;
  DWORD biClrUsed;
  DWORD biClrImportant;
} BITMAPINFOHEADER, *PBITMAPINFOHEADER;
#endif
#if 1
typedef struct tagBITMAPFILEHEADER {
  uint16_t	bfType;
  uint32_t	bfSize;
  uint16_t	bfReserved1;
  uint16_t	bfReserved2;
  uint32_t	bfOffBits;
} BITMAPFILEHEADER, *PBITMAPFILEHEADER;
#else
typedef struct tagBITMAPFILEHEADER {
  WORD  bfType;
  DWORD bfSize;
  WORD  bfReserved1;
  WORD  bfReserved2;
  DWORD bfOffBits;
} BITMAPFILEHEADER, *PBITMAPFILEHEADER;
#endif
void saveImageToFile(char*szName, int w, int h, unsigned char*data)
{
	BITMAPINFOHEADER *bmpInfoHeader = (BITMAPINFOHEADER*)malloc(sizeof(BITMAPINFOHEADER)+w*h*3);
	memset(bmpInfoHeader,0,sizeof(BITMAPINFOHEADER));
	bmpInfoHeader->biBitCount=24;
	bmpInfoHeader->biPlanes=1;
	bmpInfoHeader->biSize=sizeof(BITMAPINFOHEADER);
	bmpInfoHeader->biSizeImage=w*h*3;
	bmpInfoHeader->biWidth=w;
	bmpInfoHeader->biHeight=h;
	
	unsigned char *pT=(unsigned char*)(bmpInfoHeader+1);
	
	for(int i=0;i<w*h;++i)
	{
		memset(&pT[i*3],data[i],3);
	}
	
	BITMAPFILEHEADER bmpFileHeader = {0x4D42,0,0,0,0};
	bmpFileHeader.bfOffBits = sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER);
	bmpFileHeader.bfSize=bmpFileHeader.bfOffBits+bmpInfoHeader->biSizeImage;
//printf("%d  %d ",sizeof(BITMAPFILEHEADER),sizeof(BITMAPINFOHEADER));	
	
	ofstream of;
	of.open(szName, ios::out|std::ios::binary);
	of.write( (const char*)&bmpFileHeader, 14);
//	of.write( (const char*)&bmpFileHeader, sizeof(BITMAPFILEHEADER));
	of.write( (const char*)bmpInfoHeader, sizeof(BITMAPINFOHEADER));
	of.write( (const char*)pT, bmpInfoHeader->biSizeImage);
	
	of.close();	
}

namespace Private
{
    #define BUFFER_COUNT        (4)

	ZDColotPalette ColorPalette;

    // Private details of the implementation
    class XV4LCameraData
    {
    private:
        mutable recursive_mutex Sync;
        recursive_mutex         ConfigSync;
        thread                  ControlThread;
        XManualResetEvent       NeedToStop;
        IVideoSourceListener*   Listener;
        bool                    Running;

        int                     VideoFd;
        bool                    VideoStreamingActive;
        uint8_t*                MappedBuffers[BUFFER_COUNT];
        uint32_t                MappedBufferLength[BUFFER_COUNT];
        map<XVideoProperty, int32_t> PropertiesToSet;
#define _HIST_MAX 3000
        uint32_t                nZDTable[2048];
        uint32_t 				nZDTableLen;
		uint32_t				nDepthHist[_HIST_MAX];
		bool bUSB20;
		int nPType;
    public:
        uint32_t                VideoDevice;
        uint32_t                FramesReceived;
        uint32_t                FrameWidth;
        uint32_t                FrameHeight;
        uint32_t                FrameRate;
		bool	JpegEncoding;
		int    nImageType;  // Color, Gray, Depth, DepthRaw
		char szDeviceName[256];
#define 		_IMAGE_COLOR 0 
#define 		_IMAGE_GRAY 1 
#define 		_IMAGE_DEPTH 2
#define 		_IMAGE_DEPTHRAW 3
	public:
        XV4LCameraData( ) :
            Sync( ), ConfigSync( ), ControlThread( ), NeedToStop( ), Listener( nullptr ), Running( false ),
            VideoFd( -1 ), VideoStreamingActive( false ), MappedBuffers( ), MappedBufferLength( ), PropertiesToSet( ),
            VideoDevice( 0 ),
            FramesReceived( 0 ), FrameWidth( 640 ), FrameHeight( 480 ), FrameRate( 30 ), JpegEncoding( true )
        {
        }

        bool Start( );
        void SignalToStop( );
        void WaitForStop( );
        bool IsRunning( );
        IVideoSourceListener* SetListener( IVideoSourceListener* listener );

        void NotifyNewImage( const std::shared_ptr<const XImage>& image );
        void NotifyError( const string& errorMessage, bool fatal = false );

        static void ControlThreadHanlder( XV4LCameraData* me );

        void SetVideoDevice( uint32_t videoDevice );
        void SetVideoSize( uint32_t width, uint32_t height );
        void SetFrameRate( uint32_t frameRate );
        void EnableJpegEncoding( bool enable );
        void SetImageType( uint32_t ntype );


        XError SetVideoProperty( XVideoProperty property, int32_t value );
        XError GetVideoProperty( XVideoProperty property, int32_t* value ) const;
        XError GetVideoPropertyRange( XVideoProperty property, int32_t* min, int32_t* max, int32_t* step, int32_t* def ) const;

    private:
        bool Init( );
        void VideoCaptureLoop( );
        void Cleanup( );

		void DecodeDepthToRgb( const uint8_t* depthPtr, const uint8_t* grayPtr, uint8_t* rgbPtr, int32_t width, int32_t height,int32_t rgbStride  );
		
		shared_ptr<XImage> rgbImage;

    };
}


const shared_ptr<XV4LCamera> XV4LCamera::Create( )
{
    return shared_ptr<XV4LCamera>( new XV4LCamera );
}

XV4LCamera::XV4LCamera( ) :
    mData( new Private::XV4LCameraData( ) )
{
}

XV4LCamera::~XV4LCamera( )
{
    delete mData;
}
char *XV4LCamera::getDeviceName()
{
    return mData->szDeviceName;
}

// Start the video source
bool XV4LCamera::Start( )
{
    return mData->Start( );
}

// Signal video source to stop
void XV4LCamera::SignalToStop( )
{
	camera.stop();
    mData->SignalToStop( );
}

// Wait till video source stops
void XV4LCamera::WaitForStop( )
{
    mData->WaitForStop( );
}

// Check if video source is still running
bool XV4LCamera::IsRunning( )
{
    return mData->IsRunning( );
}

// Get number of frames received since the start of the video source
uint32_t XV4LCamera::FramesReceived( )
{
    return mData->FramesReceived;
}

// Set video source listener
IVideoSourceListener* XV4LCamera::SetListener( IVideoSourceListener* listener )
{
    return mData->SetListener( listener );
}

// Set/get video device
uint32_t XV4LCamera::VideoDevice( ) const
{
    return mData->VideoDevice;
}
void XV4LCamera::SetVideoDevice( uint32_t videoDevice )
{
    mData->SetVideoDevice( videoDevice );
}

// Get/Set video size
uint32_t XV4LCamera::Width( ) const
{
    return mData->FrameWidth;
}
uint32_t XV4LCamera::Height( ) const
{
    return mData->FrameHeight;
}
void XV4LCamera::SetVideoSize( uint32_t width, uint32_t height )
{
    mData->SetVideoSize( width, height );
}
void XV4LCamera::SetImageType( uint32_t ntype )
{
    mData->SetImageType( ntype );
}

// Get/Set frame rate
uint32_t XV4LCamera::FrameRate( ) const
{
    return mData->FrameRate;
}
void XV4LCamera::SetFrameRate( uint32_t frameRate )
{
    mData->SetFrameRate( frameRate );
}

// Enable/Disable JPEG encoding
bool XV4LCamera::IsJpegEncodingEnabled( ) const
{
    return mData->JpegEncoding;
}
void XV4LCamera::EnableJpegEncoding( bool enable )
{
    mData->EnableJpegEncoding( enable );
}

// Set the specified video property
XError XV4LCamera::SetVideoProperty( XVideoProperty property, int32_t value )
{
    return mData->SetVideoProperty( property, value );
}

// Get current value if the specified video property
XError XV4LCamera::GetVideoProperty( XVideoProperty property, int32_t* value ) const
{
    return mData->GetVideoProperty( property, value );
}

// Get range of values supported by the specified video property
XError XV4LCamera::GetVideoPropertyRange( XVideoProperty property, int32_t* min, int32_t* max, int32_t* step, int32_t* def ) const
{
    return mData->GetVideoPropertyRange( property, min, max, step, def );
}

namespace Private
{

// Start video source so it initializes and begins providing video frames
bool XV4LCameraData::Start( )
{
    lock_guard<recursive_mutex> lock( Sync );
//	printf("Call Start....\n");
    if ( !IsRunning( ) )
    {
        NeedToStop.Reset( );
        Running = true;
        FramesReceived = 0;

        ControlThread = thread( ControlThreadHanlder, this );
    }

    return true;
}

// Signal video to stop, so it could finalize and clean-up
void XV4LCameraData::SignalToStop( )
{
    lock_guard<recursive_mutex> lock( Sync );

//	printf("Call Camera Stop....\n");
	camera.stop();

    if ( IsRunning( ) )
    {
        NeedToStop.Signal( );
    }
}

// Wait till video source (its thread) stops
void XV4LCameraData::WaitForStop( )
{
    if ( ( IsRunning( ) ) || ( ControlThread.joinable( ) ) )
    {
        ControlThread.join( );
    }
}

// Check if video source is still running
bool XV4LCameraData::IsRunning( )
{
    lock_guard<recursive_mutex> lock( Sync );
    
    if ( ( !Running ) && ( ControlThread.joinable( ) ) )
    {
        ControlThread.join( );
    }
    
    return Running;
}

// Set video source listener
IVideoSourceListener* XV4LCameraData::SetListener( IVideoSourceListener* listener )
{
    lock_guard<recursive_mutex> lock( Sync );
    IVideoSourceListener* oldListener = listener;

    Listener = listener;

    return oldListener;
}

// Notify listener with a new image
void XV4LCameraData::NotifyNewImage( const std::shared_ptr<const XImage>& image )
{
    IVideoSourceListener* myListener;
    
    {
        lock_guard<recursive_mutex> lock( Sync );
        myListener = Listener;
    }
    
    if ( myListener != nullptr )
    {
        myListener->OnNewImage( image );
    }
}

// Notify listener about error
void XV4LCameraData::NotifyError( const string& errorMessage, bool fatal )
{
    IVideoSourceListener* myListener;
    
    {
        lock_guard<recursive_mutex> lock( Sync );
        myListener = Listener;
    }
    
    if ( myListener != nullptr )
    {
        myListener->OnError( errorMessage, fatal );
    }
}

// Initialize camera and start capturing
bool XV4LCameraData::Init( )
{
    bool ret = true;
//	printf("Call Init....\n");

//	nImageType = _IMAGE_DEPTH;
	bUSB20 = true; // always USB20 for raspi nanopi
//	camera.setVerbose(true);
//	sprintf("Camera output imaage type =%d\n", nImageType);

	if (!camera.checkDevice())
	{
		ret =  false;
	}
	else
	{
		strcpy(szDeviceName ,camera.getProductName());
		nPType = camera.getPType();
		bUSB20 = camera.getUSB20();
		if ( bUSB20  )
		{
			printf("Camera using MJPEG\n");
			camera.useMJPG(true);
		}
		else
		{
			printf("Camera using YUV\n");
		}

        rgbImage = XImage::Allocate( FrameWidth, FrameHeight, XPixelFormat::RGB24 ,true);
        if ( !rgbImage )
        {
            NotifyError( "Failed allocating an image", true );
            return false;
        }

		if (ret = camera.open(NULL, NULL, FrameWidth, FrameHeight))
		{
			VideoFd = camera.getFD();
			// Start aeroTAP
			camera.start();
//			camera.setFilter(1);
			nZDTableLen = camera.getZDTable((uint16_t *)nZDTable);

		}
	}
	if ( !ret )
		printf("camera Init error\n");
	

	return ret;
}

// Stop camera capture and clean-up
void XV4LCameraData::Cleanup( )
{
    lock_guard<recursive_mutex> lock( ConfigSync );
	camera.stop();
}
void XV4LCameraData::DecodeDepthToRgb( const uint8_t* depthPtr, const uint8_t* grayPtr, uint8_t* rgbPtr, int32_t width, int32_t height,int32_t rgbStride  )
{
	uint16_t nDepth;
	uint8_t* rgbRow = rgbPtr;
	uint8_t *pTrg= rgbPtr;

//	printf("%d %d x %d \n", rgbStride, width,height);
	if (nImageType == _IMAGE_DEPTHRAW)
	{
		int _width = width;
		uint16_t *depthW = (uint16_t*)depthPtr;
		for (int y = 0; y < height; ++y)
		{
			rgbRow = pTrg+y*(int)width * 3;
			int p = y*width;
			for (int x = 0; x < _width; ++x)
			{
				nDepth = depthW[p];
				rgbRow[0] = grayPtr[p];
				rgbRow[1] = (nDepth & 0xFF);
				rgbRow[2] = (nDepth >> 8);
				rgbRow += 3;
				++p;
			}
		}
		return;
	}
	if (nImageType == _IMAGE_DEPTH)
	{
		RGBQUAD	*pColorPalette = ColorPalette.GetColorPalette();
		uint16_t *depthW = (uint16_t*)depthPtr;
		for (int y = 0; y < height; ++y)
		{
			int p = y*width;
			rgbRow = pTrg + y*(int)width * 3;
			for (int x = 0; x < width; ++x)
			{
				nDepth = depthW[p];
				if (nDepth == 0 )
				{
					rgbRow[0] = 255;
					rgbRow[1] = 255;
					rgbRow[2] = 255;
				}
				else
				if (nDepth > 10000)
				{
					rgbRow[0] = 0;
					rgbRow[1] = 0;
					rgbRow[2] = 0;
				}
				else
				{
					rgbRow[0] = pColorPalette[nDepth].rgbRed;
					rgbRow[1] = pColorPalette[nDepth].rgbGreen;
					rgbRow[2] = pColorPalette[nDepth].rgbBlue;
				}
				rgbRow += 3;
				++p;
			}

		}
	}
}

// Helper function to decode YUYV data into RGB
static void DecodeYuyvToRgb( const uint8_t* yuyvPtr, uint8_t* rgbPtr, int32_t width, int32_t height, int32_t rgbStride )
{
    /* 
        The code below does YUYV to RGB conversion using the next coefficients.
        However those are multiplied by 256 to get integer calculations.
     
        r = y + (1.4065 * (cr - 128));
        g = y - (0.3455 * (cb - 128)) - (0.7169 * (cr - 128));
        b = y + (1.7790 * (cb - 128));
    */

    int r, g, b;
    int y, u, v;
    int z = 0;

    for ( int32_t iy = 0; iy < height; iy++ )
    {
        uint8_t* rgbRow = rgbPtr + iy * rgbStride;

        for ( int32_t ix = 0; ix < width; ix++ )
        {
            y = ( ( z == 0 ) ? yuyvPtr[0] : yuyvPtr[2] ) << 8;
            u = yuyvPtr[1] - 128;
            v = yuyvPtr[3] - 128;

            r = ( y + ( 360 * v ) ) >> 8;
            g = ( y - ( 88  * u ) - ( 184 * v ) ) >> 8;
            b = ( y + ( 455 * u ) ) >> 8;

            rgbRow[RedIndex]   = (uint8_t) ( r > 255 ) ? 255 : ( ( r < 0 ) ? 0 : r );
            rgbRow[GreenIndex] = (uint8_t) ( g > 255 ) ? 255 : ( ( g < 0 ) ? 0 : g );
            rgbRow[BlueIndex]  = (uint8_t) ( b > 255 ) ? 255 : ( ( b < 0 ) ? 0 : b );

            if ( z++ )
            {
                z = 0;
                yuyvPtr += 4;
            }

            rgbRow += 3;
        }
    }
}


// Do video capture in an end-less loop until signalled to stop
void XV4LCameraData::VideoCaptureLoop( )
{
    v4l2_buffer videoBuffer;
    uint32_t    sleepTime = 0;
    uint32_t    frameTime = 1000 / FrameRate;
    uint32_t    handlingTime ;
    int         ecode;

    // If JPEG encoding is used, client is notified with an image wrapping a mapped buffer.
    // If not used howver, we decode YUYV data into RGB.
    

    // acquire images untill we've been told to stop
    while ( !NeedToStop.Wait( sleepTime ) )
    {
        steady_clock::time_point startTime = steady_clock::now( );

		if (camera.isNewFrame())
        {
            shared_ptr<XImage> image;
            FramesReceived++;
			camera.updateFrame();

//			printf("new frame %d type = %d\n", FramesReceived, nImageType);
			if (nImageType < _IMAGE_DEPTH)
			{
				if (nImageType == _IMAGE_GRAY)
					image = XImage::Create((uint8_t*)camera.getGrayData(), FrameWidth, FrameHeight, rgbImage->Stride()/3,XPixelFormat::Grayscale8);
				else
					image = XImage::Create((uint8_t*)camera.getColorData(), FrameWidth, FrameHeight, rgbImage->Stride(), XPixelFormat::RGB24);
				if ( image )
				{
//saveImageToFile("rawdata.bmp", FrameWidth, FrameHeight, image->Data());
					NotifyNewImage( image );
				}
				else
				{
					NotifyError( "Failed allocating an image" );
				}
			}
			else
			{ 
				DecodeDepthToRgb((uint8_t*)camera.getDepthData(), (uint8_t*)camera.getGrayData(), rgbImage->Data(), FrameWidth, FrameHeight, rgbImage->Stride());
				if ( rgbImage )
				{
//saveImageToFile("rawdata.bmp", FrameWidth, FrameHeight, rgbImage->Data());
					NotifyNewImage( rgbImage );
				}
				else
				{
					NotifyError( "Failed allocating an image" );
				}
			}
        }

        handlingTime = static_cast<uint32_t>( duration_cast<milliseconds>( steady_clock::now( ) - startTime ).count( ) );
        sleepTime    = ( handlingTime > frameTime ) ? 0 : ( frameTime - handlingTime );
    }
}

// Background control thread - performs camera init/clean-up and runs video loop
void XV4LCameraData::ControlThreadHanlder( XV4LCameraData* me )
{    
    if ( me->Init( ) )
    {
        me->VideoCaptureLoop( );
    }
    
    me->Cleanup( );
    
    {
        lock_guard<recursive_mutex> lock( me->Sync );
        me->Running = false;
    }
}

// Set vide device number to use
void XV4LCameraData::SetVideoDevice( uint32_t videoDevice )
{
    lock_guard<recursive_mutex> lock( Sync );

    if ( !IsRunning( ) )
    {
        VideoDevice = videoDevice;
    }
}

// Set size of video frames to request
void XV4LCameraData::SetVideoSize( uint32_t width, uint32_t height )
{
    lock_guard<recursive_mutex> lock( Sync );

    if ( !IsRunning( ) )
    {
        FrameWidth  = width;
        FrameHeight = height;
    }
}
void XV4LCameraData::SetImageType( uint32_t ntype )
{
    lock_guard<recursive_mutex> lock( Sync );

    if ( !IsRunning( ) )
    {
		nImageType = ntype;
    }
}

// Set rate to query images at
void XV4LCameraData::SetFrameRate( uint32_t frameRate )
{
    lock_guard<recursive_mutex> lock( Sync );

    if ( !IsRunning( ) )
    {
        FrameRate = frameRate;
    }
}

// Enable/disable JPEG encoding
void XV4LCameraData::EnableJpegEncoding( bool enable )
{
    lock_guard<recursive_mutex> lock( Sync );

    if ( !IsRunning( ) )
    {
        JpegEncoding = enable;
    }
}

static const uint32_t nativeVideoProperties[] =
{
    V4L2_CID_BRIGHTNESS,
    V4L2_CID_CONTRAST,
    V4L2_CID_SATURATION,
    V4L2_CID_HUE,
    V4L2_CID_SHARPNESS,
    V4L2_CID_GAIN,
    V4L2_CID_BACKLIGHT_COMPENSATION,
    V4L2_CID_RED_BALANCE,
    V4L2_CID_BLUE_BALANCE,
    V4L2_CID_AUTO_WHITE_BALANCE,
    V4L2_CID_HFLIP,
    V4L2_CID_VFLIP,
    V4L2_CID_EXPOSURE_AUTO,
    
};

// Set the specified video property
XError XV4LCameraData::SetVideoProperty( XVideoProperty property, int32_t value )
{
    lock_guard<recursive_mutex> lock( Sync );
    XError                      ret = XError::Success;

printf("called SetVideoPropertyRange\n");

    if ( ( property < XVideoProperty::Brightness ) || ( property > XVideoProperty::AutoExposure ) )
    {
		printf("Unknown property");
        ret = XError::UnknownProperty;
    }
    else if ( ( !Running ) || ( VideoFd == -1 ) )
    {
        // save property value and try setting it when device gets runnings
        PropertiesToSet[property] = value;
    }
    else
    {
        v4l2_control control;

        control.id    = nativeVideoProperties[static_cast<int>( property )];
        control.value = value;

        if ( ioctl( VideoFd, VIDIOC_S_CTRL, &control ) < 0 )
        {
            ret = XError::Failed;
        }
    }

    return ret;
}

// Get current value if the specified video property
XError XV4LCameraData::GetVideoProperty( XVideoProperty property, int32_t* value ) const
{
    lock_guard<recursive_mutex> lock( Sync );
    XError                      ret = XError::Success;

    if ( value == nullptr )
    {
        ret = XError::NullPointer;
    }
    else if ( ( property < XVideoProperty::Brightness ) || ( property > XVideoProperty::AutoExposure ) )
    {
		if ( property == XVideoProperty::FocalLengthW )
		{
//printf("called GetVideoProperty FocalLengthW %.3f\n",camera.getFocalLength(0));
            *value = (int32_t)(camera.getFocalLength(0)*1000);
			return ret;
		}
		if ( property == XVideoProperty::FocalLengthH )
		{
			float fl =camera.getFocalLength(1);
//printf("called GetVideoProperty FocalLengthH %d %.3f\n",(int32_t)(fl*1000),fl);
            *value = (int32_t)(fl*1000);
			return ret;
		}
//printf("called GetVideoProperty UnknownProperty %d>%d\n",static_cast<int>(property),XVideoProperty::AutoExposure );
        ret = XError::UnknownProperty;
    }
    else if ( ( !Running ) || ( VideoFd == -1 ) )
    {
        ret = XError::DeivceNotReady;
    }
    else
    {
        v4l2_control control;

        control.id = nativeVideoProperties[static_cast<int>( property )];

        if ( ioctl( VideoFd, VIDIOC_G_CTRL, &control ) < 0 )
        {
            ret = XError::Failed;
        }
        else
        {
            *value = control.value;
        }
    }

    return ret;
}

// Get range of values supported by the specified video property
XError XV4LCameraData::GetVideoPropertyRange( XVideoProperty property, int32_t* min, int32_t* max, int32_t* step, int32_t* def ) const
{
    lock_guard<recursive_mutex> lock( Sync );
    XError                      ret = XError::Success;

    if ( ( min == nullptr ) || ( max == nullptr ) || ( step == nullptr ) || ( def == nullptr ) )
    {
        ret = XError::NullPointer;
    }
    else if ( ( property < XVideoProperty::Brightness ) || ( property > XVideoProperty::AutoExposure ) )
    {
        ret = XError::UnknownProperty;
//printf("called GetVideoPropertyRange UnknownProperty %d\n",static_cast<int>( property ));
    }
    else if ( ( !Running ) || ( VideoFd == -1 ) )
    {
        ret = XError::DeivceNotReady;
    }
    else
    {
        v4l2_queryctrl queryControl;

        queryControl.id = nativeVideoProperties[static_cast<int>( property )];

        if ( ioctl( VideoFd, VIDIOC_QUERYCTRL, &queryControl ) < 0 )
        {
            ret = XError::Failed;
        }
        else if ( ( queryControl.flags & V4L2_CTRL_FLAG_DISABLED ) != 0 )
        {
            ret = XError::ConfigurationNotSupported;
        }
        else if ( ( queryControl.type & ( V4L2_CTRL_TYPE_BOOLEAN | V4L2_CTRL_TYPE_INTEGER ) ) != 0 )
        {
/*			
            printf( "property: %d, min: %d, max: %d, step: %d, def: %d, type: %s \n ", static_cast<int>( property ),
                    queryControl.minimum, queryControl.maximum, queryControl.step, queryControl.default_value,
                    ( queryControl.type & V4L2_CTRL_TYPE_BOOLEAN ) ? "bool" : "int" );
*/
            *min  = queryControl.minimum;
            *max  = queryControl.maximum;
            *step = queryControl.step;
            *def  = queryControl.default_value;
        }
        else
        {
			printf("xoctl ConfigurationNotSupported!\n");
            ret = XError::ConfigurationNotSupported;
        }
    }

    return ret;
}

} // namespace Private

