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

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pwd.h>
#include <linux/limits.h>
#include <map>

#include "XV4LCamera.hpp"
#include "XV4LCameraConfig.hpp"
#include "XWebServer.hpp"
#include "XVideoSourceToWeb.hpp"
#include "XObjectConfigurationSerializer.hpp"
#include "XObjectConfigurationRequestHandler.hpp"
#include "XManualResetEvent.hpp"

// Release build embeds web resources into executable
#ifdef NDEBUG
    #include "index.html.h"
    #include "styles.css.h"
    #include "cam2web.png.h"
    #include "cam2web_white.png.h"
    #include "camera.js.h"
    #include "cameraproperties.html.h"
    #include "cameraproperties.js.h"
    #include "jquery.js.h"
    #include "jquery.mobile.js.h"
    #include "jquery.mobile.css.h"
    #include "jquery.mobile.min.map.h"
#endif

using namespace std;

// Information provided on version request
#define STR_INFO_PRODUCT        "cam2web for aeroTAP"
#define STR_INFO_VERSION        "1.1.4"
#define STR_INFO_PLATFORM       "Linux"

// Name of the device and default title of the camera
const char* DEVICE_NAME = "Video for aeroTAP 3D Camera";

XManualResetEvent ExitEvent;

// Different application settings
struct
{
    uint32_t DeviceNumber;
    uint32_t FrameWidth;
    uint32_t FrameHeight;
    uint32_t FrameRate;
    uint32_t JpegQuality;
    uint32_t WebPort;
    uint32_t ImageType;
    uint32_t AngleX;
    string   HtRealm;
    string   HtDigestFileName;
    string   CameraConfigFileName;
    string   CustomWebContent;
    string   CameraTitle;
    string   CameraDevice;
    UserGroup ViewersGroup;
    UserGroup ConfigGroup;
}
Settings;

// Raise exit event when signal is received
void sigIntHandler( int s )
{
    ExitEvent.Signal( );
}

// Listener for camera errors
class CameraErrorListener : public IVideoSourceListener
{
public:
    // New video frame notification - ignore it
    virtual void OnNewImage( const std::shared_ptr<const XImage>& image ) { };

    // Video source error notification
    virtual void OnError( const std::string& errorMessage, bool fatal )
    {
        printf( "[%s] : %s \n", ( ( fatal ) ? "Fatal" : "Error" ), errorMessage.c_str( ) );
        if ( fatal )
        {
            // time to exit if something has bad happened
            ExitEvent.Signal( );
        }
    }
};

// Set default values for settings
void SetDefaultSettings( )
{
    Settings.DeviceNumber = 0;
    Settings.FrameWidth   = 640;
    Settings.FrameHeight  = 480;
    Settings.FrameRate    = 30;
    Settings.JpegQuality = 85;
    Settings.WebPort      = 8000;
    Settings.ImageType      = 2; //IMAGE_DEPTH
    Settings.AngleX      = 0; //camera rotation Angle X axis ( DEGREE )

    Settings.HtRealm = "cam2web";
    Settings.HtDigestFileName.clear( );

    Settings.ViewersGroup = UserGroup::Anyone;
    Settings.ConfigGroup  = UserGroup::Anyone;

    struct passwd* pwd = getpwuid( getuid( ) );
    if ( pwd )
    {
        Settings.CameraConfigFileName  = pwd->pw_dir;
        Settings.CameraConfigFileName += "/.cam_config";
    }

#ifdef NDEBUG
    Settings.CustomWebContent.clear( );
#else
    // default location of web content for debug builds
    Settings.CustomWebContent = "./web";
#endif

    Settings.CameraTitle = DEVICE_NAME;
    Settings.CameraDevice = DEVICE_NAME;
}

// Parse command line and override default settings
bool ParseCommandLine( int argc, char* argv[] )
{
    static const uint32_t SupportedWidth[]  = { 320, 640, 640, 640, 1280 };
    static const uint32_t SupportedHeight[] = { 240, 360, 400, 480, 720 };
    static const map<string, UserGroup> SupportedUserGroups =
    {
        { "any",    UserGroup::Anyone   },
        { "user",   UserGroup::User     },
        { "admin",  UserGroup::Admin    }
    };

    bool overrideViewersGroup = false;
    bool overrideConfigGroup  = false;

    UserGroup viewersGroup;
    UserGroup configGroup;

    bool ret = true;
    int  i;

    for ( i = 1; i < argc; i++ )
    {
        char* ptrDelimiter = strchr( argv[i], ':' );

        if ( ( ptrDelimiter == nullptr ) || ( argv[i][0] != '-' ) )
        {
            break;
        }

        string key   = string( argv[i] + 1, ptrDelimiter - argv[i] - 1 );
        string value = string( ptrDelimiter + 1 );

        if ( ( key.empty( ) ) || ( value.empty( ) ) )
            break;

        if ( key == "dev" )
        {
            int scanned = sscanf( value.c_str( ), "%u", &(Settings.DeviceNumber) );

            if ( scanned != 1 )
                break;
        }
        else if ( key == "size" )
        {
            int size_index;

            int scanned = sscanf( value.c_str( ), "%u", &size_index );

            if ( scanned != 1 )
                break;

            if ( ( size_index < 0 ) || ( size_index > 4 ) )
                break;

            Settings.FrameWidth  = SupportedWidth[size_index];
            Settings.FrameHeight = SupportedHeight[size_index];
        }
        else if ( key == "fps" )
        {
            int scanned = sscanf( value.c_str( ), "%u", &(Settings.FrameRate) );

            if ( scanned != 1 )
                break;

            if ( ( Settings.FrameRate < 1 ) || ( Settings.FrameRate > 30 ) )
                Settings.FrameRate = 30;
        }
        else if ( key == "jpeg" )
        {
            int scanned = sscanf( value.c_str( ), "%u", &(Settings.JpegQuality) );

            if ( scanned != 1 )
                break;

            if ( Settings.JpegQuality < 1 )
                Settings.JpegQuality = 1;
            if ( Settings.JpegQuality > 100 )
                Settings.JpegQuality = 100;
        }
        else if ( key == "port" )
        {
            int scanned = sscanf( value.c_str( ), "%u", &(Settings.WebPort) );

            if ( scanned != 1 )
                break;

            if ( Settings.WebPort > 65535 )
                Settings.WebPort = 65535;
        }
        else if ( key == "realm" )
        {
            Settings.HtRealm = value;
        }
        else if ( key == "htpass" )
        {
            Settings.HtDigestFileName = value;
            // if user specified password file, then he wants some security most probably
            // allow viewing only to users and changing settings to admin
            Settings.ViewersGroup = UserGroup::User;
            Settings.ConfigGroup  = UserGroup::Admin;
        }
        else if ( key == "viewer" )
        {
            map<string, UserGroup>::const_iterator itGroup = SupportedUserGroups.find( value );

            if ( itGroup == SupportedUserGroups.end( ) )
            {
                break;
            }
            else
            {
                viewersGroup = itGroup->second;
                overrideViewersGroup = true;
            }
        }
        else if ( key == "config" )
        {
            map<string, UserGroup>::const_iterator itGroup = SupportedUserGroups.find( value );

            if ( itGroup == SupportedUserGroups.end( ) )
            {
                break;
            }
            else
            {
                configGroup = itGroup->second;
                overrideConfigGroup = true;
            }
        }
        else if ( key == "fcfg" )
        {
            Settings.CameraConfigFileName = value;
        }
        else if ( key == "web" )
        {
            Settings.CustomWebContent = value;
        }
        else if ( key == "title" )
        {
            Settings.CameraTitle = value;
        }
        else if ( key == "device" )
        {
            Settings.CameraDevice = value;
        }
        else if ( key == "image" )
        {
            int scanned = sscanf( value.c_str( ), "%u", &(Settings.ImageType) );
            if ( scanned != 1 )
                break;

            if ( Settings.ImageType > 3 )
                Settings.ImageType = 2;
        }
        else if ( key == "angleX" )
        {
            int scanned = sscanf( value.c_str( ), "%u", &(Settings.AngleX) );
            if ( scanned != 1 )
                break;
        }
        else
        {
            break;
        }
    }

    if ( ( ( ( overrideViewersGroup ) && ( viewersGroup != UserGroup::Anyone ) ) ||
           ( ( overrideConfigGroup ) && ( configGroup != UserGroup::Anyone ) ) ) &&
         ( Settings.HtDigestFileName.empty( ) ) )
    {
        printf( "Warning: users file was not specified, so ignoring the specified viewer/configuration groups. \n\n" );
    }
    else
    {
        if ( overrideViewersGroup )
        {
            Settings.ViewersGroup = viewersGroup;
        }
        if ( overrideConfigGroup )
        {
            Settings.ConfigGroup = configGroup;
        }
    }

    if ( i != argc )
    {
        printf( "cam2web - streaming camera to web \n" );
        printf( "Version: %s \n\n", STR_INFO_VERSION );
        printf( "Available command line options: \n" );
        printf( "  -dev:<num>   Sets video device number to use. \n" );
        printf( "               Default is 0. \n" );
        printf( "  -size:<0-12> Sets video size to one from the list below: \n" );
        printf( "               0: 320x240 \n" );
        printf( "               1: 640x360 \n" );
        printf( "               2: 640x400 \n" );
        printf( "               3: 640x480 (default) \n" );
        printf( "               4: 1280x720 \n" );
        printf( "               Note: video device may switch to a different frame size, \n" );
        printf( "                     the one it supports. \n" );
        printf( "  -fps:<1-30>  Sets camera frame rate. Same is used for MJPEG stream. \n" );
        printf( "               Default is 30. \n" );
        printf( "  -jpeg:<num> JPEG quantization factor (quality). \n" );
        printf( "              Default is 85. \n" );
        printf( "  -port:<num>  Port number for web server to listen on. \n" );
        printf( "               Default is 8000. \n" );
        printf( "  -realm:<?>   HTTP digest authentication domain. \n" );
        printf( "               Default is 'cam2web'. \n" );
        printf( "  -htpass:<?>  htdigest file containing list of users to access the camera. \n" );
        printf( "               Note: only users for the specified/default realm are loaded. \n" );
        printf( "               Note: if users file is specified, then by default only users \n" );
        printf( "                     from that list are allowed to view camera and only \n" );
        printf( "                     'admin' user is allowed to change its settings. \n" );
        printf( "  -viewer:<?>  Group of users allowed to view camera: any, user, admin. \n" );
        printf( "               Default is 'any' if users file is not specified, \n" );
        printf( "               or 'user' otherwise. \n" );
        printf( "  -config:<?>  Group of users allowed to change camera settings. \n" );
        printf( "               Default is 'any' if users file is not specified, \n" );
        printf( "               or 'admin' otherwise. \n" );
        printf( "  -fcfg:<?>    Name of the file to store camera settings in. \n" );
        printf( "               Default is '~/.cam_config'. \n" );
        printf( "  -web:<?>     Name of the folder to serve custom web content. \n" );
        printf( "               By default embedded web files are used. \n" );
        printf( "  -title:<?>   Name of the camera to be shown in WebUI. \n" );
        printf( "               Use double quotes if the name contains spaces. \n" );
        printf( "  -device:<?>  Name of the camera device to be shown in WebUI. \n" );
        printf( "               Use double quotes if the name contains spaces. \n" );
        printf( "  -image:<?>   Output Image Type\n" );
        printf( "               by default value is 2 Depth Map, 0:Color, 1:Gray, 2:Depth Map, 3:Depth RAW+Gray\n" );
        printf( "  -angleX:<?>  Camera rotation angle X axis\n" );
        printf( "               by default value is 0, Set Camera Ratation Angle for 3D reconstruction\n" );
        printf( "\n" );

        ret = false;
    }

    return ret;
}

int main( int argc, char* argv[] )
{
    struct sigaction sigIntAction;

    SetDefaultSettings( );

    if ( !ParseCommandLine( argc, argv ) )
    {
        return -1;
    }

    // set-up handler for certain signals
    sigIntAction.sa_handler = sigIntHandler;
    sigemptyset( &sigIntAction.sa_mask );
    sigIntAction.sa_flags = 0;

    sigaction( SIGINT,  &sigIntAction, NULL );
    sigaction( SIGQUIT, &sigIntAction, NULL );
    sigaction( SIGTERM, &sigIntAction, NULL );
    sigaction( SIGABRT, &sigIntAction, NULL );
    sigaction( SIGTERM, &sigIntAction, NULL );

    // create camera object
    shared_ptr<XV4LCamera>           xcamera       = XV4LCamera::Create( );
    shared_ptr<IObjectConfigurator>  xcameraConfig = make_shared<XV4LCameraConfig>( xcamera );
    XObjectConfigurationSerializer   serializer( Settings.CameraConfigFileName, xcameraConfig );

    // some read-only information about the version
    PropertyMap versionInfo;

    versionInfo.insert( PropertyMap::value_type( "product", STR_INFO_PRODUCT ) );
    versionInfo.insert( PropertyMap::value_type( "version", STR_INFO_VERSION ) );
    versionInfo.insert( PropertyMap::value_type( "platform", STR_INFO_PLATFORM ) );

    // prepare some read-only informational properties of the camera
    PropertyMap cameraInfo;
    char        strVideoSize[32];
	char        strAngleX[16];

    sprintf( strVideoSize,      "%u", Settings.FrameWidth );
    sprintf( strVideoSize + 16, "%u", Settings.FrameHeight );
	sprintf(strAngleX, "%u", Settings.AngleX);

    cameraInfo.insert( PropertyMap::value_type( "device", Settings.CameraDevice ) );
    cameraInfo.insert( PropertyMap::value_type( "title",  Settings.CameraTitle ) );
    cameraInfo.insert( PropertyMap::value_type( "width",  strVideoSize ) );
    cameraInfo.insert( PropertyMap::value_type( "height", strVideoSize + 16 ) );
	cameraInfo.insert(PropertyMap::value_type("AngleX", strAngleX));

    // create and configure web server
    XWebServer          server( "", Settings.WebPort );
    XVideoSourceToWeb   video2web;
    UserGroup           viewersGroup = Settings.ViewersGroup;
    UserGroup           configGroup  = Settings.ConfigGroup;

    if ( !Settings.HtRealm.empty( ) )
    {
        server.SetAuthDomain( Settings.HtRealm );
    }
    if ( !Settings.HtDigestFileName.empty( ) )
    {
        server.LoadUsersFromFile( Settings.HtDigestFileName );
    }

    // set camera configuration
    xcamera->SetVideoDevice( Settings.DeviceNumber );
    xcamera->SetVideoSize( Settings.FrameWidth, Settings.FrameHeight );
    xcamera->SetFrameRate( Settings.FrameRate );
    xcamera->SetImageType( Settings.ImageType );

    if ( Settings.ImageType ==0 )
		printf( "Output ImageType is Color\n");
    if ( Settings.ImageType ==1 )
		printf( "Output ImageType is Grayscale\n");
    if ( Settings.ImageType ==2 )
		printf( "Output ImageType is Colored Depth map\n");
	if (Settings.ImageType == 3)
	{
//		Settings.JpegQuality =100;
		printf("Output ImageType is DepthRAW\n");
	}
	video2web.SetJpegQuality( Settings.JpegQuality );

    // restore camera settings
    // Do not restore configuration
//    serializer.LoadConfiguration( );

    // add web handlers
    server.AddHandler( make_shared<XObjectInformationRequestHandler>( "/version", make_shared<XObjectInformationMap>( versionInfo ) ) ).
           AddHandler( make_shared<XObjectConfigurationRequestHandler>( "/camera/config", xcameraConfig ), configGroup ).
           AddHandler( make_shared<XObjectInformationRequestHandler>( "/camera/properties", make_shared<XV4LCameraPropsInfo>( xcamera ) ), configGroup ).
           AddHandler( make_shared<XObjectInformationRequestHandler>( "/camera/info", make_shared<XObjectInformationMap>( cameraInfo ) ), viewersGroup ).
           AddHandler( video2web.CreateBmpHandler( "/camera/bmp" ), viewersGroup ).
           AddHandler( video2web.CreateJpegHandler( "/camera/jpeg" ), viewersGroup ).
           AddHandler( video2web.CreateMjpegHandler( "/camera/mjpeg", Settings.FrameRate ), viewersGroup );

    // use custom or embedded web content
    if ( !Settings.CustomWebContent.empty( ) )
    {
        server.SetDocumentRoot( Settings.CustomWebContent );
    }
    else
    {
    #ifdef NDEBUG
        // web content is embedded in release builds to get single executable
        server.AddHandler( make_shared<XEmbeddedContentHandler>( "/", &web_index_html ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "index.html", &web_index_html ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "styles.css", &web_styles_css ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "cam2web.png", &web_cam2web_png ) ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "cam2web_white.png", &web_cam2web_white_png ) ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "camera.js", &web_camera_js ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "cameraproperties.js", &web_cameraproperties_js ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "cameraproperties.html", &web_cameraproperties_html ), configGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "jquery.js", &web_jquery_js ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "jquery.mobile.js", &web_jquery_mobile_js ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "jquery.mobile.css", &web_jquery_mobile_css ), viewersGroup ).
               AddHandler( make_shared<XEmbeddedContentHandler>( "jquery.mobile-1.4.2.min.map", &web_jquery_mobile_min_map ), viewersGroup );
    #endif
    }

    // set camera listeners
    XVideoSourceListenerChain   listenerChain;
    CameraErrorListener         cameraErrorListener;

    listenerChain.Add( video2web.VideoSourceListener( ) );
    listenerChain.Add( &cameraErrorListener );
    xcamera->SetListener( &listenerChain );

    if ( server.Start( ) )
    {
        printf( "Web server started on port %d ...\n", server.Port( ) );
        printf( "Ctrl+C to stop.\n" );

        xcamera->Start( );

        while ( !ExitEvent.Wait( 60000 ) )
        {
            // save camera settings from time to time
            serializer.SaveConfiguration( );
        }

        serializer.SaveConfiguration( );

        xcamera->SignalToStop( );
        xcamera->WaitForStop( );
        server.Stop( );

        printf( "Done \n" );
    }
    else
    {
        printf( "Failed starting web server on port %d\n", server.Port( ) );
    }

    return 0;
}

