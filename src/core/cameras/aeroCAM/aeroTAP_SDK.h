#ifndef __AEROTAP_HEADER_INCLUDED__
#define __AEROTAP_HEADER_INCLUDED__

#include <functional>
#include <stdint.h>
#include <semaphore.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char * LPBYTE;
typedef unsigned char  BYTE;
typedef uint16_t WORD;
typedef uint32_t  DWORD;

namespace aerotap
{
	enum {
		HANDMODE_NONE=0,
		HANDMODE_CLICK=10,			// Close to Click
		HANDMODE_DD=11,			// Drag & Drop
		HANDMODE_PUSH=1			// Push to Click
	};
	enum {
		GESTURE_NONE=0,
		GESTURE_CIRCLE_L = -1,
		GESTURE_CIRCLE_R = 1,
		GESTURE_FLIP_L =2,
		GESTURE_FLIP_R,
		GESTURE_FLIP_U,
		GESTURE_FLIP_D
	};
	
struct RECT
{
	int32_t left;
	int32_t right;
	int32_t top;
	int32_t bottom;
};

typedef struct rvRect
{
    int32_t x;
    int32_t y;
    int32_t width;
    int32_t height;
}
rvRect;

struct POINT
{
	int32_t x,y;
};
typedef struct _POINT3I
{
	int32_t x;
	int32_t y;
	int32_t z;
} POINT3I;
struct POINT3F
{
	float x;
	float y;
	float z;
} ;

typedef struct _SYSTEMTIME {
  WORD wYear;
  WORD wMonth;
  WORD wDayOfWeek;
  WORD wDay;
  WORD wHour;
  WORD wMinute;
  WORD wSecond;
  WORD wMilliseconds;
} SYSTEMTIME, *PSYSTEMTIME;

typedef struct _HandResult
{
	RECT rect;
	bool bUsed;
	POINT3I pCenter;
	clock_t clock;
	clock_t clockStart;
	int nObjectType;
	int nDetected;
} HANDRESULT;
	
struct aeroTAPdata {
	unsigned char *pColor[2];
	unsigned char *pGray[2];
	unsigned char *pDepth[2];
	unsigned char *pDepthRAW[2];
	unsigned char *pGray320x240;
	unsigned char *pDepth320x240;
	unsigned char *pDepth80x60;
	int nBufferId;
	char szVideo0[256];
	char szVideo1[256];
	bool bUseMJPG;
	bool bUserBuffer;
	uint16_t nZDTableLen;
	int camWidth, camHeight;
	int nStreamIndex;
	int nDepthIndex;
	int nPType;
	int nPID;
	int nFPS;
	int nCamNo;
	bool bUSB20;
	int nMaxDepth;
	int nFilter;
	float nFocalLengthHeight;
	float nFocalLengthWidth;
	bool bMirror;
	bool bDoDetection;
	int nScale;
	int nAngle;
	int nRotate;
	bool bCrop;
	int  nRunning;
	void *pPalmTracker;
	POINT *pZoomTable;
	uint16_t nZDTable[2048];
};

	struct GestureState
	{
		int nState;
		int nRep;
		int nDir;
	};
	
	struct OutputMode
	{
		int fps;
		int xres;
		int yres;
		float hfov;
		float vwfov;
	};


	class aeroTAP
	{
	public:
		aeroTAP ();
		~aeroTAP();
		void setVerbose(bool bEnable);
		int getResWidth();
		int getResHeight();
		bool open(const char *video0, const char *video1, int width, int height);
		bool start();
		void stop();
		bool isNewFrame();
		void updateFrame();
		void setFilter(int nFilter) { aeroData.nFilter = nFilter; };
		void useMJPG(bool bMode) { aeroData.bUseMJPG = bMode; };
		bool getUSB20();
		void setUSB20(bool bUSB20) { aeroData.bUSB20 = bUSB20; };
		// Module Path to load ZDTable txt file
		void setModulePath(void *module) { modulePath=module; };
		uint8_t*getColorData();
		uint8_t*getGrayData();
		uint16_t*getDepthData();
		uint16_t*getDepthSmallData();
		// Set User Buffer to receive data directly
		void setBuffers(unsigned char *pColor[],unsigned char *pGray[], unsigned char*pDepthW[], unsigned char *pDepthRAW[]);
		uint8_t getBufferID();
		uint16_t getZDTable(uint16_t *pTable);
		int getPType() { return aeroData.nPType; };
		void setFPS(int fps) { aeroData.nFPS = fps; };
		bool getUserData(void *pUserData);
		uint16_t getPID();


		// Check /dev/video list to find aeroTAP Camera
		bool checkDevice();
		bool isConnectionLost();

		bool isRunning();

		// PalmTracker
		bool enablePalmTracker(bool bMode);
		void doPalmTracking();
		//
		// vector<rvRect>matchList;
		int  doPalmDetection(unsigned char*pGraySrc,RECT *pROI, int nMinS,int nMaxS,void *matchList);
		void setPalmDetectMode(int nMode);
		bool getPalmTrackResult(HANDRESULT *hands);
		void setZoomTable(int nZoom,  int cx, int cy);
		uint8_t*getQVGAData();

		// Depth Tracker
		bool getNearestPos(POINT3I *pos);
	private:
		void projectionToReal(int32_t *x, int32_t *y);
		
		aeroTAPdata aeroData;
		void *modulePath;

		RECT cropQVGA;
		RECT viewROI;
		POINT viewCenter;
		int nViewZoom;
		POINT zoomTable[320*240];
	};

};
#ifdef __cplusplus
}
#endif

#endif
