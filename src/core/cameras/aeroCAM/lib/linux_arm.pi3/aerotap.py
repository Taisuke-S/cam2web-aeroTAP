# -*- coding: utf-8 -*-
"""
Python library for aeroTAP 3D USB Camera

Created on Wed Aug  3 23:24:01 2022
@author: nextEDGE Technology

"""
__author__ = "nextEDGE Technology"
__copyright__ = "Copyright (C) 2022 nextEDGE Technology K.K."
__license__ = "Public Domain"
__version__ = "1.0"

import os
import sys
import ctypes
import time
from ctypes import *
from typing import List
from datetime import datetime
from ctypes import c_void_p, c_char_p
import numpy as np
import cv2
import struct
import ZDColorPalette

if os.name == "nt":
    from ctypes.wintypes import BOOL, HWND, LPARAM
    class HandResultWin(Structure):
	    _fields_ = [
        ('rect_x', c_int32),
        ('rect_x', c_int32),
        ('rect_y', c_int32),
        ('rect_width', c_int32),
        ('rect_height', c_int32),
        ('rectArea_x', c_int32),
        ('rectArea_y', c_int32),
        ('rectArea_width', c_int32),
        ('rectAera_height', c_int32),
        ('used', ctypes.wintypes.BOOL),
        ('centerX', ctypes.wintypes.INT),
        ('centerY', ctypes.wintypes.INT),
        ('centerZ', ctypes.wintypes.INT),
        ('centerLastX', ctypes.wintypes.INT),
        ('centerLastY', ctypes.wintypes.INT),
        ('centerLastZ', ctypes.wintypes.INT),
        ('clock', c_uint64),
        ('clockStart', c_uint64),
        ('clockStop', c_uint64),
        ('clockMove', c_uint64),
        ('objectType', ctypes.wintypes.INT),
        ('detected', ctypes.wintypes.INT),
        ('moving', ctypes.wintypes.INT),
        ('movedCount', ctypes.wintypes.INT),
        ('deltaX', ctypes.wintypes.INT),
        ('deltaY', ctypes.wintypes.INT),
        (       'rectGesture_x', c_int32),
        ('rectGesture_y', c_int32),
        ('rectGesture_width', c_int32),
        ('rectGesture_height', c_int32),
        ('effectType', ctypes.wintypes.INT),
        ('diff', ctypes.wintypes.INT),
        ('posDepthX', ctypes.wintypes.INT),
        ('posDepthY', ctypes.wintypes.INT),
        ('posDepthZ', ctypes.wintypes.INT),
        ('nPID', ctypes.wintypes.INT),
        ]
#  struct.Struct("=iiiiiiiiiliiiiiiqqqqiiiiiiiiiiiiiiii")


class BITMAPINFOHEADER(Structure):
    _fields_ = [
        ('biSize',        c_uint32),
        ('biWidth',       c_int32),
        ('biHeight',      c_int32),
        ('biPlanes',      c_uint16),
        ('biBitCount',    c_uint16),
        ('biCompression', c_uint32),
        ('biSizeImage',   c_uint32),
        ('biXPPM',        c_int32),
        ('biYPP',         c_int32),
        ('biClrUsed',     c_uint32),
        ('biClrImportant',c_uint32),
        ('bfType',        c_uint16),
        ('bfSize',        c_uint32),
        ('bfReserved1',   c_uint16),
        ('bfReserved2',   c_uint16),
        ('bfOffBits',     c_uint32),
    ]
#   struct.Struct("=IllHHIIllII")
class HandResult(Structure):
	_fields_ = [
    ('rect_x', c_int),
    ('rect_y', c_int),
    ('rect_width', c_int),
    ('rect_height', c_int),
    ('used', c_bool),
    ('centerX', c_int),
    ('centerY', c_int),
    ('centerZ', c_int),
    ('clock', c_ulong),
    ('clockStart', c_ulong),
    ('objectType', c_int32),
    ('nDetected', c_int32),
    ]
#  struct.Struct("=llllIiiiiqqii")
    
"""
def IsConnected():
    Check is aeroTAP camera is connected and ready
    returns True when it is ready
    Only works for Win
"""
def IsConnected():
    global lib,obj,os

    if os.name == "nt":
        lib.InitAeroCam.restype = ctypes.wintypes.INT
        return lib.AERO_IsCameraConnected()
    # Linux
    lib.aerotap_checkDevice.restype = ctypes.c_bool
    lib.aerotap_checkDevice.argtypes = [ctypes.c_void_p]
    if  not lib.aerotap_checkDevice(obj):
	    return False;
    return True;
    
"""
def Init():
    Initialize aeroTAP SDK with parameters
    returns True when it is ready
"""
def Init():
    global lib,os,camID ,camMode,camFPS,bEnableMJPEG,nIsUSB30,bEnableDepthFilter
    if os.name == "nt":
        lib.InitAeroCam.restype = ctypes.wintypes.BOOL
        lib.InitAeroCam.argtypes = [ctypes.wintypes.HWND,ctypes.wintypes.UINT,ctypes.wintypes.INT,ctypes.wintypes.INT, ctypes.wintypes.LPBYTE,ctypes.wintypes.LPBYTE,ctypes.wintypes.LPBYTE, ctypes.c_uint32]
        if not lib.InitAeroCam(None,0,camID,camFPS,None,None, None, camMode):
            return False
        return True

    # Linux
    lib.aerotap_setFPS.argtypes = [ctypes.c_int]
    lib.aerotap_setFPS.argtypes = [ctypes.c_void_p,ctypes.c_int]
    lib.aerotap_setFPS(obj,camFPS)

    lib.aerotap_useMJPG.argtypes = [ctypes.c_bool]
    lib.aerotap_useMJPG.argtypes = [ctypes.c_void_p,ctypes.c_bool]
    lib.aerotap_useMJPG(obj,bEnableMJPEG)

    if nIsUSB30 ==1:
        lib.aerotap_setUSB20.argtypes = [ctypes.c_bool]
        lib.aerotap_setUSB20.argtypes = [ctypes.c_void_p,ctypes.c_bool]
        lib.aerotap_setUSB20(obj,True)

    lib.aerotap_setFilter.argtypes = [ctypes.c_void_p,ctypes.c_int]
    if bEnableDepthFilter:
        lib.aerotap_setFilter(obj,1)
    else:
        lib.aerotap_setFilter(obj,0)
    return True;

"""
def StartCam(width,height):
    Start camera streaming with resolution
    returns True 
"""
def StartCam(width,height):
    global lib,obj,os,camID ,camMode,camFPS,camWidth,camHeight

    if os.name == "nt":
        lib.AERO_StartCam.restype = ctypes.wintypes.BOOL
        lib.AERO_StartCam.argtypes = [ctypes.wintypes.INT]
        if not lib.AERO_StartCam(camID):
            return False
        SetResolution(width,height)
        return True
    # Linux
    lib.aerotap_open.restype = ctypes.c_bool
    lib.aerotap_open.argtypes = [ctypes.c_void_p, ctypes.c_char_p,ctypes.c_char_p,ctypes.c_int,ctypes.c_int]
    if not lib.aerotap_open(obj,None, None, camWidth, camHeight):
        return False

    lib.aerotap_start.restype = ctypes.c_bool
    lib.aerotap_start.argtypes = [ctypes.c_void_p]
    if not lib.aerotap_start(obj):
        return False

    return True


"""
def Close():
    Stop and Release resource
"""
def Close():
    global lib,obj,os,camID ,camMode,camFPS

    if os.name == "nt":
        lib.AERO_StopCam.restype = ctypes.wintypes.BOOL
        lib.AERO_StopCam.argtypes = [ctypes.wintypes.INT]
        lib.AERO_StopCam(camID)
    
        lib.ExitAeroCam()
        return None

    # Linux
    lib.aerotap_stop.argtypes = [ctypes.c_void_p]
    lib.aerotap_stop(obj)
    
    lib.aerotap_delete.argtypes = [ctypes.c_void_p]
    lib.aerotap_delete(obj)
    return None    
    
"""
def SetResolution(width,height)
    Set Camera Resolution
    
"""
def SetResolution(width,height):
    global lib,camID,os ,camMode,camFPS,zdTableLen

    camWidth= width
    camHeight = height
    LoadZDTable()
    print("Set Resolution", camWidth, camHeight,zdTableLen)
    if not os.name == "nt":
        return True
        
    lib.AERO_SetResolution.restype = ctypes.wintypes.BOOL
    lib.AERO_SetResolution.argtypes = [ctypes.wintypes.UINT,ctypes.wintypes.UINT,ctypes.wintypes.UINT]
    if not lib.AERO_SetResolution(camID,width,height):
        return False
    return True

"""
def LoadZDTable()
    Load ZDTable for 8 or 11bit disparity depth map
    return 0 when ZDTable is not required and depth map is 14bit    
"""
def LoadZDTable():
    global lib,obj,os, camID ,camMode,camFPS,zdTable,zdTableLen
    
    if os.name == "nt":
        lib.AERO_LoadZDTable.restype = ctypes.wintypes.INT
        lib.AERO_LoadZDTable.argtypes = [ctypes.POINTER(ctypes.c_int32),ctypes.wintypes.INT]

        buffer = (ctypes.c_int32* 2048)()
        buffer_p = cast(buffer,ctypes.POINTER(ctypes.c_int32))
        zdTableLen = lib.AERO_LoadZDTable(buffer_p, 0)
        zdTable.clear()
        for i in range( zdTableLen):
            zdTable.append(buffer[i])
#    print("zdTableLen = {}",format(zdTableLen))
#    print(zdTable)
        return zdTableLen

    # Linux
    lib.aerotap_getZDTable.restype = ctypes.c_int
    lib.aerotap_getZDTable.argtypes = [ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
    zdTableLen = lib.aerotap_getZDTable(obj,zdTable)
    return zdTableLen


"""
def GetPType()
    Get Product tpye of connected aeroTAP 3D USB Camera
    return 0,1,3
"""
def GetPType():
    global lib,obj,os, camID ,camMode,camFPS

    if os.name == "nt":
        lib.AERO_GetPType.restype = ctypes.wintypes.INT
        lib.AERO_GetPType.argtypes = [ctypes.wintypes.INT]
        return lib.AERO_GetPType(camID)

    lib.aerotap_getPType.restype = ctypes.c_int
    lib.aerotap_getPType.argtypes = [ctypes.c_void_p]
    return lib.aerotap_getPType(obj)

"""
def IsUSB30()
    Check current connected USB mode
    returns 0,1,2
"""
def IsUSB30():
    global lib,obj,os,camID

    #// Return code -1 = error, 0 = unknown, 1= USB2.0,  2=USB3.0
    #typedef int(__cdecl *AERO_IsUSB30)(int nCamNo);

    if os.name == "nt":
        lib.AERO_IsUSB30.restype = ctypes.wintypes.INT
        lib.AERO_IsUSB30.argtypes = [ctypes.wintypes.INT]
        return lib.AERO_IsUSB30(camID)

    lib.aerotap_getUSB20.restype = ctypes.c_bool
    lib.aerotap_getUSB20.argtypes = [ctypes.c_void_p]
    bUSB20 = lib.aerotap_getUSB20(obj)
    if bUSB20:
        return 1
    return 2


"""
def EnableHWPostProcess(mode)
    Enable/Disable HWPostProcess
    Function is only valid for aeroTAP 3D USB G2
"""
def EnableHWPostProcess(mode):
    global lib,camID ,os, camMode,camFPS

    if os.name == "nt":
        lib.AERO_EnableHWPostProcess.argtypes = [ctypes.wintypes.INT,ctypes.wintypes.BOOL]
        lib.AERO_EnableHWPostProcess(camID,mode)
    # Linux
    # Alwasy On if HW support it
    
"""
def EnableDepthFilter(mode)
    Enable/Disable SWPostProcess
"""
def EnableDepthFilter(mode):
    global lib,camID ,os, camMode,camFPS

    if os.name == "nt":
        lib.AERO_EnableDepthFilter.argtypes = [ctypes.wintypes.INT,ctypes.wintypes.BOOL]
        lib.AERO_EnableDepthFilter(camID,mode)
    # Linux
    # does not suport to change it after start streaming

    
"""
def IsNewFrame()
    Check if New Frame is available
    returns Trues when it is ready
"""
def IsNewFrame():
    global lib,obj,os, camID ,camMode,camFPS

    if os.name == "nt":
        lib.AERO_IsNewFrame.restype = ctypes.wintypes.BOOL
        lib.AERO_IsNewFrame.argtypes = [ctypes.wintypes.INT]
        if lib.AERO_IsNewFrame(camID):
            return True
        return False
    # linux
    lib.aerotap_isNewFrame.restype = ctypes.c_bool
    lib.aerotap_isNewFrame.argtypes = [ctypes.c_void_p]
    if lib.aerotap_isNewFrame(obj):
        return True
    return False


"""
def GetLastError()
    Call this methos if IsNewFrame() gets error to determin it is connection eror or ont
    returns 1 when camera gets connection error
"""
def GetLastError():
    global lib,obj,os, camID ,camMode,camFPS

    if os.name == "nt":
        lib.AERO_GetLastError.restype = ctypes.wintypes.INT
        return lib.AERO_GetLastError()
    # Linux
    lib.aerotap_isConnectionLost.restype = ctypes.c_bool
    lib.aerotap_isConnectionLost.argtypes = [ctypes.c_void_p]
    if lib.aerotap_isConnectionLost(obj):
        return 1
    return 0


"""
def Read(mode)
    Read color image from camera
    mode =0: Color Image, mode ==1: Grayscale
    returns np 3dim in BGR or 1dim Grayscale image
"""
def Read(mode):
    global lib,obj,os,camID ,img_buffer,depth_buffer,camWidth,camHeight,cBMIH 

    if mode>1:
        mode =0
    if os.name == "nt":
        lib.AERO_GetImage.restype = ctypes.wintypes.BOOL
        lib.AERO_GetImage.argtypes = [ctypes.POINTER(ctypes.wintypes.LPBYTE),ctypes.wintypes.INT,ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT)]
#    typedef BOOL (__cdecl *AERO_GetImage)(BITMAPINFOHEADER * pBuffer,int nType,int *nMax/*=NULL*/, int *nTotal/*=NULL*/,int *nCount/*=NULL*/, int *nFrame/*=NULL*/, int *nP/*=NULL*/, int *nBlackout/*=NULL*/);

        if not lib.AERO_GetImage(img_buffer.ctypes.data_as(ctypes.POINTER(ctypes.wintypes.LPBYTE)),mode,None,None,None,None,None,None):
            print("GetImage Error")
            return None
     # read bmih
        BMIH = struct.Struct("=IllHHIIllII")
        bmih = BMIH.unpack(img_buffer[:40])
        cBMIH.biSize = bmih[0]
        cBMIH.biWidth = bmih[1]
        cBMIH.biHeight = bmih[2]
        cBMIH.biBitCount = bmih[4]
        cBMIH.biSizeImage = bmih[6]
        if mode ==0 :
            deserialized_resized = img_buffer[40:camWidth*camHeight*3+40]
            deserialized = np.reshape(deserialized_resized, newshape=(camHeight, camWidth,3))
            deserialized = np.flipud(deserialized)
        else:
            deserialized_resized = img_buffer[40:camWidth*camHeight+40]
            deserialized = np.reshape(deserialized_resized, newshape=(camHeight, camWidth,1))

    else:
        cBMIH.biWidth = camWidth
        cBMIH.biHeight = camHeight
        cBMIH.biBitCount = 24
        cBMIH.biSizeImage = camWidth*camHeight*3
        
        if mode==0:
            lib.aerotap_getColorData.restype = ctypes.POINTER(ctypes.c_uint8)
            lib.aerotap_getColorData.argtypes = [ctypes.c_void_p]
            deserialized = np.reshape(np.ctypeslib.as_array( lib.aerotap_getColorData(obj),shape=(1,camWidth*camHeight*3)), newshape=(camHeight, camWidth,3))
            deserialized = deserialized[:,:,::-1] # to BGR
        else:
            lib.aerotap_getGrayData.restype = ctypes.POINTER(ctypes.c_uint8)
            lib.aerotap_getGrayData.argtypes = [ctypes.c_void_p]
            deserialized = np.reshape(np.ctypeslib.as_array( lib.aerotap_getGrayData(obj),shape=(1,camWidth*camHeight)), newshape=(camHeight, camWidth,1))
    # RGB to BGR for OpenCV
#    deserialized = np.fliplr(deserialized)
#    print(deserialized.shape)

    return deserialized

"""
def ReadDepth():
    read de@thMap from camera
    returns depthmap in np dmin=1  16bit data
"""
def ReadDepth():
    global lib,obj,os, camID ,img_buffer,depth_buffer,camWidth,camHeight,bEnableDepthFilter ,camMode,zdTable

    if not os.name == "nt":
    # Linux
        lib.aerotap_getDepthData.restype = ctypes.POINTER(ctypes.c_uint16)
        lib.aerotap_getDepthData.argtypes = [ctypes.c_void_p]
        depth = np.frombuffer(np.ctypeslib.as_array(lib.aerotap_getDepthData(obj),shape=(1,camWidth*camHeight)),dtype=np.uint16)
        return depth

# for dmin2
        deserialized = np.reshape(depth, newshape=(camHeight, camWidth,1))
        return deserialized

    # Windows    
    lib.AERO_GetImage.restype = ctypes.wintypes.BOOL
#    lib.AERO_GetImage.argtypes = [ctypes.POINTER(ctypes.wintypes.LPBYTE),ctypes.wintypes.INT,ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT)]
    lib.AERO_GetImage.argtypes = [ctypes.POINTER(ctypes.c_uint8),ctypes.wintypes.INT,ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT),ctypes.POINTER(ctypes.wintypes.INT)]
#    typedef BOOL (__cdecl *AERO_GetImage)(BITMAPINFOHEADER * pBuffer,int nType,int *nMax/*=NULL*/, int *nTotal/*=NULL*/,int *nCount/*=NULL*/, int *nFrame/*=NULL*/, int *nP/*=NULL*/, int *nBlackout/*=NULL*/);

    if not bEnableDepthFilter:
# Depth RAW
        if not lib.AERO_GetImage(depth_buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),8,None,None,None,None,None,None):
            print("GetImage Error")
            return None
    else:
# Depth Filtered
        if not lib.AERO_GetImage(depth_buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),9,None,None,None,None,None,None):
            print("GetImage Error")
            return None
    if camMode ==10 and not bEnableDepthFilter:
        deserialized_resized = depth_buffer[40:camWidth*camHeight+40]
        buffer = deserialized_resized.tobytes()

        # convert disparity to actual depth value using zdTable
        depth16 = np.empty(camWidth*camHeight,dtype=np.uint16)
        for index,value in enumerate(buffer):
            if value>0 :
                depth16[index] = zdTable[value]

#        deserialized = np.reshape(np.frombuffer(depth16,dtype=np.uint8), newshape=(camHeight, camWidth,2))
        return depth16
    else:
        deserialized_resized = depth_buffer[40:camWidth*camHeight*2+40]
        return deserialized_resized
# for dmin2
#        buffer = deserialized_resized.tobytes()
#        deserialized = np.reshape(np.frombuffer(buffer,dtype=np.uint16), newshape=(camHeight, camWidth,1))
#        return deserialized

"""
def DepthToRGB(depth):
    input: 16bit depthmap
    returns color image in np dmin=3  BGR
"""
def DepthToRGB(depth):
    global lib,camID ,os, imgDepth,depth_buffer,camWidth,camHeight,bEnableDepthFilter ,camMode,ColorPalette

    deserialized = np.reshape(np.frombuffer(depth,dtype=np.uint8),newshape=(camHeight,camWidth,2))
    imgDepth[:,:,1:3] = deserialized
    return None

# Slow when coverting using ColorPalette table
#    depth =np.array( [ColorPalette[value] for value in depth],dtype=np.uint16)
#    imgDepth = np.reshape(np.frombuffer(depth.tobytes(),dtype=np.uint8),newshape=(camHeight,camWidth,4))
#    imgDepth[:,:,0:3] = deserialized[:,:,0:3]
    return None
    
# or     
#    for i,value in enumerate(depth):
#        if value>0 and value<16384:
#            depth[i] = ColorPalette[value]
#    deserialized = np.reshape(np.frombuffer(depth,dtype=np.uint8),newshape=(camHeight,camWidth,2))
#    imgDepth[:,:,1:3] = deserialized
#    return None

    
"""
def UpdateFrame():
    call this metohs to make next frame available
"""
def UpdateFrame():
    global lib,obj, os, camID

    if os.name == "nt":
#   typedef void(__cdecl *AERO_UpdateFrame)(int nDevice);
        lib.AERO_UpdateFrame.argtypes = [ctypes.wintypes.INT]
        return lib.AERO_UpdateFrame(camID)
    else:
# Linex
        lib.aerotap_updateFrame.argtypes = [ctypes.c_void_p]
        return lib.aerotap_updateFrame(obj)


#
#  aerotap 
#
#
if os.name == "nt":
    cwd = os.path.dirname(__file__)
    os.environ['PATH'] = cwd + ';' + os.environ['PATH']
    aerodll = os.path.join(cwd, "aeroTAP_CAM.dll")
    envKeys = list()
    for k, v in os.environ.items():
        envKeys.append(k)
    try:
        if not os.path.exists(aerodll):
            raise ValueError("NoDLL")
        lib = ctypes.WinDLL(aerodll)
    except (KeyError, ValueError):
        print("Error loading aeroTAPA_CAM.DLL {ValueError}")
else:
    lib = ctypes.CDLL("./libaeroTAP-sdk.so")
    lib.aerotap_create.restype = c_void_p
    obj = lib.aerotap_create()

print("Loaded aeroTAPA_CAM.DLL")

BMlen = 40 #sys.getsizeof(BITMAPINFOHEADER())
#print(BMlen)

# Globals 
# Win supports 0-3, Linux supports only 0
camID =0 # caera ID 0 to 3
camMode = 10 # 8bit Depth
camFPS =30
camWidth=640
camHeight = 480
nColorImage = 0 # 1:Color image 24bit bgr , 1: Grayscale 1bit
nIsUSB30 =0 # 0: follow current connection mode

# PType 3==G1, 1== GS
nPType =-1 # Product Type
# Enble/Distable HW PostProcess only for aeroTAP 3D USB G2 camera
bEnableHWPostProcess = True
# Enble/Distable Simple Depth Poast Process
bEnableDepthFilter = True
# Enble MJPEG streaming for Color Image. Only for USB2.0 mode
bEnableMJPEG = False
# zdTable is required for 8bit/11bit depth map
zdTableLen =0
zdTable = [0]

#
#  Allocate Image Buffers
cBMIH = BITMAPINFOHEADER()
cHandResult = HandResult()

img_buffer = np.empty(1280*720*3+BMlen,dtype=np.uint8)
depth_buffer = np.empty(1280*720*2+BMlen,dtype=np.uint8)

ColorPalette = ZDColorPalette.BuildColorPaletteValue()


# 
#

if __name__ == "__main__":
    
   if os.name == "nt":
       if not IsConnected():
            raise ValueError("Error aeroTAP camera is not connected")

   nPType = GetPType()
   if nPType ==3:
     camMode = 12 # 14Bit
     print("Running aeroTAP 3D USB G2 Camera")
     # always eable HE Postprocess
     EnableHWPostProcess(bEnableHWPostProcess)
   elif nPType ==0:
     print("Running aeroTAP 3D USB G1 Camera")
   elif nPType ==1:
     camMode = 12 # 14Bit
     print("Running aeroTAP 3D USB GS Camera")

   # if USB mode is not defined by user
   if os.name == "nt":
       if nIsUSB30 ==0:
          nIsUSB30 = IsUSB30()
   else:
       if nIsUSB30 ==0:
          lib.aerotap_getUSB20.argtypes = [ctypes.c_void_p]
          if lib.aerotap_getUSB20(obj):
              nIsUSB30 = 1 # USB 2.0
          else:
              nIsUSB30 = 2 # USB 3.0
       
       
   if nIsUSB30 ==1:
     # fore to enable MJPEG mode when USB20 is used
     bEnableMJPEG = True
     print("Connecting to USB2.0")
   elif nPType ==2:
     print("Connecting to USB3.0")

   if bEnableMJPEG:
     camMode = 110 # bEnableMJPEG

   if not Init():
     raise ValueError("Error aeroInit")

   
   EnableDepthFilter(bEnableDepthFilter)
   print("Starting camera...")      
   if not StartCam(camWidth,camHeight):
     raise ValueError("Camera Start Error")
      
   imgDepth = np.full((camHeight,camWidth,3),0,dtype=np.uint8)

# wait for 1st frame
   for i in range(100):
      if IsNewFrame():
          img = Read(nColorImage)
#          depth = ReadDepth()
          UpdateFrame()
          print("width = ",cBMIH.biWidth," height= ",cBMIH.biHeight ," imageSize = ", cBMIH.biSizeImage)
          break;
      time.sleep(1)
# Loop Start          
   for i in range(200):
      if IsNewFrame():

           img = Read(nColorImage)   # Color Image
           depth = ReadDepth()  # depth map 16 bit

           UpdateFrame()

           if( not (img is None) ):
               cv2.imshow( "aeroTAP camera View", img )
           if( not (depth is None) ):
           # convert depth map to bgr ColorImage
               DepthToRGB(depth)
               cv2.imshow( "aeroTAP camera DepthMap", imgDepth )
           key = cv2.waitKey(1)
           if key == ord("q"):
              break
      else:
          if GetLastError()!=0:
            print("Lost connection\n");
            break;
          print("no frame")
#          continue

      time.sleep(0.033)
# Loop end
Close()
cv2.destroyAllWindows() 
    
  
