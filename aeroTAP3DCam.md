# aeroTAP 3D USB camera support

Modified cam2web for aeroTAP 3D USB camera.
It supports Linux ( x64,Arm, Aarch64) only.

## Support aeroTAP3D Camera
- aeroTAP 3D Camera
- aeroTAP 3D G2 Camera
- aeroTAP 3D GS Camera
 
### Device Tested
- Raspberry pi3, Raspberry Pi4
- Nano Pi NEO

## Building
Before cam2web can be built in release configuration, it is required to replace some source files which are related to aeroTAP camera.

It uses aeroTAP-sdk which is located in /src/core/cameras/aeroCAM/lib. It should be moved to shared library folder or set as shared.

## for Raspberry pi3
Edit MAKEFILE
```
export LD_LIBRARY_PATH=~/cam2web-aeroTAP/src/core/cameras/aeroCAM/linux_arm.pi3 
```

## Build

```
pushd .
cd src/tools/web2h/make/gcc/
make
popd

pushd .
cd src/apps/linux/
# or cd src/apps/pi/
make
popd
```

Note: libjpeg development library must be installed for cam2web build to succeed (which may not be installed by default) :
```
sudo apt install libusb-dev
sudo apt install libudev-dev freeglut3-dev
sudo apt install libv4l-dev
sudo apt install libpng-dev libjpeg-dev

```
##Dependency
The following modues are also required for Raspi/Nanopi
```
sudo apt-get install libv4l-dev
```

### Modifications
- Supports http://IP:PORT/camera/bmp
  to get DepthMap 16bit + Grayscale 8bit in BMP image
- Command Line parameters
  -image. -jpeg, -angleX   
- Supports config ImageType value set via http POST
  i.e JSON format like {"ImageType":"0"}  
```
Added /src/core/cameras/aeroTAP folder
    *All files should be copy over to /L4L2 folder to build
/src/apps/linux/cam2web.cpp
/src/core.XVideoSourceToWeb.cpp
/src/core.XVideoSourceToWeb.hpp
/src/core.XJpegEncoder.hpp
```

### Support Resolution
You can select resolution by setting -size:X parameter.
0:320x240 (G1, G2 Only)
1:640x360 (G2, GS Only)
2:640x400 (G1 Only)
3:640x480 (Default)
4:1270x720 (G2, GS Only)



### Known issues
 - Segmentation faulut when startup
   restart cam2web
 - Use USB2.0 port even if device has USB3.0 port for SBCs (Raspberry Pi, NanoPi )
   You mey see the following error message when access to server right after starting.
    VIDIOC_DQBUF error EAGAIN (11) fd=X

For more information about aeroTAP 3D USB Camera, please visit project web site.
https://www.aerotap.com
