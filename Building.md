# Building cam2web for aeroTAP 3D Camera

## Support aeroTAP3D Camera
- aeroTAP 3D Camera
- aeroTAP 3D G2 Camera
- aeroTAP 3D GS Camera

Before cam2web can be built in release configuration, it is required to replace some source files which are related to aeroTAP camera.

It uses aeroTAP-sdk which is located in /src/core/cameras/aeroCAM/lib. It should be moved to shared library folder or set as shared.

## for Raspberry pi
export LD_LIBRARY_PATH=~/cam2web-aeroTAP/src/core/cameras/aeroCAM/linux_arm 
## for Nano pi
export LD_LIBRARY_PATH=~/cam2web-aeroTAP/src/core/cameras/aeroCAM/linux_aarch64 
## for Linux
export LD_LIBRARY_PATH=~/cam2web-aeroTAP/src/core/cameras/aeroCAM/linux64 

## Building on Linux and Raspberry Pi
aeroTAP camera related modifications are located in src/core/cameras/aeroCAM. Copy over all files to ~/cam2web-aeroTAP/src/core/cameras/V4L2.

Edit  /src/apps/linux/Makefile Line:95
## for Raspi
        $(COMPILER) -o $@ $(OBJ) $(LDFLAGS) -L/home/pi/cam2web-aeroTAP/src/core/cameras/aeroCAM/lib/linux_arm
## for Nano pi
        $(COMPILER) -o $@ $(OBJ) $(LDFLAGS) -L/home/pi/cam2web-aeroTAP/src/core/cameras/aeroCAM/lib/linux_aarch64
## for Linux (Ubuntu)
        $(COMPILER) -o $@ $(OBJ) $(LDFLAGS) -L/home/pi/cam2web-aeroTAP/src/core/cameras/aeroCAM/lib/linux64


## Build
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
sudo apt-get install libjpeg-dev
```
##Dependency
The following modues are also required for Raspi/Nanopi
sudo apt-get libraspberrypi-dev
sudo apt-get libudev-dev
sudo apt-get install libv4l-dev
