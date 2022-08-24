# Customization for aeroTAP 3D SUB camera

Modified cam2web for aeroTAP 3D USB camera.
It supports Linux ( x64,Arm, Aarch64 cpu) only.
 
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

### Known issues
 - Segmentation faulut when startup
   restart cam2web
 - Use USB2.0 port even if device has USB3.0 port for SBCs (Raspberry Pi, NanoPi )
   You mey see the following error message when access to server right after starting.
    VIDIOC_DQBUF error EAGAIN (11) fd=X
