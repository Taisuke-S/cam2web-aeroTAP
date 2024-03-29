#
#   cam2web - streaming camera to web
#
#   Copyright (C) 2017, cvsandbox, cvsandbox@gmail.com
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License along
#   with this program; if not, write to the Free Software Foundation, Inc.,
#   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#

# find out the platform on which we're running
MACHINE = $(shell uname -m)
ifneq (,$(findstring x86_64,$(MACHINE)))
	HOST_PLATFORM = x64
else ifneq (,$(findstring x86,$(MACHINE)))
	HOST_PLATFORM = x86
else ifneq (,$(findstring i686,$(MACHINE)))
	HOST_PLATFORM = x86
else ifneq (,$(findstring i386,$(MACHINE)))
	HOST_PLATFORM = x86
else ifneq (,$(findstring arm,$(MACHINE)))
	HOST_PLATFORM = arm
else ifneq (,$(findstring aarch64,$(MACHINE)))
	HOST_PLATFORM = aarch64
else
	DUMMY:=$(error Can't determine host platform)
endif

# Additional folders to look for source files
VPATH = ../../../externals/mongoose/ \
        ../../core \
        ../../core/cameras/V4L2

# C code
SRC_C = mongoose.c 
# C++ code
SRC_CPP = cam2web.cpp XImage.cpp XJpegEncoder.cpp XManualResetEvent.cpp \
    XV4LCamera.cpp XV4LCameraConfig.cpp XVideoSourceToWeb.cpp XWebServer.cpp \
    XSimpleJsonParser.cpp XObjectConfigurationSerializer.cpp \
    XObjectConfigurationRequestHandler.cpp XStringTools.cpp \
    XError.cpp

# Output name    
OUT = cam2web

# Compiler to use
COMPILER = g++
# Base compiler flags
CFLAGS = -std=c++0x

# Object files list
OBJ = $(SRC_CPP:.cpp=.o) $(SRC_C:.c=.o)

# Additional include folders
INCLUDE = -I../../../externals/mongoose/ \
    -I../../core \
    -I../../core/cameras/V4L2

# Libraries to use
LIBS = -ljpeg  -laeroTAP-sdk

# Enable threads in Mongoose
mongoose.o: CFLAGS += -DMG_ENABLE_THREADS
XWebServer.o: CFLAGS += -DMG_ENABLE_THREADS

ifneq "$(findstring debug, $(MAKECMDGOALS))" ""
# "Debug" build - no optimization and add debugging symbols 
OUT_FOLDER = ../../../build/gcc/debug/
CFLAGS += -O0 -g
else 
# "Release" build - optimization and no debug symbols 
OUT_FOLDER = ../../../build/gcc/release/
CFLAGS += -O2 -s -DNDEBUG
INCLUDE += -I$(OUT_FOLDER)include
endif

# Update compiler/linker flags include folders and libraries
CFLAGS += $(INCLUDE) 
LDFLAGS = $(LIBS) -pthread

# Different output folders
OUT_BIN = $(OUT_FOLDER)bin/
OUT_INC = $(OUT_FOLDER)include/
OUT_WEB = $(OUT_BIN)web/

# web2h tool used to generate header files for web files
WEB2H = $(OUT_BIN)web2h

# ===================================

all: release

release: generateweb build

debug: build copyweb
 
%.o: %.c
	$(COMPILER) $(CFLAGS) -c $^ -o $@
%.o: %.cpp
	$(COMPILER) $(CFLAGS) -c $^ -o $@

$(OUT): $(OBJ)
	$(COMPILER) -o $@ $(OBJ) $(LDFLAGS) -L/home/pi/cam2web-aeroTAP/src/core/cameras/aeroCAM/lib/linux_$(HOST_PLATFORM)


build: $(OUT)
	mkdir -p $(OUT_BIN)
	cp $(OUT) $(OUT_BIN)

clean:
	rm $(OBJ) $(OUT)
	rm -rf web

copyweb:
	mkdir -p $(OUT_WEB)
	cp ../../web/* $(OUT_WEB)
	cp ../../../externals/jquery/*.js $(OUT_WEB)
	cp ../../../externals/jquery/*.css $(OUT_WEB)
	cp -r $(OUT_WEB) .

generateweb:
	mkdir -p $(OUT_INC)
	cp ../../web/* $(OUT_INC)
	cp ../../../externals/jquery/*.js $(OUT_INC)
	cp ../../../externals/jquery/*.css $(OUT_INC)
	$(WEB2H) -i $(OUT_INC)index.html -o $(OUT_INC)index.html.h
	$(WEB2H) -i $(OUT_INC)styles.css -o $(OUT_INC)styles.css.h
	$(WEB2H) -i $(OUT_INC)cam2web.png -o $(OUT_INC)cam2web.png.h
	$(WEB2H) -i $(OUT_INC)cam2web_white.png -o $(OUT_INC)cam2web_white.png.h
	$(WEB2H) -i $(OUT_INC)cameraproperties.html -o $(OUT_INC)cameraproperties.html.h    
	$(WEB2H) -i $(OUT_INC)camera.js -o $(OUT_INC)camera.js.h
	$(WEB2H) -i $(OUT_INC)cameraproperties.js -o $(OUT_INC)cameraproperties.js.h
	$(WEB2H) -i $(OUT_INC)jquery.js -o $(OUT_INC)jquery.js.h
	$(WEB2H) -i $(OUT_INC)jquery.mobile.js -o $(OUT_INC)jquery.mobile.js.h
	$(WEB2H) -i $(OUT_INC)jquery.mobile.css -o $(OUT_INC)jquery.mobile.css.h
	$(WEB2H) -i $(OUT_INC)jquery.mobile.min.map -o $(OUT_INC)jquery.mobile.min.map.h
	rm $(OUT_INC)*.html
	rm $(OUT_INC)*.css
	rm $(OUT_INC)*.js
