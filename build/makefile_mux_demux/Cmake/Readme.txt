
###  Building the MPEG-H mp4 muxer and demuxer using Cmake
Users can also use cmake to build for `x86`, `x86_64`, `armv7`, `armv8` and Windows (MSVS project) platforms.
##### To build for native platform, run the following commands: 
```
$ mkdir cmake_build
$ cd cmake_build
$ cmake ..
$ make
```

##### Cross-compiling 
Ensure to edit the file toolchain_<arch>.cmake to set proper paths in host for corresponding platforms:

# Example: ARMv7
# Specify the cross compiler path in toolchain_armv7.cmake
SET(CMAKE_C_COMPILER   /<Path>/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /<Path>/arm-linux-gnueabihf-g++)

# Specify target environment path in toolchain_armv7.cmake
SET(CMAKE_FIND_ROOT_PATH  <Path>/arm-linux-gnueabihf)

Run the following commands to cross compile for `x86`, `ARMv7` or `ARMv8`:
```
$ mkdir cmake_build
$ cd cmake_build
$ cmake .. -DCMAKE_TOOLCHAIN_FILE=../toolchain_<arch>.cmake
$ make
```

##### Building on Windows
To create MSVS project files for the MPEG-H 3D Audio Low Complexity Profile encoder from cmake, run the following commands:
```
$ mkdir cmake_build
$ cd cmake_build
$ cmake -G "Visual Studio 15 2017" ..
```
Above command will create Win32 version of workspace 
To create MSVS project files for Win64 version from cmake, run the following commands:
```
$ mkdir cmake_build
$ cd cmake_build
$ cmake -G "Visual Studio 15 2017 Win64" ..
```
The above command creates MSVS 2017 project files. If the version is different, modify the generator name accordingly.

Note: This Cmake creates debug version for msvs and release version for other platforms.
      For msvs ,it can be manually converted to release mode in the solution configuration.
	  For other platforms adding "-g" for the Cflag in the Cmakelists will set the debug version.