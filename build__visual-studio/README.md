author: Alexandra Ion
15.2.2018

-----------------------------------
HOW TO BUILD FOR VISUAL STUDIO 2015
-----------------------------------

Here is some info about what settings were included as to build for windows. No changes should be necessary as everything is stored in the .vcxproj file.

(1) You need to run *Release | x64* configuration!
The ipopt .dlls are compiled for this configuration only.

(2) ipopt binaries
All dependencies are included in the ./dependencies. The .dlls are automatically copied to the bin directory next to the .exe via pre-build scripts in .vcxproj.
Currently, we are using the .dlls from https://www.coin-or.org/download/binary/Ipopt/Ipopt-3.11.0-Win32-Win64-dll.7z.

(3) -D_SCL_SECURE_NO_WARNINGS 
Is added to the C++ command line settings in .vcxproj (I forgot the reason though :)

(4) somehow aligned doesn't work, so needed to switch it off in MetaGrid.hpp
#ifdef _WIN32
	#define EIGEN_DONT_ALIGN_STATICALLY
#endif
