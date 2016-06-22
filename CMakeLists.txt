
#Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX) 
add_definitions(-std=c++11)
endif()
cmake_minimum_required(VERSION 2.8)
project( ratonekf	 )
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable( ejemplotiempo ejemplotiempo.cpp )
#target_link_libraries( ejemplotiempo ${OpenCV_LIBS} )
#
#
#add_executable( raton raton.cpp )
#target_link_libraries( raton ${OpenCV_LIBS} )


add_executable( ratonekf ratonekf.cpp )
target_link_libraries( ratonekf ${OpenCV_LIBS} )

