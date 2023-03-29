cmake_minimum_required (VERSION 3.1.0)
project (test)

#find_package(OpenMP) # jesli akaze jako bibliotekla statyczna i zostala skompilowana z openmp. Jesli wyrzucilo sie z cmakelists linijki powiazane z openmp to tu jest to niepotrzebne. W oryginalnym AKAZE wykorzystwane są wszystkie watki do obliczeń w różnych miejscach algorytmu a ponieważ ja korzystam z wszystkich wątków do optymalizacji, to mi to niestety trochę może przeszkadzać. Chyba, że na koncu obliczeń bede liczyc dla 2-3 czastek i pozostale watki beda wolne to ewentualnie.
#if (OPENMP_FOUND)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif()

include_directories("akaze-master-autor/src/lib/")

# AKAZE libraries
set(AKAZE_SRCS
    akaze-master-autor/src/lib/AKAZEConfig.h
    akaze-master-autor/src/lib/AKAZE.h                  akaze-master-autor/src/lib/AKAZE.cpp
    akaze-master-autor/src/lib/fed.h                    akaze-master-autor/src/lib/fed.cpp
    akaze-master-autor/src/lib/nldiffusion_functions.h  akaze-master-autor/src/lib/nldiffusion_functions.cpp
    akaze-master-autor/src/lib/utils.h                  akaze-master-autor/src/lib/utils.cpp)

add_library(AKAZE ${AKAZE_SRCS})
target_link_libraries(AKAZE ${OpenCV_LIBS})

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(CXX11)
check_for_cxx11_compiler(CXX11_COMPILER)
# If a C++11 compiler is available, then set the appropriate flags
IF(CXX11_COMPILER)
  enable_cxx11()
ENDIF(CXX11_COMPILER)

#include_directories(/home/alek/liby)

find_package (Threads REQUIRED)

SET(OpenCV_DIR "~/opencv3/opencv3.3.1/build")
find_package (OpenCV REQUIRED)
#find_package (libAKAZE REQUIRED)

  add_definitions (-Wall -march=native -O3)

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/modules/")
  include_directories(${ADDITIONAL_INCLUDE_DIRS} /usr/local/include/eigen3 )
  set(ADDITIONAL_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR}/unsupported )

include_directories(${ADDITIONAL_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
include_directories ("${PROJECT_SOURCE_DIR}")

   add_executable (main_rpe main_rpe.cpp rpe/rpe.cpp)
   add_executable (main_rpe_5param main_rpe_5param.cpp rpe/rpe.cpp)
   add_executable (main_ate main_ate.cpp)
   add_executable (main_ate_5param main_ate_5param.cpp)
   target_link_libraries (main_ate ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} AKAZE)
   target_link_libraries (main_ate_5param ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} AKAZE)
   target_link_libraries (main_rpe ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} AKAZE)
   target_link_libraries (main_rpe_5param ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} AKAZE)
