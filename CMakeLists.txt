cmake_minimum_required( VERSION 2.8 )

project(handcrafted_descriptor)

set(SHOT_SOURCES SHOT.cpp src/core.cpp)
add_executable(SHOT ${SHOT_SOURCES})
target_include_directories(SHOT PUBLIC ${PROJECT_SOURCE_DIR}/include)

set(SI_SOURCES SI.cpp src/core.cpp)
add_executable(SpinImage ${SI_SOURCES})
target_include_directories(SpinImage PUBLIC ${PROJECT_SOURCE_DIR}/include)




# Find Packages
find_package( PCL 1.8 REQUIRED )

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
  
  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries(SHOT ${PCL_LIBRARIES} )
  target_link_libraries(SpinImage ${PCL_LIBRARIES} )

  # add_subdirectory(pybind11)
  find_package(pybind11 REQUIRED)
  set(SOURCES_PYTHON src/binding.cpp src/core.cpp )
  pybind11_add_module(handcrafted_descriptor ${SOURCES_PYTHON})
  target_include_directories(handcrafted_descriptor PUBLIC ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(handcrafted_descriptor ${PCL_LIBRARIES} )
  
                                  
endif()

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES( SHOT ${Boost_LIBRARIES} )
TARGET_LINK_LIBRARIES( SpinImage ${Boost_LIBRARIES} )

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


