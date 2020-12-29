cmake_minimum_required(VERSION 3.5.1)
project(PCLDemo)
set(CMAKE_CXX_STANDARD 14)

# 设置输出根目录为build/Debug
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)

find_package(PCL REQUIRED)
# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/inc)
include_directories(${PROJECT_SOURCE_DIR}/src)
# 设置依赖库链接目录
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS} -g -rdynamic)

add_executable(main main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})

#add_executable(demo demo.cpp)
#target_link_libraries(demo ${PCL_LIBRARIES})

#add_executable(normal_distributions_transform normal_distributions_transform.cpp)
#target_link_libraries(normal_distributions_transform ${PCL_LIBRARIES})




add_executable (test1 test1.cpp ${PROJECT_SOURCE_DIR}/src/_freecplus.cpp)
target_link_libraries (test1 ${PCL_LIBRARIES})


add_executable (testall testall.cpp ${PROJECT_SOURCE_DIR}/src/_freecplus.cpp)
target_link_libraries (testall ${PCL_LIBRARIES})
add_executable (voxel_grid voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})
