# CMakeList.txt: Physics 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

# 如果支持，请为 MSVC 编译器启用热重载。
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("Physics")

set(CMAKE_CXX_STANDARD 20)


file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(Physics "main.cpp" ${SOURCES})


#添加include路径
target_include_directories(Physics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/include
)


#链接静态库
target_link_directories(Physics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/SDL2
)
target_link_libraries(Physics
    SDL2main.lib
    SDL2.lib
    SDL2_image.lib
)


#复制动态库到exe文件夹
file(COPY
    thirdparty/dll/SDL2.dll
    thirdparty/dll/SDL2_image.dll
    DESTINATION ${CMAKE_BINARY_DIR}
)



if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET Physics PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
