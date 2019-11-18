# CMakeLists files in this project can
# refer to the root source directory of the project as ${Photoneo_SOURCE_DIR} and
# to the root binary directory of the project as ${Photoneo_BINARY_DIR}.
cmake_minimum_required (VERSION 2.8.12)

project (Photoneo)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
    ${Photoneo_SOURCE_DIR}/Photoneo.cpp
    ${Photoneo_SOURCE_DIR}/ReadMe.txt
)

add_executable (Photoneo 
    ${Files}
)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

if (UNIX AND NOT APPLE)
    target_link_libraries(Photoneo
        ${PHOXI_LIBRARY}
        rt
    )
else (UNIX AND NOT APPLE)
    target_link_libraries(Photoneo
        ${PHOXI_LIBRARY}
    )
endif(UNIX AND NOT APPLE)

get_target_property(ReleaseBinFolder Photoneo RUNTIME_OUTPUT_DIRECTORY_RELEASE)
get_target_property(DebugBinFolder Photoneo RUNTIME_OUTPUT_DIRECTORY_DEBUG)

if(MSVC)
    add_custom_command(TARGET Photoneo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:Photoneo>
    )

    add_custom_command(TARGET Photoneo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:Photoneo>
    )
endif(MSVC)

target_include_directories (Photoneo PUBLIC ${PHOXI_INCLUDE_DIRS})

set_target_properties(Photoneo
    PROPERTIES
    #for executables, inheritance of variables CMAKE_DEBUG_POSTFIX... does not work
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)
