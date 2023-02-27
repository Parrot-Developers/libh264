cmake_minimum_required(VERSION 3.15)
project(libh264 VERSION 1.0)

set(LIB_SOURCES
      src/h264.c
      src/h264_bac.c
      src/h264_bitstream.c
      src/h264_cabac.c
      src/h264_cabac_ctx_tables.c
      src/h264_ctx.c
      src/h264_dump.c
      src/h264_fmo.c
      src/h264_macroblock.c
      src/h264_reader.c
      src/h264_slice_data.c
      src/h264_types.c
      src/h264_writer.c
)

add_library(h264 ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
set_target_properties(h264 PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(h264 PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(h264 PRIVATE "H264_API_EXPORTS")
target_compile_definitions(h264 PRIVATE "_GNU_SOURCE")
target_compile_options(h264 PRIVATE "-fvisibility=hidden")
target_compile_options(h264 PRIVATE "-std=gnu99")


if(WIN32)
  target_link_libraries(h264 PRIVATE ws2_32)
endif()

find_package(json-c CONFIG REQUIRED)
target_link_libraries(h264 PRIVATE json-c::json-c)
target_link_libraries(h264 PRIVATE ulog)