
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libh264
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := H.264 bitstream reader/writer library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DH264_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/h264.c \
	src/h264_bac.c \
	src/h264_bitstream.c \
	src/h264_cabac.c \
	src/h264_cabac_ctx_tables.c \
	src/h264_ctx.c \
	src/h264_dump.c \
	src/h264_fmo.c \
	src/h264_macroblock.c \
	src/h264_reader.c \
	src/h264_slice_data.c \
	src/h264_types.c \
	src/h264_writer.c
LOCAL_LIBRARIES := \
	json \
	libulog
include $(BUILD_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := h264-dump
LOCAL_DESCRIPTION := H.264 bitstream dump tool
LOCAL_CATEGORY_PATH := libs/h264
LOCAL_CFLAGS := -std=gnu99
LOCAL_SRC_FILES := \
	tools/h264_dump.c
LOCAL_LIBRARIES := \
	json \
	libh264 \
	libulog
include $(BUILD_EXECUTABLE)
