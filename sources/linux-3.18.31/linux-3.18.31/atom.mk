# Linux, Qualcomm apq8009/msm89090 cpu
ifeq ("$(TARGET_OS)","linux")
ifeq ("$(TARGET_CPU)","apq8009")

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := linux
LOCAL_CATEGORY_PATH := system

LINUX_DEFAULT_CONFIG_TARGET := msm8909_defconfig

include $(BUILD_LINUX)

endif
endif
