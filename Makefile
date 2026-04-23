# Beehive Monitor - Makefile
# Compatible with current Myriota SDK layout (module/app.mk)

PROGRAM_NAME = beehive_monitor
APP_SRC = beehive_monitor.c

empty :=
space := $(empty) $(empty)
ROOTDIR_RAW ?= $(MYRIOTA_SDK_ROOT)
ifeq ($(strip $(ROOTDIR_RAW)),)
ROOTDIR_RAW := SDK-master
endif
ROOTDIR := $(subst $(space),\ ,$(ROOTDIR_RAW))

include $(ROOTDIR)/module/app.mk
