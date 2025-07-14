COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= fw2h.c

COINES_BACKEND ?= COINES_BRIDGE

# Override other target definitions
TARGET = PC

ifndef TARGET
    $(error TARGET is not defined; please specify a target)
endif

INVALID_TARGET = MCU
INVALID_STR := $(findstring $(INVALID_TARGET), $(TARGET))
ifeq ($(INVALID_STR),$(INVALID_TARGET))
    $(error TARGET has an invalid value '$(TARGET)'; 'MCU' targets are not a valid. Please use PC as target.)
endif

include $(COINES_INSTALL_PATH)/coines.mk
