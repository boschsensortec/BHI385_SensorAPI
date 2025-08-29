
COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= system_param.c

BHI385_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

API_LOCATION ?= ../..

FW_LOCATION ?= ../..

COMMON_LOCATION ?= ..
LOCATION ?= FLASH

C_SRCS += \
$(COMMON_LOCATION)/common/common.c \
$(COMMON_LOCATION)/common/verbose.c \
$(API_LOCATION)/bhi385.c \
$(API_LOCATION)/bhi385_hif.c \
$(API_LOCATION)/bhi385_system_param.c \
$(API_LOCATION)/bhi385_logbin.c \
$(API_LOCATION)/bhi385_parse.c \
$(API_LOCATION)/bhi385_event_data.c \
$(API_LOCATION)/bhi385_virtual_sensor_info_param.c \
$(API_LOCATION)/bhi385_klio_param.c

INCLUDEPATHS += . \
$(COMMON_LOCATION)/common \
$(API_LOCATION) \
$(FW_LOCATION)/firmware

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHI385_INTF), I2C)
CFLAGS += -DBHI385_USE_I2C
endif

include $(COINES_INSTALL_PATH)/coines.mk