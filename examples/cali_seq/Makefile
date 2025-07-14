COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= cali_seq.c

BHI385_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

API_LOCATION ?= ../..

FW_LOCATION ?= ../..

COMMON_LOCATION ?= ..

C_SRCS += \
$(COMMON_LOCATION)/common/common.c \
$(COMMON_LOCATION)/common/verbose.c \
$(API_LOCATION)/bhi385.c \
$(API_LOCATION)/bhi385_hif.c \
$(API_LOCATION)/bhi385_logbin.c \
$(API_LOCATION)/bhi385_parse.c \
$(API_LOCATION)/bhi385_event_data.c \
$(API_LOCATION)/bhi385_virtual_sensor_conf_param.c \
$(API_LOCATION)/bhi385_phy_sensor_ctrl_param.c \
$(API_LOCATION)/bhi385_virtual_sensor_info_param.c \
$(API_LOCATION)/bhi385_system_param.c

INCLUDEPATHS += . \
$(COMMON_LOCATION)/common \
$(API_LOCATION) \
$(FW_LOCATION)/firmware 

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHI385_INTF), I2C)
CFLAGS += -DBHI385_USE_I2C
endif

ifeq ($(TARGET),$(filter $(TARGET),MCU_APP30 MCU_APP31))
LOCATION := FLASH
endif

include $(COINES_INSTALL_PATH)/coines.mk