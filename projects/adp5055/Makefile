# Select the example you want to enable by choosing y for enabling and n for disabling
BASIC_EXAMPLE = n
IIO_EXAMPLE = y

# File naming based on enabled measurement
ifeq (y,$(strip $(BASIC_EXAMPLE)))
MEAS = basic
endif

ifeq (y,$(strip $(IIO_EXAMPLE)))
MEAS = iio
endif

ifeq (maxim,$(strip $(PLATFORM)))
BINARY	?=  $(BUILD_DIR)/adp5055-$(TARGET)-$(MEAS).elf
endif

ifeq (mbed,$(strip $(PLATFORM)))
BINARY	?=  $(BUILD_DIR)/adp5055-$(PLATFORM)-$(MEAS).elf
endif

include ../../tools/scripts/generic_variables.mk

include src.mk

include ../../tools/scripts/generic.mk
