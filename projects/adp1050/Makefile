# Select the example you want to enable by choosing y for enabling and n for disabling
BASIC_EXAMPLE = n
IIO_EXAMPLE = y

include ../../tools/scripts/generic_variables.mk

# File Naming based on enabled measurement
ifeq (y,$(strip $(BASIC_EXAMPLE)))
MEAS = basic-example
endif

ifeq (y,$(strip $(IIO_EXAMPLE)))
MEAS = iio-example
endif

include src.mk

# File naming based on PLATFORM or TARGET (if applicable)
ifeq (maxim,$(strip $(PLATFORM)))
BINARY	?=  $(BUILD_DIR)/adp1050-$(TARGET)-$(MEAS).elf
endif

ifeq (mbed,$(strip $(PLATFORM)))
BINARY	?=  $(BUILD_DIR)/adp1050-$(PLATFORM)-$(MEAS).elf
endif

include ../../tools/scripts/generic.mk
