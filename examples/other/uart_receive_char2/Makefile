# User Test
#------------------------------------
PMSIS_OS         = freertos

APP              = uart_receive_char
APP_SRCS        += uart_receive_char.c

# Define the location of our generated MAVLink headers
# MAVLINK_DIALECT_DIR = ../../../lib/mavlink/generated/ardupilotmega
# Add the MAVLink dialect headers to the compiler's include path
# APP_INC += $(MAVLINK_DIALECT_DIR)

APP_CFLAGS      +=

ifeq ($(ASYNC), 1)
APP_CFLAGS      += -DASYNC=1
endif

include $(RULES_DIR)/pmsis_rules.mk
