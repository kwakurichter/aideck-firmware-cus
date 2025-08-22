# User Test
#------------------------------------
PMSIS_OS         = freertos

APP              = mavlink_uart
APP_SRCS        += mavlink_heartbeat.c

# Define the location of our generated MAVLink headers
MAVLINK_DIALECT_DIR = ../../../lib/mavlink/generated/ardupilotmega
# Add the MAVLink dialect headers to the compiler's include path
APP_INC += $(MAVLINK_DIALECT_DIR)

APP_CFLAGS      +=

APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

ifeq ($(ASYNC), 1)
APP_CFLAGS      += -DASYNC=1
endif

include $(RULES_DIR)/pmsis_rules.mk
