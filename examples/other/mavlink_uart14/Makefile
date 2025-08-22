# User Test
#------------------------------------
PMSIS_OS         = freertos

APP              = mavlink_uart
APP_SRCS        += mavlink_heartbeat.c

# Define the location of our generated MAVLink headers
MAVLINK_DIALECT_DIR = ../../../lib/mavlink/generated/ardupilotmega
# Add the MAVLink dialect headers to the compiler's include path
APP_INC += $(MAVLINK_DIALECT_DIR)

APP_CFLAGS      += -O3 -g

APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

include $(RULES_DIR)/pmsis_rules.mk
