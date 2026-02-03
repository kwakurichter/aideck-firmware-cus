# Directs printf output to the UART for debugging
io=host

# Selects the FreeRTOS operating system
PMSIS_OS = freertos

# Enable Access Point setup code
APP_CFLAGS += -DSETUP_WIFI_AP

# Explicitly tell the build runner to use our local config file
RUNNER_CONFIG = $(CURDIR)/config.ini

# Set the name of our application's output files
APP = mav_wifi_test

# Add our main C file, AND the source files for the CPX/COM libraries
APP_SRCS += mav_wifi_test.c ../../../lib/cpx/src/com.c ../../../lib/cpx/src/cpx.c

# Add the include directory for the CPX/COM headers
APP_INC += ../../../lib/cpx/inc

# Define the location of our generated MAVLink headers
MAVLINK_DIALECT_DIR = ../../../lib/mavlink/generated/ardupilotmega
# Add the MAVLink dialect headers to the compiler's include path
APP_INC += $(MAVLINK_DIALECT_DIR)

# Add any necessary compiler flags.
APP_CFLAGS += -O3 -g
# These flags are required by FreeRTOS for certain timing features used by CPX
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

# This line includes all the core build rules from the GAP8 SDK.
include $(RULES_DIR)/pmsis_rules.mk