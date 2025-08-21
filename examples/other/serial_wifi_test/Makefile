# Directs printf output to the UART for debugging
io=uart

# Selects the FreeRTOS operating system
PMSIS_OS = freertos

# Enable Access Point setup code
APP_CFLAGS += -DSETUP_WIFI_AP

# Explicitly tell the build runner to use our local config file
RUNNER_CONFIG = $(CURDIR)/config.ini

# Set the name of our application's output files
APP = serial_wifi_test

# Add our main C file, AND the source files for the CPX/COM libraries
APP_SRCS += serial_wifi_test.c ../../../lib/cpx/src/com.c ../../../lib/cpx/src/cpx.c

# Add the include directory for the CPX/COM headers
APP_INC += ../../../lib/cpx/inc

# Add any necessary compiler flags.
APP_CFLAGS += -O3 -g
# These flags are required by FreeRTOS for certain timing features used by CPX
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

# This line includes all the core build rules from the GAP8 SDK.
include $(RULES_DIR)/pmsis_rules.mk