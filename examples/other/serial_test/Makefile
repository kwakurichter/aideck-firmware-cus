# Directs printf output to the UART for debugging
io=uart

# Selects the FreeRTOS operating system, which is needed for complex applications
PMSIS_OS = freertos

# Set the name of our application's output files
APP = serial_test

# Tell the build system which C source files to compile.
APP_SRCS += serial_test.c

# Add any necessary compiler flags.
# -O3 is for optimization, -g adds debugging information.
APP_CFLAGS += -O3 -g

# This line includes all the core build rules from the GAP8 SDK.
# It must be the last line in the file.
include $(RULES_DIR)/pmsis_rules.mk