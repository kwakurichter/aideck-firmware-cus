# 1. Set the IO to uart for printf redirect
io=uart
PMSIS_OS = freertos

# 2. Update the application name
APP = mavlink_uart3

# 3. List your source files. We don't need CPX, so this is simpler.
APP_SRCS += mavlink_uart.c

# 4. Compiler flags - we can remove the CPX-specific ones
APP_CFLAGS += -O3 -g

# These flags are required by FreeRTOS for certain timing features used by CPX
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

# 5. Include the main build rules
include $(RULES_DIR)/pmsis_rules.mk