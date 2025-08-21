# User Test
#------------------------------------
PMSIS_OS         = freertos

APP              = test
APP_SRCS        += mavlink_heartbeat.c
APP_INC         +=
APP_CFLAGS      +=

ifeq ($(ASYNC), 1)
APP_CFLAGS      += -DASYNC=1
endif

include $(RULES_DIR)/pmsis_rules.mk
