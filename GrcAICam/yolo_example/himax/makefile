
# Root dir of EPII_ROOT - Windows
# EPII_ROOT ?= c:\Users\Vladimir.Shalyt\Work\Seeed_Grove_Vision_AI_Module_V2\EPII_CM55M_APP_S
# Root dir of EPII_ROOT - Linux
# EPII_ROOT ?= /tmp/Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S
# EPII_ROOT is sent as an input parameter

ifeq ($(EPII_ROOT),)
$(error You need to set EPII_ROOT variable to path to EPII_CM55M_APP_S folder of the Seeed_Grove_Vision_AI_Module_V2 repository)
endif

override PREBUILT_LIB := $(addprefix $(EPII_ROOT), \prebuilt_libs\gnu\)
override OUT_DIR = build

APPL ?= EPII_CM55M
OLEVEL ?= O2
DEBUG ?= 1
BOARD ?= epii_evb
BD_VER ?= 10
CORTEX_M ?= 55
EPII_CORE ?= BIG
IC_PACKAGE_SEL = WLCSP65
IC_VER ?= 30
CoreVolt ?= 0P9V
TOOLCHAIN ?= gnu
MPU ?= y

TRUSTZONE ?= y
TRUSTZONE_TYPE ?= security
TRUSTZONE_FW_TYPE ?= 1

EPII_USECASE_SEL = drv_onecore_cm55m_s_only
EPII_NSC_SEL = cm55m_nsc_cfg

LIB_CMSIS_NN_ENALBE = 1
LIB_CMSIS_NN_VERSION = 7_0_0
LIB_CV_PREBUILT = 1
LIB_AUDIOALGO_PREBUILT = 0

CMSIS_DRIVERS_USECASE_SEL =
SUPPORT_SEC_LIB ?= y
APP_TYPE = yolo
 
override EPII_ROOT := $(strip $(subst \,/,$(EPII_ROOT)))
override OUT_DIR := $(strip $(subst \,/,$(OUT_DIR)))
include options/options.mk

ifneq "$(HOST_OS)" "Windows"
override PREBUILT_LIB := $(strip $(subst \,/,$(PREBUILT_LIB)))
endif
