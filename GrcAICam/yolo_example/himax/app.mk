SCENARIO_APP_ROOT = .

include $(SCENARIO_APP_ROOT)/drivers.mk

override ALGO_TYPE := $(strip $(ALGO_TYPE))

APPL_DEFINES += -DYOLO
APPL_DEFINES += -DIP_xdma
APPL_DEFINES += -DEVT_DATAPATH
APPL_DEFINES += -DDBG_MORE
APPL_DEFINES += -DTRUSTZONE_SEC_ONLY

################# You should to choose appropriate scenario (1,2 or 3) here #################
#################              (refer to readme.md file for details)        #################
## Scenario 1 - external TFT Display is used,nRF does nothing
## Scenario 2 - external TFT display is used, nRF turns on Himax when the TOF sensor is triggered
## Scenario 3 - a smartfon with Bluetooth connection is used,Himax sends data to nRF
APPL_SCENARIO := 3
#############################################################################################

ifeq ($(APPL_SCENARIO), 1)
  APPL_DEFINES += -DSPI_DISPLAY
else ifeq ($(APPL_SCENARIO), 2)
  APPL_DEFINES += -DSPI_DISPLAY
else ifeq ($(APPL_SCENARIO), 3)
  APPL_DEFINES += -DSPI_NRF
else
  $(error Scvenario can be 1,2 or 3 only)
endif

LIB_SEL = pwrmgmt sensordp tflmtag2209_u55tag2205 spi_ptl spi_eeprom hxevent img_proc

override undefine OS_SEL
override TRUSTZONE := y
override TRUSTZONE_TYPE := security
override TRUSTZONE_FW_TYPE := 1
override CIS_SEL := HM_COMMON
override EPII_USECASE_SEL := drv_user_defined

SCENARIO_APP_REQUIRED = 

SCENARIO_APP_INCDIR = $(SCENARIO_APP_ROOT)

SCENARIO_APP_CSRCDIR = 
SCENARIO_APP_CXXSRCDIR = 
SCENARIO_APP_CCSRCDIR = 
SCENARIO_APP_ASMSRCDIR = 

SCENARIO_APP_CSRCS = 
SCENARIO_APP_CXXSRCS =
SCENARIO_APP_CCSRCS =
SCENARIO_APP_ASMSRCS =
SCENARIO_APP_ALLSRCS = 

SCENARIO_APP_COBJS = 
SCENARIO_APP_CXXOBJS =
SCENARIO_APP_CCOBJS =  
SCENARIO_APP_ASMOBJS = 
SCENARIO_APP_ALLOBJS =

SCENARIO_APP_DEFINES =
SCENARIO_APP_DEPS =
SCENARIO_APP_LIBS =

LIB_SCENARIO_APP_DEFINES = 

LIB_SCENARIO_APP= $(OUT_DIR)/libscenario_app.a

# application
SCENARIO_APP_INCDIR = $(SCENARIO_APP_ROOT)/src
SCENARIO_APP_CSRCDIR = $(SCENARIO_APP_ROOT)/src
SCENARIO_APP_CXXSRCDIR = $(SCENARIO_APP_ROOT)/src
# sensor
SCENARIO_APP_INCDIR += $(SCENARIO_APP_ROOT)/src/cis_sensor/cis_ov5647
SCENARIO_APP_CSRCDIR += $(SCENARIO_APP_ROOT)/src/cis_sensor/cis_ov5647
SCENARIO_APP_CXXSRCDIR += $(SCENARIO_APP_ROOT)/src/cis_sensor/cis_ov5647
# display
SCENARIO_APP_CSRCDIR += $(SCENARIO_APP_ROOT)/src/display
SCENARIO_APP_CXXSRCDIR += $(SCENARIO_APP_ROOT)/src/display
# event handler
SCENARIO_APP_INCDIR += $(SCENARIO_APP_ROOT)/src/evt_datapath
SCENARIO_APP_CSRCDIR += $(SCENARIO_APP_ROOT)/src/evt_datapath
SCENARIO_APP_CXXSRCDIR += $(SCENARIO_APP_ROOT)/src/evt_datapath

SCENARIO_APP_CSRCS = $(call get_csrcs, $(SCENARIO_APP_CSRCDIR))
SCENARIO_APP_CXXSRCS = $(call get_cxxsrcs, $(SCENARIO_APP_CXXSRCDIR))
SCENARIO_APP_CCSRCS = $(call get_ccsrcs, $(SCENARIO_APP_CCSRCDIR))
SCENARIO_APP_ASMSRCS = $(call get_asmsrcs, $(SCENARIO_APP_ASMSRCDIR))

override LINKER_SCRIPT_FILE := $(SCENARIO_APP_ROOT)/src/yolo.ld


