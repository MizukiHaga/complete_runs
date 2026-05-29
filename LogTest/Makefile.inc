mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

APPL_CXXOBJS +=\
	Angle.o\
	Morter.o\
	DistanceSensor.o\
	Battery.o\
	Sonic.o\

APPL_LIBS += -lm

APPL_DIRS += $(mkfile_path)Viewcmd/Angle
APPL_DIRS += $(mkfile_path)Viewcmd/Morter
APPL_DIRS += $(mkfile_path)Viewcmd/DistanceSensor
APPL_DIRS += $(mkfile_path)Viewcmd/Battery
APPL_DIRS += $(mkfile_path)ViewLight/Sonic

INCLUDES += \
	-I$(mkfile_path)Viewcmd\
	-I$(mkfile_path)Viewcmd/Angle\
	-I$(mkfile_path)Viewcmd/Morter\
	-I$(mkfile_path)Viewcmd/DistanceSensor\
	-I$(mkfile_path)Viewcmd/Battery\
	-I$(mkfile_path)ViewLight\
	-I$(mkfile_path)ViewLight/Sonic\
