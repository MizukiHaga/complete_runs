mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

APPL_CXXOBJS +=\
LineTracer.o\
Straight.o\
Color.o\
Sonic.o\
Nansho1.o\

APPL_LIBS += -lm

APPL_DIRS += $(mkfile_path)LineTracer
APPL_DIRS += $(mkfile_path)Straight
APPL_DIRS += $(mkfile_path)Color
APPL_DIRS += $(mkfile_path)Sonic
APPL_DIRS += $(mkfile_path)Nansho1

INCLUDES += \
	-I$(mkfile_path)LineTracer\
	-I$(mkfile_path)Straight\
	-I$(mkfile_path)Color\
	-I$(mkfile_path)Sonic\
	-I$(mkfile_path)Nansho1\
