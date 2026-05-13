mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

APPL_CXXOBJS +=\
LineTracer.o\
Straight.o\

APPL_LIBS += -lm

APPL_DIRS += $(mkfile_path)LineTracer
APPL_DIRS += $(mkfile_path)Straight

INCLUDES += \
	-I$(mkfile_path)LineTracer\
	-I$(mkfile_path)Straight\
