mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

APPL_CXXOBJS +=\
	Angle.o\
	Morter.o\

APPL_LIBS += -lm

APPL_DIRS += $(mkfile_path)Angle
APPL_DIRS += $(mkfile_path)Morter

INCLUDES += \
	-I$(mkfile_path)Angle\
	-I$(mkfile_path)Morter\
