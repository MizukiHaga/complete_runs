mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

# === Common modules (要レビュー) =========================
APPL_CXXOBJS += \
	LineTracer.o \
	Straight.o \
	ColorHSV.o \
	ColorRGB.o \
	Sonic.o \
	Gyro.o

APPL_DIRS += \
	$(mkfile_path)Common/LineTracer \
	$(mkfile_path)Common/Straight \
	$(mkfile_path)Common/Color/ColorHSV \
	$(mkfile_path)Common/Color/ColorRGB \
	$(mkfile_path)Common/Sonic \
	$(mkfile_path)Common/Gyro

INCLUDES += \
	-I$(mkfile_path)Common/LineTracer \
	-I$(mkfile_path)Common/Straight \
	-I$(mkfile_path)Common/Color/ColorHSV \
	-I$(mkfile_path)Common/Color/ColorRGB \
	-I$(mkfile_path)Common/Sonic \
	-I$(mkfile_path)Common/Gyro

APPL_LIBS += -lm

# === Scene: Member A =====================================
APPL_CXXOBJS += Nansho1.o
APPL_DIRS    += $(mkfile_path)Scene/Nansho1
INCLUDES     += -I$(mkfile_path)Scene/Nansho1

# === Scene: Member B =====================================
# (ここにメンバー B が追記)
