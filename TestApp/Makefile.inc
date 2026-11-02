mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

SRCLANG = c++

# === Common modules (要レビュー) =========================
APPL_CXXOBJS += \
	LineTracer.o \
	Straight.o \
	ColorHSV.o \
	ColorLAB.o \
	ColorRGB.o \
	Sonic.o \
	Gyro.o \
	Arm.o

APPL_DIRS += \
	$(mkfile_path)Common/LineTracer \
	$(mkfile_path)Common/Straight \
	$(mkfile_path)Common/Color/ColorHSV \
	$(mkfile_path)Common/Color/ColorLAB \
	$(mkfile_path)Common/Color/ColorRGB \
	$(mkfile_path)Common/Sonic \
	$(mkfile_path)Common/Gyro \
	$(mkfile_path)Common/Arm

INCLUDES += \
	-I$(mkfile_path)Common/LineTracer \
	-I$(mkfile_path)Common/Straight \
	-I$(mkfile_path)Common/Color \
	-I$(mkfile_path)Common/Color/ColorHSV \
	-I$(mkfile_path)Common/Color/ColorLAB \
	-I$(mkfile_path)Common/Color/ColorRGB \
	-I$(mkfile_path)Common/Sonic \
	-I$(mkfile_path)Common/Gyro \
	-I$(mkfile_path)Common/Arm

APPL_LIBS += -lm

# === Scene: HCR ===========================================
APPL_CXXOBJS += HCR.o
APPL_DIRS    += $(mkfile_path)Scene/HCR
INCLUDES     += -I$(mkfile_path)Scene/HCR

# === Scene: Member A =====================================
APPL_CXXOBJS += Nansho1.o
APPL_DIRS    += $(mkfile_path)Scene/Nansho1
INCLUDES     += -I$(mkfile_path)Scene/Nansho1

# === Scene: Member B =====================================
APPL_CXXOBJS += Nansho2.o
APPL_DIRS    += $(mkfile_path)Scene/Nansho2
INCLUDES     += -I$(mkfile_path)Scene/Nansho2

# === Scene: Member C =====================================
APPL_CXXOBJS += Nansho3.o
APPL_DIRS    += $(mkfile_path)Scene/Nansho3
INCLUDES     += -I$(mkfile_path)Scene/Nansho3

# === Scene: Test (首振り走査のみ) ========================
APPL_CXXOBJS += Test.o
APPL_DIRS    += $(mkfile_path)Scene/Test
INCLUDES     += -I$(mkfile_path)Scene/Test

# === Scene: FixedRun =====================================
APPL_CXXOBJS += FixedRunComplete.o FixedRunShort.o
APPL_DIRS    += $(mkfile_path)Scene/FixedRun
INCLUDES     += -I$(mkfile_path)Scene/FixedRun

# === Continuous curve / Pure Pursuit scene ===
APPL_CXXOBJS += Nansho3KYOKU.o Path.o
APPL_DIRS += $(mkfile_path)Scene/Nansho3KYOKU
INCLUDES += -I$(mkfile_path)Scene/Nansho3KYOKU

