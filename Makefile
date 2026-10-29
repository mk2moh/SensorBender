export THEOS_PACKAGE_SCHEME = roothide
TARGET := iphone:clang:14.5:15.0
ARCHS = arm64 arm64e
include $(THEOS)/makefiles/common.mk

TWEAK_NAME = SensorBender
SensorBender_FILES = Tweak.x
SensorBender_FRAMEWORKS = UIKit CoreMotion CoreLocation Foundation
SensorBender_LOGOS_DEFAULT_GENERATOR = internal
SensorBender_CFLAGS = -fobjc-arc -Wno-deprecated-declarations -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function
SensorBender_INSTALL = 0

include $(THEOS_MAKE_PATH)/tweak.mk
include $(THEOS_MAKE_PATH)/aggregate.mk
