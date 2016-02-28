#
# Copyright (C) 2008-2012 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=i2c-ch341
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/i2c-ch341
  SUBMENU:=USB Support
  TITLE:=CH341 USB-to-I2C SPI UART converter
  FILES:=$(PKG_BUILD_DIR)/i2c-ch341.ko
  DEPENDS:=+kmod-i2c-core +kmod-usb-core
  AUTOLOAD:=$(call AutoLoad,30,i2c-ch341,1)
  KCONFIG:=
endef

define KernelPackage/i2c-ch341/description
	Drives ch341 based usb dongles. 
endef

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)"

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,i2c-ch341))
