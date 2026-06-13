THIS_MAKEFILE := $(abspath $(lastword $(MAKEFILE_LIST)))
THIS_DIR := $(dir $(THIS_MAKEFILE))

BUSYBOX_VERSION := 1.36.1
BUSYBOX_DIR := busybox-$(BUSYBOX_VERSION)
BUSYBOX_FILENAME := $(BUSYBOX_DIR).tar.bz2
BUSYBOX_URL := https://busybox.net/downloads/$(BUSYBOX_FILENAME)

BOARD ?= n31
CONF := apple_$(BOARD)_defconfig
OUT_FILENAME := initramfs-$(BOARD).cpio
ROOTFS_DIR := rootfs

BB_FILE := $(THIS_DIR)/$(BUSYBOX_FILENAME)
BB := $(THIS_DIR)/$(BUSYBOX_DIR)
FS := $(THIS_DIR)/$(ROOTFS_DIR)
OUT := $(THIS_DIR)/$(OUT_FILENAME)
OUT_Z := $(OUT).gz

all: rootfs-compress

bb-download: $(BB_FILE)
bb-extract: $(BB)/.extracted
bb-config: $(BB)/.config
bb-build: $(BB)/busybox
bb-install: $(FS)/bin/busybox
rootfs-install: $(FS)/bin/busybox $(FS)/etc/inittab $(FS)/etc/init.d/rcS $(FS)/init
rootfs-pack: $(OUT)
rootfs-compress: $(OUT_Z)

$(BB_FILE):
	curl -L -o $@ $(BUSYBOX_URL)

$(BB)/.extracted: $(BB_FILE)
	tar -xjvf $<
	touch $@

$(BB)/.config: $(CONF) $(BB)/.extracted
	cp $< $@
	yes '' | $(MAKE) -C $(BB) oldconfig

$(BB)/busybox: $(BB)/.config
	$(MAKE) -C $(BB)

$(FS)/bin/busybox: $(BB)/busybox
	$(MAKE) -C $(BB) CONFIG_PREFIX=$(FS) install

$(FS)/etc/inittab: $(THIS_DIR)/inittab
	mkdir -p $(dir $@)
	cp $< $@

$(FS)/etc/init.d/rcS: $(THIS_DIR)/rcS
	mkdir -p $(dir $@)
	cp $< $@
	chmod +x $@

$(FS)/init: $(THIS_DIR)/init
	cp $< $@
	chmod +x $@

$(OUT): rootfs-install
	find $(FS) -print0 | sort -z | cpio --quiet --null --format=newc --owner 0:0 --create > $@

$(OUT_Z): $(OUT)
	gzip -9 < $< > $@

clean:
	rm -rf $(FS) $(BB)

distclean:
	rm -rf $(FS) $(BB) $(BB_FILE)

.PHONY: all bb-extract bb-config bb-build bb-install rootfs-install rootfs-pack rootfs-compress clean distclean
