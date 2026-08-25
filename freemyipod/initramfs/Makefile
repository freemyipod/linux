ifndef BOARD
    $(error BOARD is not set. Please run: make target BOARD=value. Possible values: n20, n31)
endif

BUSYBOX_VERSION := 1.36.1
BUSYBOX_DIR := busybox-$(BUSYBOX_VERSION)
BUSYBOX_FILENAME := $(BUSYBOX_DIR).tar.bz2
BUSYBOX_URL := https://busybox.net/downloads/$(BUSYBOX_FILENAME)

APPS := fbclock aes-test rng-test sha1-test
APP_CROSS_COMPILE := arm-linux-gnueabihf-
APP_CFLAGS := -mcpu=cortex-a5+nosimd -static -O2 -Wall

THIS_MAKEFILE := $(abspath $(lastword $(MAKEFILE_LIST)))
THIS_DIR := $(dir $(THIS_MAKEFILE))

CONF := apple_$(BOARD)_defconfig
BB_FILE := $(THIS_DIR)/$(BUSYBOX_FILENAME)
BB := $(THIS_DIR)/$(BUSYBOX_DIR)
BB_EXTRACTED := $(BB)/.extracted
BB_CONFIG := $(BB)/.config
BB_BINARY := $(BB)/busybox

ROOTFS_DIR := rootfs
ROOTFS_TEMPLATE_DIR := rootfs-template
ROOTFS_STAMP_FILENAME := rootfs.stamp
ROOTFS := $(THIS_DIR)/$(ROOTFS_DIR)
ROOTFS_TEMPLATE := $(THIS_DIR)/$(ROOTFS_TEMPLATE_DIR)
ROOTFS_STAMP := $(THIS_DIR)/$(ROOTFS_STAMP_FILENAME)
ROOTFS_BB_BINARY := $(ROOTFS)/bin/busybox

OUT_FILENAME := initramfs-$(BOARD).cpio
OUT := $(THIS_DIR)/$(OUT_FILENAME)
OUT_Z := $(OUT).gz

ROOTFS_TEMPLATE_FILES := $(shell find $(ROOTFS_TEMPLATE) -type f)
ROOTFS_TEMPLATE_DEPS := $(patsubst $(ROOTFS_TEMPLATE)/%,$(ROOTFS)/%,$(ROOTFS_TEMPLATE_FILES))
ROOTFS_APP_DEPS := $(patsubst %,$(ROOTFS)/usr/bin/%,$(APPS))
ROOTFS_DEPS := $(ROOTFS_BB_BINARY) $(ROOTFS_TEMPLATE_DEPS) $(ROOTFS_APP_DEPS)

all: rootfs-compress

bb-download: $(BB_FILE)
bb-extract: $(BB_EXTRACTED)
bb-config: $(BB_CONFIG)
bb-build: $(BB_BINARY)
bb-install: $(ROOTFS_BB_BINARY)

apps-install: $(ROOTFS_APP_DEPS)

rootfs-install: $(ROOTFS_STAMP)
rootfs-pack: $(OUT)
rootfs-compress: $(OUT_Z)

$(BB_FILE):
	curl -L -o $@ $(BUSYBOX_URL)

$(BB_EXTRACTED): $(BB_FILE)
	tar -xjvf $<
	touch $@

$(BB_CONFIG): $(CONF) $(BB_EXTRACTED)
	cp $< $@
	yes '' | $(MAKE) -C $(BB) oldconfig

$(BB_BINARY): $(BB_CONFIG)
	$(MAKE) -C $(BB)

$(ROOTFS_BB_BINARY): $(BB_BINARY)
	$(MAKE) -C $(BB) CONFIG_PREFIX=$(ROOTFS) install

$(ROOTFS)/%: $(ROOTFS_TEMPLATE)/%
	mkdir -p $(dir $@)
	cp -p $< $@

$(ROOTFS)/usr/bin/%: ../%.c
	mkdir -p $(dir $@)
	$(APP_CROSS_COMPILE)gcc $(APP_CFLAGS) -o $@ $<
	$(APP_CROSS_COMPILE)strip $@

$(ROOTFS_STAMP): $(ROOTFS_DEPS)
	touch $@

$(OUT): $(ROOTFS_STAMP)
	cd $(ROOTFS) && find . -print0 | sort -z | cpio --quiet --null --format=newc --owner 0:0 --create > $@

$(OUT_Z): $(OUT)
	gzip -9 < $< > $@

clean:
	rm -rf $(ROOTFS) $(BB) $(ROOTFS_STAMP)

distclean: clean
	rm -f $(BB_FILE)

.PHONY: all bb-download bb-extract bb-config bb-build bb-install apps-install rootfs-install rootfs-pack rootfs-compress clean distclean
