#!/bin/bash
# Build a minimal ARMv5te initramfs for the iPod nano 3g (S5L8702 / ARM926EJ-S).
# Requires: gcc-arm-linux-gnueabi, make, cpio, gzip
# Output:   freemyipod/initramfs-n3g.cpio.gz  (relative to kernel tree root)

set -eEx

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KERNEL_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUSYBOX_SRC="$(cd "$KERNEL_ROOT/../busybox-1.36.1" && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

CROSS=arm-linux-gnueabi-

if ! command -v "${CROSS}gcc" &>/dev/null; then
    echo "ERROR: ${CROSS}gcc not found. Install with: sudo apt-get install gcc-arm-linux-gnueabi"
    exit 1
fi

# ARMv5TE matches ARM926EJ-S; soft-float (no VFP on this SoC)
ARCH_FLAGS="-march=armv5te -mtune=arm926ej-s -msoft-float"

echo "=== Configuring busybox for ARMv5TE ==="
BB_BUILD="$WORK/busybox-build"
cp -a "$BUSYBOX_SRC" "$BB_BUILD"

# Generate a complete base config, then patch the settings we need
make -C "$BB_BUILD" ARCH=arm CROSS_COMPILE="$CROSS" defconfig

set_opt() {
    local key="$1" val="$2" cfg="$BB_BUILD/.config"
    if grep -q "^${key}=" "$cfg" || grep -q "^# ${key} is not set" "$cfg"; then
        sed -i "s|^${key}=.*|${key}=${val}|; s|^# ${key} is not set|${key}=${val}|" "$cfg"
    else
        echo "${key}=${val}" >> "$cfg"
    fi
}

set_opt CONFIG_CROSS_COMPILER_PREFIX '"arm-linux-gnueabi-"'
set_opt CONFIG_EXTRA_CFLAGS '"-march=armv5te -mtune=arm926ej-s -msoft-float"'
set_opt CONFIG_STATIC y
set_opt CONFIG_STATIC_LIBGCC y
# tc needs kernel-specific headers not available in the cross toolchain sysroot
set_opt CONFIG_TC n

# Accept any new questions with their defaults (non-interactive)
yes "" | make -C "$BB_BUILD" ARCH=arm CROSS_COMPILE="$CROSS" oldconfig

echo "=== Building busybox ==="
make -C "$BB_BUILD" ARCH=arm CROSS_COMPILE="$CROSS" \
    CFLAGS="$ARCH_FLAGS" \
    -j"$(nproc)"

echo "=== Assembling initramfs ==="
FS="$WORK/rootfs"
mkdir -p "$FS"/{bin,sbin,usr/bin,usr/sbin,etc/init.d,dev,proc,sys,tmp,root}

# Install busybox + symlinks
make -C "$BB_BUILD" ARCH=arm CROSS_COMPILE="$CROSS" \
    CONFIG_PREFIX="$FS" install

# /etc/inittab: spawn a root shell directly on the iPod's UART
cat > "$FS/etc/inittab" <<'EOF'
::sysinit:/etc/init.d/rcS
::respawn:/bin/sh
::ctrlaltdel:/sbin/reboot
::shutdown:/bin/umount -a -r
EOF

cat > "$FS/etc/init.d/rcS" <<'EOF'
#!/bin/sh
mount -t proc     proc     /proc
mount -t sysfs    sysfs    /sys
mount -t devtmpfs devtmpfs /dev 2>/dev/null || true
echo "iPod nano 3g — Linux $(uname -r)"
EOF
chmod +x "$FS/etc/init.d/rcS"

# /init is the kernel's entry point into initramfs
cat > "$FS/init" <<'EOF'
#!/bin/sh
exec /sbin/init
EOF
chmod +x "$FS/init"

echo "=== Packing initramfs (via fakeroot for device nodes) ==="
OUT="$SCRIPT_DIR/initramfs-n3g.cpio.gz"
fakeroot bash -c "
    mknod -m 600 '$FS/dev/console' c 5 1
    mknod -m 666 '$FS/dev/null'    c 1 3
    mknod -m 666 '$FS/dev/zero'    c 1 5
    cd '$FS' && find . | cpio -H newc -o --quiet | gzip -9
" > "$OUT"

echo "Done: $OUT ($(du -sh "$OUT" | cut -f1))"
