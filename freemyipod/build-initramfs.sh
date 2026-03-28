#!/bin/bash

set -eEx

ARCH=armv7
VER=3.23.2
MAJOR=v${VER%.*}
TAR=alpine-minirootfs-${VER}-${ARCH}.tar.gz
URL=https://dl-cdn.alpinelinux.org/alpine/${MAJOR}/releases/${ARCH}/${TAR}

wget -c $URL
WORK_DIR=$(mktemp -d)
trap 'sudo rm -rf "$WORK_DIR"' EXIT
tar xf ${TAR} -C $WORK_DIR

# login on the serial console
echo 'ttySAC0::respawn:/sbin/getty -L 115200 ttySAC0 vt100' >> "$WORK_DIR/etc/inittab"
echo ttySAC0 >> "$WORK_DIR/etc/securetty"

# create minimal /init for initramfs
cat >"$WORK_DIR/init" <<'EOF'
#!/bin/sh

echo "initramfs: starting init"
exec /sbin/init
EOF

chmod +x "$WORK_DIR/init"

# setup networking
echo 'nameserver 8.8.8.8' > "$WORK_DIR/etc/resolv.conf"
echo nano7g > "$WORK_DIR/etc/hostname"
cat >"$WORK_DIR/etc/network/interfaces" <<'EOF'
auto lo
iface lo inet

auto usb0
iface usb0 inet static
    address 10.0.0.2
    netmask 255.255.255.0
    gateway 10.0.0.1
EOF

# setup the rootfs
cp $(which qemu-arm-static) $WORK_DIR
# install packages
sudo systemd-nspawn -D $WORK_DIR /qemu-arm-static /sbin/apk add dropbear openrc haveged chrony zlib evtest i2c-tools

sudo systemd-nspawn -D $WORK_DIR /qemu-arm-static /bin/sh -c "export PATH=
source /etc/profile

# setup services
for s in devfs dmesg root
do
  rc-update add \$s sysinit
done

for s in chronyd bootmisc hostname sysctl haveged
do
  rc-update add \$s boot
done

rc-update add networking
rc-update add dropbear

# pre-generate dropbear host keys
for algo in rsa ecdsa ed25519
do dropbearkey -t \$algo -f /etc/dropbear/dropbear_\${algo}_host_key; done

# set root password
echo -ne 'alpine\nalpine\n' | passwd root
"

rm -rf "$WORK_DIR/qemu-arm-static"

# make the initramfs
(
  find $WORK_DIR -printf "%P\0" |
  sudo cpio --directory=$WORK_DIR --null --create --verbose --owner root:root --format=newc |
  gzip -9
) > freemyipod/initramfs.cpio.gz
