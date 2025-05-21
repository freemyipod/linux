mkimage -A arm -C none -O linux -T multi -a 0x08000000 -e 0x08000000 -d arch/arm/boot/zImage:../initramfs.cpio.gz:../u-boot/arch/arm/dts/s5l8700.dtb mImage
