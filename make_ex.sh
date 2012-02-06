#!/bin/sh

# Должен присутствовать arm-none-linux-gnueabi-gcc, иначе не будет работать

do_exit()
{
	echo $1
	exit 1
}

echo -n "." > localversion-1
echo -n "E_" >> localversion-1
hg id -i >> localversion-1

test ! -z "$1" || do_exit "Need device name"
CONFIG_NAME=$1

make distclean || do_exit "distclean failed"
make ${CONFIG_NAME}ex_config || do_exit "Config failed"
make -j 2 || do_exit "Make failed"

mkimage -A arm -O u-boot -T firmware -C none -a 579fffc0 -e 57a00000 -n "ExtBoot" -d u-boot.bin u-boot.ext

dd if=u-boot.ext of=u-boot.tr bs=4 skip=1
echo -n "diag" > /tmp/diag
cat /tmp/diag u-boot.tr > u-boot.ext
rm u-boot.tr

sync && sync
