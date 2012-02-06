#!/bin/sh

# Должен присутствовать arm-none-linux-gnueabi-gcc, иначе не будет работать

do_exit()
{
	echo $1
	exit 1
}

echo -n "." > localversion-1
hg id -i >> localversion-1

test ! -z "$1" || do_exit "Need device name"
CONFIG_NAME=$1

make distclean || do_exit "distclean failed"
make ${CONFIG_NAME}_config || do_exit "Config failed"
make || do_exit "Make failed"

