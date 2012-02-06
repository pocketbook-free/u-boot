#!/bin/sh

UBOOT=u-boot.ext

if [ ! -f ./${UBOOT} ]
then 
	echo "need uboot"
	exit 1
fi

if [ `id -u` -ne 0 ]
then
	echo "need root"
	exit 1
fi

if [ -z $1 ]
then
	echo "need path"
	exit 1
fi

echo Using: $1
DEVICE=$1
SIZE=`cat /sys/block/${DEVICE}/size`

BL1=$((${SIZE} - 18))
BL2=$((${SIZE} - 562))

echo "BL1: $BL1, BL2: $BL2"

dd if=./${UBOOT} of=/dev/${DEVICE} bs=512 seek=${BL1} count=16 && sync
dd if=./${UBOOT} of=/dev/${DEVICE} bs=512 seek=${BL2} && sync && sync

