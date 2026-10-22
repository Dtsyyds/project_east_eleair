#!/bin/bash

CURDIR=`pwd`
echo "Your current directory is $CURDIR. This is where the MVSDK software will be installed..."
A=`whoami`
B=`arch`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   echo "Fail !!!"
   exit 1;
fi

# 切换到安装包所在目录：后续 cp/-f 判断均基于包根相对路径
cd "$(dirname "$0")" || exit 1

cp 88-mvusb.rules /etc/udev/rules.d/
cp 99-mvusb.rules /etc/udev/rules.d/

# 仅智能机（ita）包附带 99-mvusb-ita.rules：放开/dev/mem并即时生效；标准包不含此文件
if [ -f 99-mvusb-ita.rules ]; then
   cp 99-mvusb-ita.rules /etc/udev/rules.d/
   udevadm control --reload-rules 2>/dev/null || true
   udevadm trigger --name-match=mem 2>/dev/null || true
fi

cp include/* /usr/include/
echo "Copy header files"

if [ -f /lib/ld-musl-arm.so.1 ] || [ -f /lib/ld-musl-armhf.so.1 ]; then
	cp lib/arm_musl/libMVSDK.so /lib
	echo "Copy arm_musl/libMVSDK.so to /lib"
elif [ $B == 'x86_64' ]; then
	cp lib/x64/libMVSDK.so /lib
	echo "Copy x64/libMVSDK.so to /lib"
elif [ $B == 'aarch64' ]; then
	cp lib/arm64/libMVSDK.so /lib
	echo "Copy arm64/libMVSDK.so to /lib"
elif [[ ${B:2} == '86' ]]; then
	cp lib/x86/libMVSDK.so /lib
	echo "Copy x86/libMVSDK.so to /lib"
else
	cp lib/arm/libMVSDK.so /lib
	echo "Copy arm/libMVSDK.so to /lib"
fi

echo "Successful"
echo "Please  restart system  now!!!"
