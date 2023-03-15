#!/bin/bash

HERE="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

BOARD=mimxrt1050_evk
TTY=/dev/ttyUSB2
BAUD=115200

(
        cd $HERE/..

        if [ -z "$1" ]; then
                west build -b $BOARD -d build_bl bootloader/mcuboot/boot/zephyr -- \
                -DCONFIG_BOOT_RAM_LOAD=y \
                -DCONFIG_BOOT_IMAGE_EXECUTABLE_RAM_START=0x20200000 \
                -DCONFIG_BOOT_IMAGE_EXECUTABLE_RAM_SIZE=256000 \
                -DCONFIG_MCUBOOT_CLEANUP_ARM_CORE=y

                west build -b $BOARD -d build_dfu_usb zephyr/samples/subsys/usb/dfu -- \
                -DCONFIG_BOOTLOADER_MCUBOOT=n \
                -DCONFIG_SHELL=y -DCONFIG_FLASH_SHELL=y -DCONFIG_FLASH_MAP_SHELL=y \
                -DCONFIG_IMG_ERASE_PROGRESSIVELY=y \
                -DCONFIG_FLASH_BASE_ADDRESS=0x20200000 \
		-DCONFIG_CODE_SEMC=y \
                -DCONFIG_FLASH_SIZE=256 \
                -DCONFIG_ROM_START_OFFSET=0x200 \
                -DCONFIG_BUILD_OUTPUT_HEX=y \
                -DCONFIG_CACHE_MANAGEMENT=y \
                -DCONFIG_INIT_ARCH_HW_AT_BOOT=y

                imgtool create --key bootloader/mcuboot/root-rsa-2048.pem \
                        --hex-addr 0x60040000 --header-size 1024 \
                        --slot-size 0x300000 --align 16 --version 0.0.0 \
                        --load-addr 0x20200000 \
                        $PWD/build_dfu_usb/zephyr/zephyr.hex \
                        $PWD/build_dfu_usb/zephyr/zephyr.signed.hex

                west build -b $BOARD -d build_hello_world zephyr/samples/hello_world -- \
                -DCONFIG_BOOTLOADER_MCUBOOT=n \
                -DCONFIG_FLASH_BASE_ADDRESS=0x20200000 \
                -DCONFIG_FLASH_SIZE=256 \
		-DCONFIG_CODE_SEMC=y \
                -DCONFIG_ROM_START_OFFSET=0x200 \
                -DCONFIG_BUILD_OUTPUT_HEX=y \
                -DCONFIG_CACHE_MANAGEMENT=y \
                -DCONFIG_INIT_ARCH_HW_AT_BOOT=y

                imgtool create --key bootloader/mcuboot/root-rsa-2048.pem \
                        --header-size 1024 \
                        --slot-size 0x300000 --align 16 --version 0.0.1 \
                        --load-addr 0x20200000 \
                        $PWD/build_hello_world/zephyr/zephyr.bin \
                        $PWD/build_hello_world/zephyr/zephyr.signed.bin

                west flash -d build_bl
                west flash -d build_dfu_usb --file build_dfu_usb/zephyr/zephyr.signed.hex

                sleep 2
        fi

        sudo dfu-util --alt 1 --download build_hello_world/zephyr/zephyr.signed.bin
)
