/*
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 *  Copyright (C) 2020 Alexey Spirkov <dev@alsp.net>
 */

#ifndef __1888BC048_H
#define __1888BC048_H

#include <linux/sizes.h>

#ifdef CONFIG_SPL_BUILD

#define CONFIG_SPL_STACK		(0x00060000 - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SPL_MAX_FOOTPRINT	0x00040000

#define CONFIG_SYS_SPL_MALLOC_SIZE	SZ_128K
#define CONFIG_SYS_SPL_MALLOC_START	0x00060000
#endif

#define CONFIG_SYS_MALLOC_LEN		SZ_16M

#define CONFIG_SYS_LOAD_ADDR        0x80000000

/* to check following */
#define CONFIG_SYS_SDRAM_BASE		0x80000000
#define CONFIG_SYS_SDRAM_SIZE		SZ_512M
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_TEXT_BASE + SZ_32M - \
					 GENERATED_GBL_DATA_SIZE)

#define CONFIG_SYS_DDR_SIZE     SZ_2G

#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS      0x40000

#ifndef CONFIG_SYS_L2CACHE_OFF
#define CONFIG_SYS_L2_PL310
#define CONFIG_SYS_PL310_BASE	0x01106000
#endif


#ifndef CONFIG_BIVK_BC048

#define CONFIG_SYS_MMC_ENV_DEV 0
#define CONFIG_USE_BOOTARGS
#define CONFIG_BOOTARGS "root=/dev/mmcblk0p2 rootwait"
#define CONFIG_IPADDR 192.168.1.5
#define CONFIG_SERVERIP 192.168.1.2
#define CONFIG_NETMASK 255.255.255.0
#define CONFIG_HOSTNAME "bc048"
#define CONFIG_LOADADDR 80000000
#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootcmd=run kernelsd\0" \
	"bootfile=bc048/uImage\0" \
	"fdt_addr_r=81000000\0" \
	"hostname=bc048\0" \
	"kernel=run loadfdt; run loadkernel; bootm ${loadaddr} - ${fdt_addr_r}\0" \
	"kernelsd=run loadsd; bootm ${loadaddr} - ${fdt_addr_r}\0" \
	"loadfdt=tftp ${fdt_addr_r} bc048/bc048.dtb\0" \
	"loadfdtsd=ext4load mmc 0:2 ${fdt_addr_r} /boot/uImage-bc048.dtb\0" \
	"loadkernel=tftp ${loadaddr} ${bootfile}\0" \
	"loadkernelsd=ext4load mmc 0:2 ${loadaddr} /boot/uImage-bc048.bin\0" \
	"loadsd=run loadfdtsd; run loadkernelsd\0" \
	"tftptimeout=1000\0" \
	"tftptimeoutcountmax=100\0"

#else

#define CONFIG_SYS_MMC_ENV_DEV 1
#define CONFIG_USE_BOOTARGS
#define CONFIG_BOOTARGS "root=/dev/mmcblk1p2 rw rootwait console=ttyRCM0,115200"
#define CONFIG_IPADDR 192.168.1.4
#define CONFIG_SERVERIP 192.168.1.1
#define CONFIG_NETMASK 255.255.0.0
#define CONFIG_HOSTNAME "bc048"
#define CONFIG_LOADADDR 80000000
#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootcmd=run kernelsd\0" \
	"bootfile=bc048/uImage\0" \
	"fdt_addr_r=81000000\0" \
	"hostname=bc048\0" \
	"kernel=run loadfdt; run loadkernel; bootm ${loadaddr} - ${fdt_addr_r}\0" \
	"kernelsd=run loadsd; bootm ${loadaddr} - ${fdt_addr_r}\0" \
	"loadfdt=tftp ${fdt_addr_r} bc048/bc048.dtb\0" \
	"loadfdtsd=ext4load mmc 1:2 ${fdt_addr_r} /boot/uImage-bc048.dtb\0" \
	"loadkernel=tftp ${loadaddr} ${bootfile}\0" \
	"loadkernelsd=ext4load mmc 1:2 ${loadaddr} /boot/uImage-bc048.bin\0" \
	"loadsd=run loadfdtsd; run loadkernelsd\0" \
	"tftptimeout=1000\0" \
	"tftptimeoutcountmax=100\0"

#endif

#endif
