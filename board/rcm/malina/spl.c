/*
 * RCM 1888TX018 SPL startup code
 *
 * Copyright (C) 2018 AstroSoft
 *               Alexey Spirkov <alexeis@astrosoft.ru>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <spi.h>
#include <spi_flash.h>
#include <asm/io.h>
#include <asm/tlb47x.h>
#include "boot.h"

DECLARE_GLOBAL_DATA_PTR;

extern void _start(void);
extern void _start_secondary(void);

static const __attribute__((used)) __attribute__((section(".header")))
struct rumboot_header hdr = {
	.magic = RUMBOOT_HEADER_MAGIC,
	.entry_point_0 = (uint32_t)&_start,
	.entry_point_1 = (uint32_t)&_start_secondary,
};

void ddr_init (int slowdown);
int testdramfromto(uint *pstart, uint *pend);

void rcm_mtd_arbiter_init(void);
void rcm_sram_nor_init(void);

volatile unsigned int* _test_addr;

static void init_byte_order(void)
{
	tlb47x_inval( 0x30000000, TLBSID_256M ); 
	tlb47x_map_guarded(0x1030000000, 0x30000000, TLBSID_256M, TLB_MODE_NONE, TLB_MODE_RW);
}

/* SPL should works without DDR usage, test part of DDR for loading main U-boot and load it */

void board_init_f(ulong dummy)
{
	init_byte_order();

	spl_early_init();

	preloader_console_init();

	spl_set_bd();

	board_init_r(NULL, 0);
}

u32 spl_boot_device(void)
{
#if defined(CONFIG_TARGET_1888TX018_MALINA_SPL_FIRST_DEVICE_MMC)
	return BOOT_DEVICE_MMC1;
#elif defined(CONFIG_TARGET_1888TX018_MALINA_SPL_FIRST_DEVICE_SPI_FLASH)
	return BOOT_DEVICE_SPI;
#elif defined(CONFIG_TARGET_1888TX018_MALINA_SPL_FIRST_DEVICE_NOR)
	return BOOT_DEVICE_NOR;
#elif defined(CONFIG_TARGET_1888TX018_MALINA_SPL_FIRST_DEVICE_UART_EDCL)
	return BOOT_DEVICE_XMODEM_EDCL;
#else
#error First device for the bootloader image from SPL is not defined
#endif
}


bool is_ddr_ok(void)
{
	int i;
	_test_addr = (unsigned int*)0x4D000000; 

	for(i = 0; i < 256; i++)
		_test_addr[i] = (unsigned int) &_test_addr[i];
	printf("DDR test...");

	for(i = 0; i < 256; i++)
	{
		if(_test_addr[i] != (unsigned int) &_test_addr[i])
		{
			printf(" Error at %d [%08X != %08X]\n", i, _test_addr[i], (unsigned int)&_test_addr[i]);
			break;
		}
	}
	if(i == 256)
	{
		printf(" OK.\n");
		return 1;
	}
	else
		return 0;
}

void usleep(uint32_t usec);
#if MAKE_STRESS_TEST
int memory_stress_post_test(int flags);
#endif

void spl_board_init(void)
{
	/* init dram */
#ifdef CONFIG_1888TX018_DDR_SPD
	ddr_init(0);
#else
	ddr_init(2);
#endif

	if(!is_ddr_ok())
	{
		printf("Error in DDR resetting...\n");
		usleep(1000);
		do_reset(0,0,0,0);
	}
	gd->ram_size = CONFIG_SYS_DDR_SIZE;

#ifdef CONFIG_MTD_RCM_NOR
	rcm_mtd_arbiter_init();
	rcm_sram_nor_init();
#endif

#if MAKE_STRESS_TEST
	memory_stress_post_test(0);
#endif
}


#ifdef CONFIG_SPL_MMC_SUPPORT
u32 spl_boot_mode(const u32 boot_device)
{
	return MMCSD_MODE_RAW;
}
#endif


void board_boot_order(u32 *spl_boot_list)
{
	spl_boot_list[0] = spl_boot_device();
	switch (spl_boot_list[0]) {
	case BOOT_DEVICE_MMC1:
		spl_boot_list[1] = BOOT_DEVICE_SPI;
		spl_boot_list[2] = BOOT_DEVICE_NOR;         // NOR before NAND, becose now GPIOAFSEL is initialized for NOR earlier(see spl_board_init)
		spl_boot_list[3] = BOOT_DEVICE_NAND;        // and function for initialization NOR is absent,but for NAND this function exist as self (nand_init)
		spl_boot_list[4] = BOOT_DEVICE_XMODEM_EDCL; // for adding new elements increase array size
		break;
	case BOOT_DEVICE_SPI:
		spl_boot_list[1] = BOOT_DEVICE_MMC1;
		spl_boot_list[2] = BOOT_DEVICE_NOR;
		spl_boot_list[3] = BOOT_DEVICE_NAND;
		spl_boot_list[4] = BOOT_DEVICE_XMODEM_EDCL;
		break;
	case BOOT_DEVICE_NOR:
		spl_boot_list[1] = BOOT_DEVICE_MMC1;
		spl_boot_list[1] = BOOT_DEVICE_SPI;
		spl_boot_list[3] = BOOT_DEVICE_NAND;
		spl_boot_list[4] = BOOT_DEVICE_XMODEM_EDCL;
		break;
	}
}

#define SPRN_DBCR0_44X 0x134
#define DBCR0_RST_SYSTEM 0x30000000

int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned long tmp;

	asm volatile(
		"mfspr	%0,%1\n"
		"oris	%0,%0,%2@h\n"
		"mtspr	%1,%0"
		: "=&r"(tmp)
		: "i"(SPRN_DBCR0_44X), "i"(DBCR0_RST_SYSTEM));

	return 0;
	}

