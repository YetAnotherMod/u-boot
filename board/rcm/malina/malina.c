#undef DEBUG
#include <common.h>
#include <usb.h>
#include <dm.h>
#include "ddr_spd.h"
#include "rcm_dimm_params.h"
#include <fdt_support.h>
#include <env.h>
#include <asm/tlb47x.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_MTD_RCM_NOR
	void rcm_mtd_arbiter_init(void);
	void rcm_sram_nor_init(void);
#endif

typedef void (*voidcall)(void);

#define SPRN_DBCR0_44X 0x134
#define DBCR0_RST_SYSTEM 0x30000000

int do_reset(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
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

int get_ddr3_spd_bypath(const char *spdpath, ddr3_spd_eeprom_t *spd);
unsigned int rcm_compute_dimm_parameters(const ddr3_spd_eeprom_t *spd,
											  dimm_params_t *pdimm,
											  unsigned int dimm_number);

void ddr_init(void);

static phys_size_t ddr_em0_size = 0;
static phys_size_t ddr_em1_size = 0;

#if defined(CONFIG_OF_LIBFDT)

int arch_fixup_fdt(void *blob)
{
	int num_banks = 0;
	u64 base[2];
	u64 size[2];

	if (ddr_em0_size != 0)
	{
		base[num_banks] = 0x0000000000ULL;
		size[num_banks] = ddr_em0_size;
		num_banks++;
	}

	if (ddr_em1_size != 0)
	{
		base[num_banks] = 0x0200000000ULL;
		size[num_banks] = ddr_em1_size;
		num_banks++;
	}

	printf("Setup memory for kernel for %d banks...\n", num_banks);

	return fdt_fixup_memory_banks(blob, base, size, num_banks);
}

#endif

phys_size_t get_physical_mem_size(void)
{
	phys_size_t result = 0;

#ifdef CONFIG_1888TX018_DDR_SPD
	int dimm0_params_invalid = 1;
	int dimm1_params_invalid = 1;
	dimm_params_t dpar0, dpar1;

	ddr3_spd_eeprom_t spd;

	if (!get_ddr3_spd_bypath("spd0-path", &spd))
		dimm0_params_invalid = rcm_compute_dimm_parameters(&spd, &dpar0, 0);

	if (!get_ddr3_spd_bypath("spd1-path", &spd))
		dimm1_params_invalid = rcm_compute_dimm_parameters(&spd, &dpar1, 1);

	if (!dimm0_params_invalid)
	{
		ddr_em0_size = dpar0.capacity * 32 / dpar0.primary_sdram_width;
		result += ddr_em0_size;
	}

	if (!dimm1_params_invalid)
	{
		ddr_em1_size = dpar1.capacity * 32 / dpar0.primary_sdram_width;
		result += ddr_em1_size;
	}
#else
	#ifdef CONFIG_1888TX018_DDR_EM0_SIZE
		ddr_em0_size = ((phys_size_t)CONFIG_1888TX018_DDR_EM0_SIZE) * SZ_1M;
	#endif

	#ifdef CONFIG_1888TX018_DDR_EM1_SIZE
		ddr_em1_size = ((phys_size_t)CONFIG_1888TX018_DDR_EM1_SIZE) * SZ_1M;
	#endif
	result = ddr_em0_size + ddr_em1_size;
#endif
	return result;
}

int dram_init(void)
{
#ifdef CONFIG_SPL_BUILD
	ddr_init();
#else
	gd->ram_size = get_physical_mem_size();
#endif
	return 0;
}

#define SPR_TBL_R 0x10C
#define SPR_TBU_R 0x10D

static uint64_t tcurrent(void)
{
	uint32_t valuel = 0;
	uint32_t valueh = 0;

	__asm volatile(
		"mfspr %0, %1 \n\t"
		: "=r"(valuel)
		: "i"(SPR_TBL_R)
		:);

	__asm volatile(
		"mfspr %0, %1 \n\t"
		: "=r"(valueh)
		: "i"(SPR_TBU_R)
		:);


	return (((uint64_t)valueh) << 32) + valuel;
}

static uint32_t mcurrent(void)
{
	return (tcurrent() / (1000 * TIMER_TICKS_PER_US));
}

int __weak timer_init(void)
{
	return 0;
}
/* Returns time in milliseconds */
ulong get_timer(ulong base)
{
	return mcurrent() - base;
}

#ifdef CONFIG_CMD_USB_MASS_STORAGE

int board_usb_init(int index, enum usb_init_type init)
{
	if (init == USB_INIT_DEVICE)
	{
		struct udevice *dev;
		debug("board_usb_init called\n");
		int ret = uclass_get_device(UCLASS_USB, 0, &dev);
		if (ret)
			debug("unable to find USB device in FDT\n");
	}
	return 0;
}

#endif

#if defined(CONFIG_SYS_DRAM_TEST)

int testdramfromto(uint *pstart, uint *pend)
{
	uint *p;

	printf("Testing DRAM from 0x%08x to 0x%08x\n",
		   CONFIG_SYS_MEMTEST_START,
		   CONFIG_SYS_MEMTEST_END);

	printf("DRAM test phase 1:\n");
	for (p = pstart; p < pend; p++)
		*p = 0xaaaaaaaa;

	for (p = pstart; p < pend; p++)
	{
		if (*p != 0xaaaaaaaa)
		{
			printf("DRAM test fails at: %08x\n", (uint)p);
			return 1;
		}
	}

	printf("DRAM test phase 2:\n");
	for (p = pstart; p < pend; p++)
		*p = 0x55555555;

	for (p = pstart; p < pend; p++)
	{
		if (*p != 0x55555555)
		{
			printf("DRAM test fails at: %08x\n", (uint)p);
			return 1;
		}
	}

	printf("DRAM test passed.\n");
	return 0;
}

int testdram(void)
{
	uint *pstart = (uint *)CONFIG_SYS_MEMTEST_START;
	uint *pend = (uint *)CONFIG_SYS_MEMTEST_END;
	return testdramfromto(pstart, pend);
}

#endif

int power_init_board(void)
{
#ifdef CONFIG_MTD_RCM_NOR
	rcm_mtd_arbiter_init();
	rcm_sram_nor_init();
	// this line was commented for using U-Boot without loading the environment from any sources (CONFIG_ENV_IS_NOWHERE)
	// gd->env_valid = ENV_VALID;
#endif
	return 0;
}

phys_size_t get_effective_memsize(void)
{
#ifndef CONFIG_VERY_BIG_RAM
	return gd->ram_size;
#else
	/* limit stack to what we can reasonable map */
	return ((gd->ram_size > CONFIG_MAX_MEM_MAPPED) ?
		CONFIG_MAX_MEM_MAPPED : gd->ram_size);
#endif 
}

int dram_init_banksize(void)
{
#if defined(CONFIG_NR_DRAM_BANKS)
	unsigned i = 1;
	gd->bd->bi_dram[0].start = 0x0000000000;
	gd->bd->bi_dram[0].size = ddr_em0_size;

	if (CONFIG_NR_DRAM_BANKS > 1) {
		gd->bd->bi_dram[1].start = 0x0200000000;
		gd->bd->bi_dram[1].size = ddr_em1_size;
		i = 2;
	}
	for (; i < CONFIG_NR_DRAM_BANKS; i++) {
			gd->bd->bi_dram[i].start = 0;
			gd->bd->bi_dram[i].size = 0;
	}
#endif
	return 0;
}

/*  New version of u-boot uses logical memory blocks with regions reservation.
	We use memory map, where addresses for loading images is higher, than stack pointer.
	So we have self reservation of memory area. */

void board_lmb_reserve(struct lmb *lmb)
{
//#ifdef CONFIG_NR_DRAM_BANKS
// 	int i;
// 	lmb_init(lmb); // !!! clear all previous settings
// 	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
// 		if (gd->bd->bi_dram[i].size) {
// 			lmb_add(lmb, gd->bd->bi_dram[i].start,
// 				gd->bd->bi_dram[i].size);
// 		}
// 	}
// 	lmb_reserve( lmb,
// 				 CONFIG_SYS_TEXT_BASE,
// 				 (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_RAM_SIZE) - CONFIG_SYS_TEXT_BASE );
//#endif

	lmb_reserve(lmb, CONFIG_SYS_TEXT_BASE, CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_RAM_SIZE - CONFIG_SYS_TEXT_BASE);
}

void bi_dram_print_info( void ) {
#if defined(CONFIG_NR_DRAM_BANKS)
	unsigned int i;
	printf( "Dram regions: \n");
	for( i=0; i<CONFIG_NR_DRAM_BANKS; i++ ) {
		printf( "%u: %016llx-%016llx  (%s)\n", i, gd->bd->bi_dram[i].start, gd->bd->bi_dram[i].size, gd->bd->bi_dram[i].size ? "u" :"f" );
	}
#endif
}

int bi_dram_add_region(uint32_t start, uint32_t size)
{
#if defined(CONFIG_NR_DRAM_BANKS)
	unsigned int i;
	for( i=1; i<CONFIG_NR_DRAM_BANKS; i++ ) {
		if( gd->bd->bi_dram[i].size == 0 ) {
			gd->bd->bi_dram[i].start = start;
			gd->bd->bi_dram[i].size = size;
			break;
		}
	}
#endif
	return 0;
}

int bi_dram_drop_region(uint32_t start, uint32_t size)
{
#if defined(CONFIG_NR_DRAM_BANKS)
	unsigned int i;
	for( i=1; i<CONFIG_NR_DRAM_BANKS; i++ ) {
		if( gd->bd->bi_dram[i].start == start && gd->bd->bi_dram[i].size == size ) {
			gd->bd->bi_dram[i].start = 0;
			gd->bd->bi_dram[i].size = 0;
			break;
		}
	}
#endif
	return 0;
}

int bi_dram_drop_all(void)
{
#if defined(CONFIG_NR_DRAM_BANKS)
	unsigned int i;
	for( i=1; i<CONFIG_NR_DRAM_BANKS; i++ ) {
		gd->bd->bi_dram[i].start = 0;
		gd->bd->bi_dram[i].size = 0;
	}
#endif
	return 0;
}

int add_code_guard(void)
{
	const uint32_t code_start = CONFIG_SYS_TEXT_BASE & ~0xFFFFFF;
	const uint32_t data_start = CONFIG_SYS_INIT_RAM_ADDR & ~0xFFFFFF;

	// code
	tlb47x_map_nocache(code_start - CONFIG_SYS_SDRAM_BASE, code_start, TLBSID_16M, TLB_MODE_NONE, TLB_MODE_RX);
	// data
	tlb47x_map_nocache(data_start - CONFIG_SYS_SDRAM_BASE, data_start, TLBSID_16M, TLB_MODE_NONE, TLB_MODE_RW);
	// invalidate original page
	tlb47x_inval(CONFIG_SYS_SDRAM_BASE, TLBSID_256M);

	return 0;
}

ulong board_get_usable_ram_top(ulong total_size)
{
	return CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_RAM_SIZE - RCM_PPC_STACK_SIZE;
}
