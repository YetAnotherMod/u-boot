#undef DEBUG
#include "configs/1888tx018.h"
#include <common.h>
#include <usb.h>
#include <dm.h>
#include "ddr_spd.h"
#include "rcm_dimm_params.h"
#include <fdt_support.h>
#include <env.h>
#include <asm/tlb47x.h>
#include <post.h>
//#include <linux/io.h>

#define PCIE0_AXI_BASE                      0x10000000
#define DMA_ACCESS_BASE                     0x10500000
#define TEST_DATA_SIZE                      0x400

#define USE_DMA_FOR_TEST 0

//NWL PCIe AXI registers
#define BRIDGE_CORE_CFG_PCIE_RX1            0x00000004
#define BRIDGE_CORE_CFG_AXI_MASTER          0x00000008
#define BRIDGE_CORE_CFG_INTERRUPT           0x00000010
#define BRIDGE_CORE_CFG_PCIE_RX_MSG_FILTER  0x00000020

#define E_BREG_CAPABILITIES                 0x00000200
#define E_BREG_CONTROL                      0x00000208
#define E_BREG_BASE_LO                      0x00000210
#define E_BREG_BASE_HI                      0x00000214

#define E_ECAM_CAPABILITIES                 0x00000220
#define E_ECAM_CONTROL                      0x00000228
#define E_ECAM_BASE_LO                      0x00000230
#define E_ECAM_BASE_HI                      0x00000234

#define E_DREG_CAPABILITIES                 0x00000280
#define E_DREG_CONTROL                      0x00000288
#define E_DREG_BASE_LO                      0x00000290
#define E_DREG_BASE_HI                      0x00000294

#define I_MSII_CAPABILITIES                 0x00000300
#define I_MSII_CONTROL                      0x00000308
#define I_MSII_BASE_LO                      0x00000310

#define I_MSIX_CONTROL                      0x00000328
#define I_MSIX_BASE_LO                      0x00000330

#define DMA0_SRC_Q_PTR_LO                   0x00000000
#define DMA0_SRC_Q_PTR_HI                   0x00000004
#define DMA0_SRC_Q_SIZE                     0x00000008
#define DMA0_SRC_Q_LIMIT                    0x0000000c
#define DMA0_DST_Q_PTR_LO                   0x00000010
#define DMA0_DST_Q_PTR_HI                   0x00000014
#define DMA0_DST_Q_SIZE                     0x00000018
#define DMA0_DST_Q_LIMIT                    0x0000001c
#define DMA0_STAS_Q_PTR_LO                  0x00000020
#define DMA0_STAS_Q_PTR_HI                  0x00000024
#define DMA0_STAS_Q_SIZE                    0x00000028
#define DMA0_STAS_Q_LIMIT                   0x0000002c
#define DMA0_STAD_Q_PTR_LO                  0x00000030
#define DMA0_STAD_Q_PTR_HI                  0x00000034
#define DMA0_STAD_Q_SIZE                    0x00000038
#define DMA0_STAD_Q_LIMIT                   0x0000003c
#define DMA0_SRC_Q_NEXT                     0x00000040
#define DMA0_DST_Q_NEXT                     0x00000044
#define DMA0_STAS_Q_NEXT                    0x00000048
#define DMA0_STAD_Q_NEXT                    0x0000004c
#define DMA0_AXI_INTERRUPT_CONTROL          0x00000068
#define DMA0_AXI_INTERRUPT_STATUS           0x0000006c
#define DMA0_DMA_CONTROL                    0x00000078
#define MSI_INT_BASE                        0x80000000

#define BREG_PRESENT                        0x00000001
#define DMA_PRESENT                         0x00000001
#define I_MSII_PRESENT                      0x00000001

DECLARE_GLOBAL_DATA_PTR;

#if MAKE_STRESS_TEST

void ddr_tlbs_init(void);
void usleep(uint32_t usec);

#ifdef __GNUC__
uint32_t data_buf[TEST_DATA_SIZE >> 2] __attribute__ ((aligned(64)));
uint32_t mem_buf0[TEST_DATA_SIZE >> 2] __attribute__ ((aligned(64)));
uint32_t mem_buf1[TEST_DATA_SIZE >> 2] __attribute__ ((aligned(64)));
uint32_t log_buf[8]  __attribute__ ((aligned(64)));
uint32_t descr0[4]  __attribute__ ((aligned(64)));
uint32_t descr1[4]  __attribute__ ((aligned(64)));
uint32_t descr2[4]  __attribute__ ((aligned(64)));
uint32_t descr3[4]  __attribute__ ((aligned(64)));
#else
#error "buffers must be 64-byte aligned"
#endif 

static volatile uint32_t ioread32(uint32_t const base_addr)
{
    return *((volatile uint32_t*)(base_addr));
}

static void iowrite32(uint32_t const value, uint32_t const base_addr)
{
    *((volatile uint32_t*)(base_addr)) = value;
}

void dump_descr(void)
{
	printf("descr0: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			descr0[0], descr0[1], descr0[2], descr0[3]);
	printf("descr1: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			descr1[0], descr1[1], descr1[2], descr1[3]);
	printf("descr2: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			descr2[0], descr2[1], descr2[2], descr2[3]);
	printf("descr3: 0x%08x 0x%08x 0x%08x 0x%08x\n",
			descr3[0], descr3[1], descr3[2], descr3[3]);
}

void print_capabilities(void)
{
    printf("E_BREG_CAPABILITIES: 0x%08x\n", ioread32(PCIE0_AXI_BASE+E_BREG_CAPABILITIES));
    printf("E_DREG_CAPABILITIES: 0x%08x\n", ioread32(PCIE0_AXI_BASE+E_DREG_CAPABILITIES));
    printf("E_ECAM_CAPABILITIES: 0x%08x\n", ioread32(PCIE0_AXI_BASE+E_ECAM_CAPABILITIES));
}

/* invalid the TLBs for DDR and setup new ones to cover p_addr */
static int reset_tlb(phys_addr_t p_addr0, u32 *p_size, u32 vstart, u32 size)
{
	phys_addr_t p_addr1, phys_offset;
    uint32_t ddr0_base, ddr1_base, offset;

	tlb_size_id tlb_sid = tlb47x_get_tlb_sid_by_size(size >> 1);

	if(tlb_sid == TLBSID_ERR) tlb_sid = TLBSID_1G;
	offset = tlb47x_get_tlb_sid_size(tlb_sid);
	ddr0_base = vstart;
	ddr1_base = vstart + (offset);

	tlb47x_inval(ddr0_base, tlb_sid);
	tlb47x_inval(ddr1_base, tlb_sid);

	tlb_sid = tlb47x_get_tlb_sid_by_size(*p_size >> 1);
	offset = tlb47x_get_tlb_sid_size(tlb_sid);
	ddr0_base = CONFIG_SYS_DDR_SDRAM_BASE;
	ddr1_base = CONFIG_SYS_DDR_SDRAM_BASE + offset;

	phys_offset = p_addr0 - CONFIG_SYS_INIT_DDR0_ADDR_PHYS;
	p_addr1 = CONFIG_SYS_INIT_DDR1_ADDR_PHYS + phys_offset;

	//             physical,  logical,il1i,il1d,wimg,size,      umode,        smode,       window
	tlb47x_map_entry(p_addr0, ddr0_base, 0,   0,  4, tlb_sid, TLB_MODE_RWX, TLB_MODE_RWX, MEM_WINDOW_SHARED);
	tlb47x_map_entry(p_addr1, ddr1_base, 0,   0,  4, tlb_sid, TLB_MODE_RWX, TLB_MODE_RWX, MEM_WINDOW_SHARED);

	*p_size = (offset << 1);

	return 0;
}

/*
 * slide the testing window up to test another area
 * for 32_bit system, the maximum testable memory is limited to
 * CONFIG_MAX_MEM_MAPPED
 */
int arch_memory_test_advance(u32 *vstart, u32 *size, phys_addr_t *phys_offset)
{
	phys_addr_t p_addr0, p_addr1;
	u32 p_size = (u32)min((u64)gd->ram_size, (u64)(CONFIG_MAX_MEM_MAPPED));
	phys_size_t test_cap, done_size;
	uint32_t addr0_hi, addr0_low, addr1_hi, addr1_low;
	uint32_t addr0_hi_end, addr0_low_end, addr1_hi_end, addr1_low_end;
    u64 addr0_end, addr1_end;

#if !defined(CONFIG_PHYS_64BIT)
		test_cap = p_size;
#else
		test_cap = gd->ram_size;
#endif

	p_addr0 = CONFIG_SYS_INIT_DDR0_ADDR_PHYS + (*size >> 1) + (*phys_offset);
	p_addr1 = CONFIG_SYS_INIT_DDR1_ADDR_PHYS + (*size >> 1) + (*phys_offset);

	done_size = (*size) + (*phys_offset << 1);
	if (done_size < test_cap - 1) {
		p_size = (u32)min((u64)(test_cap - done_size), (u64)(CONFIG_MAX_MEM_MAPPED));
		if (reset_tlb(p_addr0, &p_size, *vstart, *size) == -1)
			return -1;
		*vstart = CONFIG_SYS_SDRAM_BASE;
		*size = (u32) p_size;
		*phys_offset += (p_size >> 1);

        addr0_end = (u64)p_addr0 + (p_size >> 1) - 1;
        addr1_end = (u64)p_addr1 + (p_size >> 1) - 1;

        addr0_hi  = ((u64)p_addr0 >> 32);
        addr0_low = p_addr0 & 0xFFFFFFFF;
        addr1_hi  = ((u64)p_addr1 >> 32);
        addr1_low = p_addr1 & 0xFFFFFFFF;
        addr0_hi_end  = addr0_end >> 32;
        addr0_low_end = addr0_end & 0xFFFFFFFF;
        addr1_hi_end  = addr1_end >> 32;
        addr1_low_end = addr1_end & 0xFFFFFFFF;

		printf("Testing DDR0 0x%02x%08x - 0x%02x%08x, DDR1 0x%02x%08x - 0x%02x%08x\n",
			addr0_hi, addr0_low, addr0_hi_end, addr0_low_end,
			addr1_hi, addr1_low, addr1_hi_end, addr1_low_end);

	} else
		return 1;

	return 0;
}

/* initialization for testing area */
int arch_memory_test_prepare(u32 *vstart, u32 *size, phys_addr_t *phys_offset)
{
	phys_size_t p_size = (phys_size_t )min((u64)gd->ram_size, (u64)(CONFIG_MAX_MEM_MAPPED));
	uint32_t addr0_hi, addr0_low, addr1_hi, addr1_low;
	uint32_t addr0_hi_end, addr0_low_end, addr1_hi_end, addr1_low_end;
    u64 addr0_end, addr1_end;

	*vstart = CONFIG_SYS_SDRAM_BASE;
	*size = (u32) p_size;	/* CONFIG_MAX_MEM_MAPPED < 4G */
	*phys_offset = 0;

#if USE_DMA_FOR_TEST
	puts("\nDDR memory test starting (DMA used).\n");
#else
	puts("\nDDR memory test starting.\n");
#endif

#if !defined(CONFIG_PHYS_64BIT)
		if (gd->ram_size > CONFIG_MAX_MEM_MAPPED) {
			puts("Cannot test more than ");
			print_size(CONFIG_MAX_MEM_MAPPED,
				" without proper 36BIT support.\n");
		}
#endif

    addr0_end = (u64)CONFIG_SYS_INIT_DDR0_ADDR_PHYS + (*size >> 1) - 1;
    addr1_end = (u64)CONFIG_SYS_INIT_DDR1_ADDR_PHYS + (*size >> 1) - 1;

    addr0_hi  = ((u64)CONFIG_SYS_INIT_DDR0_ADDR_PHYS >> 32);
    addr0_low = CONFIG_SYS_INIT_DDR0_ADDR_PHYS & 0xFFFFFFFF;
    addr1_hi  = ((u64)CONFIG_SYS_INIT_DDR1_ADDR_PHYS >> 32);
    addr1_low = CONFIG_SYS_INIT_DDR1_ADDR_PHYS & 0xFFFFFFFF;
    addr0_hi_end  = addr0_end >> 32;
    addr0_low_end = addr0_end & 0xFFFFFFFF;
    addr1_hi_end  = addr1_end >> 32;
    addr1_low_end = addr1_end & 0xFFFFFFFF;

    printf("Testing DDR0 0x%02x%08x - 0x%02x%08x, DDR1 0x%02x%08x - 0x%02x%08x\n",
        addr0_hi, addr0_low, addr0_hi_end, addr0_low_end,
        addr1_hi, addr1_low, addr1_hi_end, addr1_low_end);

	return 0;
}

/* invalid TLBs for DDR and remap as normal after testing */
int arch_memory_test_cleanup(u32 *vstart, u32 *size, phys_addr_t *phys_offset)
{
    uint32_t ddr0_base, ddr1_base, offset;
    tlb_size_id tlb_sid = tlb47x_get_tlb_sid_by_size(*size >> 1);

    if(tlb_sid == TLBSID_ERR) tlb_sid = TLBSID_1G;
    offset = tlb47x_get_tlb_sid_size(tlb_sid);
    ddr0_base = *vstart;
    ddr1_base = *vstart + offset;

	/* disable the TLBs for this testing */
	tlb47x_inval(ddr0_base, tlb_sid);
	tlb47x_inval(ddr1_base, tlb_sid);
	
	*phys_offset = 0;

	puts("Remap DDR ");
	ddr_tlbs_init();
	puts("\n");

	return 0;
}

void arch_memory_failure_handle(void)
{
	puts("DDR test failed!\n");
}

//*****************************************************************************
//  Reverse byte order of uint32_t argument
//*****************************************************************************
static uint32_t rotate_be (uint32_t val)
{
    return (((val>>24)&0x000000ff) | ((val>>8)&0x0000ff00) | ((val<<8)&0x00ff0000) | ((val<<24)&0xff000000));
}  

//*****************************************************************************
//  Create reference data array for next transactions
//*****************************************************************************
static void clear_destination(void)
{
    uint32_t i;

    for (i=0; i<8; i++)
    {
        iowrite32(0x00000000, (uint32_t)&log_buf[i]);
    }

    for (i=0; i<(TEST_DATA_SIZE >> 2); i++)
    {
        iowrite32(0x00000000, (uint32_t)&mem_buf0[i]);
        iowrite32(0x00000000, (uint32_t)&mem_buf1[i]);
    }
}

//*****************************************************************************
//  Create reference data array for next transactions
//*****************************************************************************
static void create_data_array(void)
{
  	uint32_t i;
  
  	for (i=0; i<(TEST_DATA_SIZE >> 2); i++) 
    	if (i%2)
      		iowrite32(i + (i<<8) + (i<<16) + (i<<24), (uint32_t)&data_buf[i]);
    	else
      		iowrite32(~(i + (i<<8) + (i<<16) + (i<<24)), (uint32_t)&data_buf[i]);
}

//*****************************************************************************
//  PCIe DMA data transaction subroutine
//  Used in this test, because direct CPU <-> DDR memory connection
//    was actually unable
//*****************************************************************************
static int pcie_dma_axi_axi(uint32_t dma_access_base, uint32_t src_addr_high, uint32_t src_addr, uint32_t dst_addr_high, uint32_t dst_addr, uint32_t size)
{
#if USE_DMA_FOR_TEST
    uint32_t reg;

    //PCIe Espresso DMA base config
    iowrite32(0x33, PCIE0_AXI_BASE+BRIDGE_CORE_CFG_PCIE_RX1); 	//Set ar/wcache for AXI master
    iowrite32(0x22, PCIE0_AXI_BASE+BRIDGE_CORE_CFG_AXI_MASTER); //Set max transaction size - 256 bytes

    //Interrupts init
    iowrite32(0x1, PCIE0_AXI_BASE+BRIDGE_CORE_CFG_INTERRUPT); 	//Route interrupts to AXI
    iowrite32(0x4, PCIE0_AXI_BASE+BRIDGE_CORE_CFG_PCIE_RX_MSG_FILTER);	//Enable msg interrupts

    reg = ioread32(PCIE0_AXI_BASE+I_MSII_CAPABILITIES) & I_MSII_PRESENT;
    if(!reg){
        puts("Error! I_MSII is not present\n");
        return reg;
    }
    iowrite32(MSI_INT_BASE, PCIE0_AXI_BASE+I_MSII_BASE_LO);	//MSI interrupts hit base address translation
    iowrite32(0x00008001, PCIE0_AXI_BASE+I_MSII_CONTROL);	//MSI interrupts and status enable

    reg = ioread32(PCIE0_AXI_BASE+E_DREG_CAPABILITIES) & DMA_PRESENT;
    if(!reg){
        puts("Error! DMA is not present\n");
        return reg;
    }
    //Egress DMA Register Translation
    iowrite32(dma_access_base, PCIE0_AXI_BASE+E_DREG_BASE_LO);
    iowrite32(0x00030001, PCIE0_AXI_BASE+E_DREG_CONTROL);

    reg = ioread32(PCIE0_AXI_BASE+E_BREG_CAPABILITIES) & BREG_PRESENT;
    if(!reg){
        puts("Error! BREG is not present\n");
        return reg;
    }
    //Egress Bridge Register Translation
    iowrite32(PCIE0_AXI_BASE, PCIE0_AXI_BASE+E_BREG_BASE_LO);
    iowrite32(0x00030001, PCIE0_AXI_BASE+E_BREG_CONTROL);

    //Reset DMA
    iowrite32(0x2, dma_access_base+DMA0_DMA_CONTROL);
    iowrite32(0x0, dma_access_base+DMA0_DMA_CONTROL);

    //Set DMA quiees addresses
    iowrite32(0x2, dma_access_base+DMA0_SRC_Q_SIZE ) ;  //
    iowrite32(0x1, dma_access_base+DMA0_SRC_Q_LIMIT ) ; //
    iowrite32((u32)(descr0) | 0xf, dma_access_base+DMA0_SRC_Q_PTR_LO) ; //Set source queue
    iowrite32(0x2, dma_access_base+DMA0_DST_Q_SIZE  ) ; //
    iowrite32(0x1, dma_access_base+DMA0_DST_Q_LIMIT ) ; //
    iowrite32((u32)(descr1) | 0xf, dma_access_base+DMA0_DST_Q_PTR_LO) ; //Set destination queue
    iowrite32(0x2, dma_access_base+DMA0_STAS_Q_SIZE  ); //
    iowrite32(0x1, dma_access_base+DMA0_STAS_Q_LIMIT ); //
    iowrite32((u32)(descr2) | 0xf, dma_access_base+DMA0_STAS_Q_PTR_LO); //Set source status queue
    iowrite32(0x2, dma_access_base+DMA0_STAD_Q_SIZE  ); //
    iowrite32(0x1, dma_access_base+DMA0_STAD_Q_LIMIT ); //
    iowrite32((u32)(descr3) | 0xf, dma_access_base+DMA0_STAD_Q_PTR_LO); //Set destination status queue
    iowrite32(0x0, dma_access_base+DMA0_SRC_Q_NEXT) ; //Reset internal pointers
    iowrite32(0x0, dma_access_base+DMA0_DST_Q_NEXT) ; //Reset internal pointers
    iowrite32(0x0, dma_access_base+DMA0_STAS_Q_NEXT); //Reset internal pointers
    iowrite32(0x0, dma_access_base+DMA0_STAD_Q_NEXT); //Reset internal pointers
    iowrite32(0x0, dma_access_base+DMA0_AXI_INTERRUPT_CONTROL); //Disable AXI interrupt every DMA event

    //Set DMA descriptors
    iowrite32(rotate_be(src_addr), (uint32_t)&descr0[0]); 			//Set source descriptor - address low
    iowrite32(rotate_be(src_addr_high), (uint32_t)&descr0[1]); 		//Set source descriptor - address high
    iowrite32(rotate_be(0x37000000 + size), (uint32_t)&descr0[2]);	//Set source descriptor - size 0x400 from AXI, interrupt, EOP
    iowrite32(0x0, (uint32_t)&descr0[3]); 					        //Set source descriptor - unused fields

    iowrite32(rotate_be(dst_addr), (uint32_t)&descr1[0]);     		//Set destination descriptor - address
    iowrite32(rotate_be(dst_addr_high), (uint32_t)&descr1[1]); 		//Set destination descriptor - address high
    iowrite32(rotate_be(0x31000000 + size), (uint32_t)&descr1[2]);	//Set destination descriptor - size 0x400 to AXI
    iowrite32(0x0, (uint32_t)&descr1[3]);					        //Set destination descriptor - unused fields

    iowrite32(0x0, (uint32_t)&descr2[0]);
    iowrite32(0x0, (uint32_t)&descr2[1]);
    iowrite32(0x0, (uint32_t)&descr2[2]);
    iowrite32(0x0, (uint32_t)&descr2[3]);

    iowrite32(0x0, (uint32_t)&descr3[0]);
    iowrite32(0x0, (uint32_t)&descr3[1]);
    iowrite32(0x0, (uint32_t)&descr3[2]);
    iowrite32(0x0, (uint32_t)&descr3[3]);

    //Start DMA operation
    iowrite32(0x1, dma_access_base+DMA0_DMA_CONTROL);

    //Wait DMA done
    usleep(100);

#else
    uint32_t i;
    uint32_t *src = ( uint32_t *)src_addr;
    uint32_t *dst = ( uint32_t *)dst_addr;

    for (i=0; i<(size >> 2); i++)
    {
        iowrite32(src[i], &dst[i]);
    }
#endif
    return 0;
}

//*****************************************************************************
//  Compare reference and read from DDR0 external memory data
//*****************************************************************************
static int Check_transaction_0 (void)
{
    volatile int j;
    iowrite32 (0x7A557A55, (uint32_t)&log_buf[0]);
    iowrite32 (0, (uint32_t)&log_buf[1]);
    iowrite32 (0, (uint32_t)&log_buf[2]);
    iowrite32 (0, (uint32_t)&log_buf[3]);
    for (j = 0; j < (TEST_DATA_SIZE >> 2); j++)
    {
        if (ioread32 ((uint32_t)&data_buf[j]) != ioread32 ((uint32_t)&mem_buf0[j]))
        {
            iowrite32 (0xDEADBEEF, (uint32_t)&log_buf[0]);
            iowrite32 (j, (uint32_t)&log_buf[1]);
            iowrite32 (ioread32 ((uint32_t)&data_buf[j]), (uint32_t)&log_buf[2]);
            iowrite32 (ioread32 ((uint32_t)&mem_buf0[j]), (uint32_t)&log_buf[3]);
            puts("ERROR: DDR0\n");
            printf("    j = %d  wr = 0x%08x  rd = 0x%08x\n", j, ioread32 ((uint32_t)&data_buf[j]), ioread32 ((uint32_t)&mem_buf0[j]));
            return -1;
        }
    }
    puts("  DDR0 test PASS\n");
	return 0;
}
//*****************************************************************************
//  Compare reference and read from DDR1 external memory data
//*****************************************************************************
static int Check_transaction_1 (void)
{
    volatile int j;
    iowrite32 (0x7A557A55, (uint32_t)&log_buf[4]);
    iowrite32 (0, (uint32_t)&log_buf[5]);
    iowrite32 (0, (uint32_t)&log_buf[6]);
    iowrite32 (0, (uint32_t)&log_buf[7]);
    for (j = 0; j < (TEST_DATA_SIZE >> 2); j++)
    {
        if (ioread32 ((uint32_t)&data_buf[j]) != ioread32 ((uint32_t)&mem_buf1[j]))
        {
            iowrite32 (0xDEADBEEF, (uint32_t)&log_buf[4]);
            iowrite32 (j, (uint32_t)&log_buf[5]);
            iowrite32 (ioread32 ((uint32_t)&data_buf[j]), (uint32_t)&log_buf[6]);
            iowrite32 (ioread32 ((uint32_t)&mem_buf1[j]), (uint32_t)&log_buf[7]);
            puts("ERROR: DDR1\n");
            printf("    j = %d  wr = 0x%08x  rd = 0x%08x\n", j, ioread32 ((uint32_t)&data_buf[j]), ioread32 ((uint32_t)&mem_buf1[j]));
            return -1;
        }
    }
	puts("  DDR1 test PASS\n");
	return 0;
} 

int arch_memory_srtess_post_tests(unsigned long start, unsigned long size)
{
	int err = 0;
	uint32_t ddr0_addr = start;
	uint32_t ddr1_addr = start + (size >> 1);

    clear_destination();
    create_data_array();
    
    //print_capabilities();
    pcie_dma_axi_axi(DMA_ACCESS_BASE, 0, (uint32_t)data_buf,  0, (uint32_t)ddr0_addr, TEST_DATA_SIZE);  //  write data
    //dump_descr();
    pcie_dma_axi_axi(DMA_ACCESS_BASE, 0, (uint32_t)ddr0_addr, 0, (uint32_t)mem_buf0,  TEST_DATA_SIZE);  //  read data
    //dump_descr();
    pcie_dma_axi_axi(DMA_ACCESS_BASE, 0, (uint32_t)data_buf,  0, (uint32_t)ddr1_addr, TEST_DATA_SIZE);  //  write data
    //dump_descr();
    pcie_dma_axi_axi(DMA_ACCESS_BASE, 0, (uint32_t)ddr1_addr, 0, (uint32_t)mem_buf1,  TEST_DATA_SIZE);  //  read data
    //dump_descr();
    
    err += Check_transaction_0();
    err += Check_transaction_1(); 

	return err;
}

#endif /*MAKE_STRESS_TEST*/
