//#define DEBUG

// 1333
//#define SLOW_DOWN 1500

// 1066
#define SLOW_DOWN 1876

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <linux/string.h>
#include <compiler.h>
#include "ddr_impl_h/ddr.h"
#include "ddr_impl_h/ddr_impl.h"
#include "ddr_impl_h/ddr_regs.h"
#include "ddr_impl_h/mivem_assert.h"
//#include "ddr_impl_h/mivem_macro.h"
#include "timer.h"
#include "ddr_impl_h/ppc_476fp_lib_c.h"
#include "ddr_impl_h/ddr/plb6mcif2.h"
#include "ddr_impl_h/ddr/ddr34lmc.h"
#include "ddr_impl_h/ddr/mcif2arb4.h"
#include "ddr_spd.h"
#include "rcm_dimm_params.h"
#include "configs/1888tx018.h"
#include <asm/tlb47x.h>

//static uint8_t __attribute__((section(".EM0.data"),aligned(16))) volatile ddr0_init_array[128] = { 0 };
//static uint8_t __attribute__((section(".EM1.data"),aligned(16))) volatile ddr1_init_array[128] = { 0 };

unsigned int *ddr0_init_array = (void*)0x40000000;
unsigned int *ddr1_init_array = (void*)0x80000000;

#define reg_field(field_right_bit_num_from_ppc_user_manual, value)\
    ((value) << IBM_BIT_INDEX( 32, field_right_bit_num_from_ppc_user_manual ))

#define T_SYS_RDLAT_BC4 0xc                    // get from debug register
#define T_SYS_RDLAT_BL8 0xe

//CONFIGURATION VALUES FOR DDR3-1600 (DEFAULT)
static CrgDdrFreq DDR_FREQ;
static ddr3config ddr_config;

// SUPPORT FUNCTIONS DECLARATIONS
static void ddr_plb6mcif2_init(const uint32_t baseAddr, const uint32_t puaba);
static void ddr_aximcif2_init(const uint32_t baseAddr);
static void ddr_mcif2arb4_init(const uint32_t baseAddr);
static void ddr3phy_init(uint32_t baseAddr, const DdrBurstLength burstLen);
static void ddr3phy_calibrate(const uint32_t baseAddr);
// static void ddr3phy_calibrate_manual_EM0(const unsigned int baseAddr);
// static void ddr3phy_calibrate_manual_EM1(const unsigned int baseAddr);
static void ddr34lmc_dfi_init(const uint32_t baseAddr);
static void ddr_init_main(DdrHlbId hlbId, DdrInitMode initMode, DdrEccMode eccMode, DdrPmMode powerManagementMode, DdrBurstLength burstLength, DdrPartialWriteMode partialWriteMode);
// static void ddr_set_nopatch_config (CrgDdrFreq FreqMode);
static void ddr_set_main_config (CrgDdrFreq FreqMode);

static void crg_ddr_config_freq (uint32_t phy_base_addr, const DdrBurstLength burstLen);
// static void crg_cpu_upd_cken(void);
// static void crg_ddr_upd_cken(void);
//static void crg_cpu_upd_ckdiv(void);
static void crg_ddr_upd_ckdiv(void);
static void crg_remove_writelock(void);
static void crg_set_writelock(void);
static void crg_ddr_reset_ddr_0 (void);
static void crg_ddr_reset_ddr_1 (void);

static volatile uint32_t ioread32(uint32_t const base_addr)
{
    return RCM_DDR_LE32_TO_CPU(*((volatile uint32_t*)(base_addr)));
}

static void iowrite32(uint32_t const value, uint32_t const base_addr)
{
    *((volatile uint32_t*)(base_addr)) = RCM_DDR_CPU_TO_LE32(value);
}

void plb6mcif2_simple_init( uint32_t base_addr, const uint32_t puaba )
{
    plb6mcif2_dcr_write_PLB6MCIF2_INTR_EN(base_addr,
            //enable logging but disable interrupts
              reg_field(15, 0xFFFF)
            | reg_field(31, 0x0000));

    plb6mcif2_dcr_write_PLB6MCIF2_PLBASYNC(base_addr,
            reg_field(1, 0b1)); //PLB clock equals MCIF2 clock

    plb6mcif2_dcr_write_PLB6MCIF2_PLBORD(base_addr,
              reg_field(0, 0b0)  //Raw
            | reg_field(3, 0b010) //Threshold[1:3]
            | reg_field(7, 0b0)   //Threshold[0]
            | reg_field(16, 0b0) //IT_OOO
            | reg_field(18, 0b00) //TOM
            );

    plb6mcif2_dcr_write_PLB6MCIF2_MAXMEM(base_addr,
            reg_field(1, 0b00)); //Set MAXMEM = 8GB

    plb6mcif2_dcr_write_PLB6MCIF2_PUABA(base_addr,
            puaba); //Set PLB upper address base address from I_S_ADDR[0:30]

    plb6mcif2_dcr_write_PLB6MCIF2_MR0CF(base_addr,
            //Set Rank0 base addr
            //M_BA[0:2] = PUABA[28:30]
            //M_BA[3:12] = 0b0000000000
              reg_field(3, (puaba & 0b1110)) | reg_field(12, 0b0000000000)
            | reg_field(19, 0b1001) //Set Rank0 size = 4GB
            | reg_field(31, 0b1)); //Enable Rank0

    plb6mcif2_dcr_write_PLB6MCIF2_MR1CF(base_addr,
            //Set Rank1 base addr
            //M_BA[0:2] = PUABA[28:30]
            //M_BA[3:12] = 0b1000000000
            reg_field(3, (puaba & 0b1110)) | reg_field(12, 0b1000000000)
          | reg_field(19, 0b1001) //Set Rank1 size = 4GB
          | reg_field(31, 0b1)); //Enable Rank1

    plb6mcif2_dcr_write_PLB6MCIF2_MR2CF(base_addr,
          reg_field(31, 0b0)); //Disable Rank2

    plb6mcif2_dcr_write_PLB6MCIF2_MR3CF(base_addr,
            reg_field(31, 0b0)); //Disable Rank3

    const uint32_t Tsnoop = (plb6bc_dcr_read_PLB6BC_TSNOOP(PLB6_BC_BASE) & (0b1111 << 28)) >> 28;
    usleep(1000);
    plb6mcif2_dcr_write_PLB6MCIF2_PLBCFG(base_addr,
            //PLB6MCIF2 enable [31] = 0b1
              reg_field(0, 0b0) //Non-coherent
            | reg_field(4, Tsnoop) //T-snoop
            | reg_field(5, 0b0) //PmstrID_chk_EN
            | reg_field(27, 0b1111) //Parity enables
            | reg_field(30, 0b000) //CG_Enable
            | reg_field(31, 0b1)); //BR_Enable

}


void ddr_ddr34lmc_init(const uint32_t baseAddr,
        DdrInitMode initMode,
        DdrEccMode eccMode,
        DdrPmMode powerManagementMode,
        DdrBurstLength burstLength,
        DdrPartialWriteMode partialWriteMode
);

static void wait_some_time(void)
{
    volatile uint32_t counter;
    for (counter = 0; counter < 100000; counter++)
        ;
}

// static void wait_some_time_1(void)
// {
//     volatile uint32_t counter;
//     for (counter = 0; counter < 1000; counter++)
//         ;
// }

//*************************************************DDR INIT FUNCTIONS*************************************************/

static void init_mmu(void)
{
    asm volatile (
	// initialize MMUCR
	"addis 0, 0, 0x0000   \n\t"  // 0x0000
	"ori   0, 0, 0x0000   \n\t"  // 0x0000
	"mtspr 946, 0          \n\t"  // SPR_MMUCR, 946
	"mtspr 48,  0          \n\t"  // SPR_PID,    48
	// Set up TLB search priority
	"addis 0, 0, 0x9ABC   \n\t"  // 0x1234
	"ori   0, 0, 0xDEF8   \n\t"  // 0x5678  
	//  "addis r0, r0, 0x1234   \n\t"  // 0x1234
	//	"ori   r0, r0, 0x5678   \n\t"  // 0x5678  
	"mtspr 830, 0          \n\t"  // SSPCR, 830
	"mtspr 831, 0          \n\t"  // USPCR, 831
	"mtspr 829, 0          \n\t"  // ISPCR, 829
	"isync                  \n\t"  // 
	"msync                  \n\t"  // 
	:::
    );   
};

static uint32_t read_dcr_reg(uint32_t addr)
{
  uint32_t res;

  asm volatile   (
  "mfdcrx (%0), (%1) \n\t"       // mfdcrx RT, RA
  :"=r"(res)
  :"r"((uint32_t) addr)
  :  
	);
  return res;
};

static void write_dcr_reg(uint32_t addr, uint32_t value)
{

  asm volatile   (
  "mtdcrx (%0), (%1) \n\t"       // mtdcrx RA, RS
  ::"r"((uint32_t) addr), "r"((uint32_t) value)
  :  
	);
};

// static void report_DDR_core_regs(uint32_t baseAddr)
// {
//     volatile uint32_t cntr;
//     volatile uint32_t current_address = baseAddr;
//    
//     printf("  report_DDR_core_regs    baseAddr = 0x%08x\n", baseAddr);
//    
//     for (cntr = 0; cntr < 0xFF; cntr++)
//     {
//         printf("  reg %03x = 0x%08x\n", cntr, read_dcr_reg (current_address));
//         current_address +=1;
//         wait_some_time_1 ();
//     }
// }

static void commutate_plb6(void)
{
    uint32_t dcr_val=0;
    init_mmu();
    
    // PLB6 commutation
    // Set Seg0, Seg1, Seg2 and BC_CR0
    write_dcr_reg(0x80000204, 0x00000010);  // BC_SGD1
    write_dcr_reg(0x80000205, 0x00000010);  // BC_SGD2
    dcr_val = read_dcr_reg(0x80000200);     // BC_CR0
    write_dcr_reg(0x80000200, dcr_val | 0x80000000);  
}


int get_ddr3_spd_bypath(const char * spdpath, ddr3_spd_eeprom_t* spd);
unsigned int rcm_compute_dimm_parameters(const ddr3_spd_eeprom_t *spd,
					 dimm_params_t *pdimm,
					 unsigned int dimm_number);

static int validate_dimm_parameters(dimm_params_t * params, int no)
{
    int result = 0;
    if(params->n_ranks != 1)
    {
        printf("Unsupported DIMM with ranks %d in slot %d\n", params->n_ranks, no);
        result = 1;
    }

    if(params->n_banks_per_sdram_device != 8)
    {
        printf("Unsupported DIMM with number of banks == %d in slot %d\n", params->n_banks_per_sdram_device, no);
        result = 1;
    }

	if (params->primary_sdram_width < 32)
	{
		printf("Unsupported DIMM primary data width == %u in slot %i\n", params->primary_sdram_width, no);
		result = 1;
	}

    return result;
}


static unsigned int getWR(int ticks)
{
    if(ticks > 14)
        return 0;
    if(ticks > 12)
        return 0b111;
    if(ticks > 10)
        return 0b110;
    if(ticks > 8)
        return 0b101;
    
    if(ticks <= 5)
        return 0b001;
    
    return ticks - 4;
}

static unsigned int getCL(int ticks)
{
    // 001-5 010-6 011-7 100-8 101-9 110-10 111-11

    if(ticks > 10)
        return 0b111;   
    if(ticks <= 5)
        return 0b001;
    
    return ticks - 4;
}

static unsigned int getCWLMicron(int CL)
{
    switch(CL)
    {
        case 11:
            return 8;
        case 10:
        case 9:
            return 7;
        case 8:
        case 7:
            return 6;
        case 6:
        case 5:
            return 5;            
    };
    return 8;
}



static unsigned int getRowWidth(int bits)
{
    if(bits <= 17 && bits >= 12)
        return bits - 12;

    printf("Unsupported DDR3 row width %d for DDR\n", bits);    
    return 0;
}


static unsigned int getAddrMode(int bits)
{
    if(bits == 12)
        return 0b100;   
    if(bits == 11)
        return 0b011;
    if(bits == 10)
        return 0b010;

    printf("Unsupported DDR3 addr mode %d for DDR\n", bits);

    return 0;
}


static void ddr_set_config_from_spd(int slowdown, int dimm0_invalid, int dimm1_invalid,
        dimm_params_t* dpar0, dimm_params_t* dpar1)
{
    dimm_params_t* dpar = 0;
    unsigned long time_base = 1250;     // max for DDR1600

    // first detect maximum allowable frequency and reference parameters
    if(!dimm0_invalid)
    {
        printf("Found DDR module in slot 0: %s, size %dMb\n", dpar0->mpart, (int) (dpar0->capacity/1024/1024));
        dpar = dpar0;
        if(dpar0->tckmin_x_ps > time_base)
            time_base = dpar0->tckmin_x_ps;
    }

    if(!dimm1_invalid)
    {
        printf("Found DDR module in slot 1: %s, size %dMb\n", dpar1->mpart, (int) (dpar1->capacity/1024/1024));
        if(dpar1->tckmin_x_ps > time_base || !dpar)
        {
            time_base = dpar1->tckmin_x_ps;
            dpar = dpar1;
        }
    }

    if(slowdown != 0)
    {
        printf("DDR slow down from %ld to %d\n", 1000000/time_base*2, 1000000/SLOW_DOWN*2);
        time_base = SLOW_DOWN;
    }

    if(time_base > 1877)
    {
        DDR_FREQ = DDR3_800;
        time_base = 2500;
    }
    else if(time_base > 1500)
    {
        DDR_FREQ = DDR3_1066;
        time_base = 1876;        
    }
    else if(time_base > 1250)
    {
        DDR_FREQ = DDR3_1333;
        time_base = 1500;
    }
    else
    {
        DDR_FREQ = DDR3_1600;
        time_base = 1250;
    }

    unsigned int cl = (dpar->taa_ps*2)/time_base;
    cl = (cl&1)?cl/2+1:cl/2;            // round up
    while(cl < 11)                      // choose supported cl 
    {
        if(dpar->caslat_x & (1<<cl))
            break;
        cl++;
    }

    ddr_config.CL_PHY =     cl;   // tocheck +1
    ddr_config.CL_MC =      getCL(cl);	//001-5 010-6 011-7 100-8 101-9 110-10 111-11 // CL_MC == SMR0 CL[3:1] ; SMR0 CL[0] == 0;
    ddr_config.CL_CHIP =    getCL(cl);	//001-5 010-6 011-7 100-8 101-9 110-10 111-11 // CL = CL_TIME/tCK

    ddr_config.WR =         getWR(dpar->twr_ps/time_base);  //10    // SMR0, WR = TWR/tCK
    ddr_config.T_REFI =     dpar->refresh_rate_ps/time_base; // refresh interval (in cycles core clock) // SDTR0, T_REFI = 25000000/tI_MC_CLOCK = 25000000/2500 = 10000 (-1 is needed according to MC db)
    ddr_config.T_RFC_XPR =  dpar->trfc_ps/time_base; //tRFC - Refresh to ACT or REF  // SDTR0, T_RFC_XPR = (TRFC_MIN + 10ns)/tI_MC_CLOCK
    ddr_config.T_RCD =      dpar->trcd_ps/time_base; //ACTIVATE to READ/WRITE (in DDR clock cycles) // SDTR2, T_RCD = tRCD/tCK
    ddr_config.T_RP =       dpar->trp_ps/time_base; //PRECHARGE to ACTIVATE (in DDR clock cycles)  // SDTR2, T_RP = tRP/tCK
    ddr_config.T_RC =       dpar->trc_ps/time_base; //ACIVATE to ACIVATE (in DDR clock cycles)     // SDTR2, T_RC = tRC/tCK
    ddr_config.T_RAS =      dpar->tras_ps/time_base; //ACTIVATE to PRECHARGE (in DDR clock cycles)  // SDTR2, T_RAS = tRAS/tCK
    ddr_config.T_WTR =      dpar->twtr_ps/time_base; //delay from start of internal write transaction to internal read command(in DDR clock cycles) // SDTR3, T_WTR = tWTR/tCK
    ddr_config.T_RTP =      dpar->trtp_ps/time_base; // internal read command to precharge delay(in DDR clock cycles) // SDTR3, T_RTP = tRTP/tCK
    ddr_config.T_RRD =      dpar->trrd_ps/time_base;
    ddr_config.T_XSDLL =    32;	    //exit self refresh and DLL lock delay (in x16 clocks) // SDTR3, TXSDLL = (tXSRD/PHY_to_MC_clock_ratio)/16

    ddr_config.T_ROW_WIDTH = getRowWidth(dpar->n_row_addr);
    ddr_config.T_ADDR_MODE = getAddrMode(dpar->n_col_addr);

    ddr_config.T_MOD =      12;	    //MRS command update delay (in DDR clock cycles) // SDTR4, T_MOD = tMOD/tCKtCK   
    ddr_config.AL_PHY =     ddr_config.CL_PHY-2;
    ddr_config.AL_MC =      getCL(ddr_config.AL_PHY)-3;
    ddr_config.AL_CHIP =    ddr_config.AL_MC;

    ddr_config.CWL_PHY = getCWLMicron(ddr_config.CL_PHY);       //? to check

    // temporary workaround for memory CWLs
    if(!strncmp("CT",dpar->mpart, 2 ))
    {
        ddr_config.CWL_CHIP = getCL(ddr_config.CWL_PHY) -1;   // crucial
        ddr_config.CWL_MC = ddr_config.CWL_CHIP -1;           // crucial
    }
    else
    {
        ddr_config.CWL_CHIP = getCL(ddr_config.CWL_PHY);        // kingston transcend
        ddr_config.CWL_MC = ddr_config.CWL_CHIP;                // kingston transcend
    }

    ddr_config.T_RDDATA_EN_BC4 = ddr_config.CL_PHY + ddr_config.AL_PHY - 2 ;	//? // RL-3 = CL + AL - 3
    ddr_config.T_RDDATA_EN_BL8 = ddr_config.T_RDDATA_EN_BC4 - 2;	//? to check

}

#ifdef DEBUG
static void dump_ddr_config(void)
{
    printf("Current ddr_config:\n");
    printf("uint32_t T_RDDATA_EN_BC4 =  %d\n", ddr_config.T_RDDATA_EN_BC4);
    printf("uint32_t T_RDDATA_EN_BL8 =  %d\n", ddr_config.T_RDDATA_EN_BL8);
    printf("uint8_t  CL_PHY =           %d\n", ddr_config.CL_PHY);
    printf("uint8_t  CL_MC =            %d\n", ddr_config.CL_MC);
    printf("uint8_t  CL_CHIP =          %d\n", ddr_config.CL_CHIP);
    printf("uint8_t  CWL_PHY =          %d\n", ddr_config.CWL_PHY);
    printf("uint8_t  CWL_MC =           %d\n", ddr_config.CWL_MC);
    printf("uint8_t  CWL_CHIP =         %d\n", ddr_config.CWL_CHIP);
    printf("uint8_t  AL_PHY =           %d\n", ddr_config.AL_PHY);
    printf("uint8_t  AL_MC =            %d\n", ddr_config.AL_MC);
    printf("uint8_t  AL_CHIP =          %d\n", ddr_config.AL_CHIP);
    printf("uint32_t WR =               %d\n", ddr_config.WR);
    printf("uint32_t T_REFI =           %d\n", ddr_config.T_REFI);
    printf("uint32_t T_RFC_XPR =        %d\n", ddr_config.T_RFC_XPR);
    printf("uint32_t T_RCD =            %d\n", ddr_config.T_RCD);
    printf("uint32_t T_RRD =            %d\n", ddr_config.T_RRD);
    printf("uint32_t T_RP =             %d\n", ddr_config.T_RP);
    printf("uint32_t T_RC =             %d\n", ddr_config.T_RC);
    printf("uint32_t T_RAS =            %d\n", ddr_config.T_RAS);
    printf("uint32_t T_WTR =            %d\n", ddr_config.T_WTR);
    printf("uint32_t T_RTP =            %d\n", ddr_config.T_RTP);
    printf("uint32_t T_XSDLL =          %d\n", ddr_config.T_XSDLL);
    printf("uint32_t T_MOD =            %d\n", ddr_config.T_MOD);
    printf("uint32_t T_ROW_WIDTH =      %d\n", ddr_config.T_ROW_WIDTH);
    printf("uint32_t T_ADDR_MODE =      %d\n", ddr_config.T_ADDR_MODE);
}
#endif

void ddr_tlbs_init(void)
{
    phys_size_t ddr_size = 0;
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
		ddr_size += dpar0.capacity * 32 / dpar0.primary_sdram_width;
	}

	if (!dimm1_params_invalid)
	{
		ddr_size += dpar1.capacity * 32 / dpar0.primary_sdram_width;
	}
#else
	#ifdef CONFIG_1888TX018_DDR_EM0_SIZE
		ddr_size += ((phys_size_t)CONFIG_1888TX018_DDR_EM0_SIZE) * SZ_1M;
	#endif

	#ifdef CONFIG_1888TX018_DDR_EM1_SIZE
		ddr_size += ((phys_size_t)CONFIG_1888TX018_DDR_EM1_SIZE) * SZ_1M;
	#endif
#endif

    phys_size_t p_size = (((ddr_size) > CONFIG_MAX_MEM_MAPPED) ? CONFIG_MAX_MEM_MAPPED : (ddr_size));
    
    uint32_t ddr0_base, ddr1_base, offset;
    tlb_size_id tlb_sid = tlb47x_get_tlb_sid_by_size(p_size >> 1);
    if(tlb_sid == TLBSID_ERR) tlb_sid = TLBSID_1G;

    offset = tlb47x_get_tlb_sid_size(tlb_sid);
    ddr0_base = CONFIG_SYS_DDR_BASE;
    ddr1_base = CONFIG_SYS_DDR_BASE + offset;

    ddr0_init_array = (void*)ddr0_base;
    ddr1_init_array = (void*)ddr1_base;

    //                  physical,                     logical,  il1i, il1d, wimg,  size,      umode,        smode,       window
    tlb47x_map_entry(CONFIG_SYS_INIT_DDR0_ADDR_PHYS, ddr0_base,  0,    0,    4,   tlb_sid, TLB_MODE_RWX,  TLB_MODE_RWX, MEM_WINDOW_SHARED);
    tlb47x_map_entry(CONFIG_SYS_INIT_DDR1_ADDR_PHYS, ddr1_base,  0,    0,    4,   tlb_sid, TLB_MODE_RWX,  TLB_MODE_RWX, MEM_WINDOW_SHARED);
}

void ddr_init (int slowdown)
{
    int dimm0_params_invalid = 1;
    int dimm1_params_invalid = 1;
    dimm_params_t dpar0, dpar1;
    // int res;
    DdrHlbId hlbId = DdrHlbId_Default;
    DdrBurstLength bl = DdrBurstLength_Default;

#ifdef CONFIG_SPD_EEPROM
    ddr3_spd_eeprom_t spd;

    if(!get_ddr3_spd_bypath("spd0-path", &spd) && 
       !rcm_compute_dimm_parameters(&spd, &dpar0, 0))
            dimm0_params_invalid = validate_dimm_parameters(&dpar0, 0);
    if(!get_ddr3_spd_bypath("spd1-path", &spd) && 
       !rcm_compute_dimm_parameters(&spd, &dpar1, 1))
            dimm1_params_invalid = validate_dimm_parameters(&dpar1, 1);

#endif

    commutate_plb6();
    ddr_tlbs_init();

    //                  physical,       logical,  il1i, il1d, wimg,   size,        umode,        smode,       window
    tlb47x_map_entry(0x0000000000ULL, 0x40000000,  0,    0,    4,   TLBSID_1G, TLB_MODE_RWX,  TLB_MODE_RWX, MEM_WINDOW_0);
    tlb47x_map_entry(0x0040000000ULL, 0x80000000,  0,    0,    4,   TLBSID_1G, TLB_MODE_RWX,  TLB_MODE_RWX, MEM_WINDOW_0);
    tlb47x_map_entry(0x0200000000ULL, 0x40000000,  0,    0,    4,   TLBSID_1G, TLB_MODE_RWX,  TLB_MODE_RWX, MEM_WINDOW_1);
    tlb47x_map_entry(0x0240000000ULL, 0x80000000,  0,    0,    4,   TLBSID_1G, TLB_MODE_RWX,  TLB_MODE_RWX, MEM_WINDOW_1);

    if((slowdown==2) || (dimm0_params_invalid && dimm1_params_invalid))
    {
        DDR_FREQ = DDR3_1333; /* Assume 1033 by default */
        #ifdef CONFIG_1888TX018_DDR_1600
        DDR_FREQ = DDR3_1600;
        #endif
        #ifdef CONFIG_1888TX018_DDR_1333
        DDR_FREQ = DDR3_1333;
        #endif
        #ifdef CONFIG_1888TX018_DDR_1060
        DDR_FREQ = DDR3_1060;
        #endif
        #ifdef CONFIG_1888TX018_DDR_800
        DDR_FREQ = DDR3_800;
        #endif

        ddr_set_main_config (DDR_FREQ);
    }
    else
    {
        hlbId = 0;
        bl = DdrBurstLength_4;
        if(!dimm0_params_invalid)
        {
            hlbId |= DdrHlbId_Em0;
            if(dpar0.burst_lengths_bitmask > 4)   /* BL=4 bit 2, BL=8 = bit 3 */
                bl = DdrBurstLength_8;
        }
        if(!dimm1_params_invalid)
        {
            hlbId |= DdrHlbId_Em1;
            if(dpar1.burst_lengths_bitmask > 4)   /* BL=4 bit 2, BL=8 = bit 3 */
                bl = DdrBurstLength_8;
        }
        
        ddr_set_config_from_spd(slowdown, dimm0_params_invalid, dimm1_params_invalid,
            &dpar0, &dpar1);
    }

#ifdef DEBUG
    dump_ddr_config();
#endif    

    ddr_init_main(
            hlbId,
            DdrInitMode_Default,
            DdrEccMode_Default,
            DdrPmMode_Default,
            bl,
            DdrPartialWriteMode_Default);            
}

static void ddr_init_main(
        DdrHlbId hlbId,
        DdrInitMode initMode,
        DdrEccMode eccMode,
        DdrPmMode powerManagementMode,
        DdrBurstLength burstLength,
        DdrPartialWriteMode partialWriteMode
        )
{
    printf ("Start init DDR3\n");

    uint8_t T_SYS_RDLAT_configured;
    uint8_t T_SYS_RDLAT_measured;
    int i;

    int reinit_emi_0;
    int reinit_emi_1;
    
    reinit_emi_0 = 1;
    reinit_emi_1 = 1;
    
    // todo: understand what wrong with single init
    // tweak for a while    
    for(i = 0; i < 6; i++)
    {
        
        if (reinit_emi_0) {
            crg_ddr_reset_ddr_0 ();
            
            if (hlbId & DdrHlbId_Em0)
            {
                printf("Init EM0 PLB6MCIF2\n");
                ddr_plb6mcif2_init(EM0_PLB6MCIF2_DCR, 0x00000000);

                printf("Init EM0 AXIMCIF2\n");
                ddr_aximcif2_init(EM0_AXIMCIF2_DCR);

                printf("Init EM0 MCIF2ARB4\n");
                ddr_mcif2arb4_init(EM0_MCIF2ARB_DCR);

                
                //printf("Init EM0 DDR3PHY\n");
                //ddr3phy_init(EM0_PHY_DCR, burstLength);
                crg_ddr_config_freq (EM0_PHY__, burstLength); //& phy_init

                printf("Init EM0 DDR34LMC\n");
                ddr_ddr34lmc_init(EM0_DDR3LMC_DCR,
                        initMode,
                        eccMode,
                        powerManagementMode,
                        burstLength,
                        partialWriteMode);


                printf("Calibrate EM0 DDR3PHY\n");
                ddr3phy_calibrate(EM0_PHY__);
                
                // printf("Calibrate (manual) EM0 DDR3PHY\n");
                // ddr3phy_calibrate_manual_EM0(EM0_PHY__);
            }

            if (hlbId & DdrHlbId_Em0)
            {
                T_SYS_RDLAT_configured = burstLength == DdrBurstLength_4 ? T_SYS_RDLAT_BC4 : T_SYS_RDLAT_BL8;
                asm volatile (
                    "stmw 0, 0(%0)\n\t"
                    ::"r"(ddr0_init_array)//woro
                );
                msync();
                //TODO: need to invalidate read address to avoid cache hit.
                dcbi(ddr0_init_array);//woro
                msync();
                (void)(MEM32(ddr0_init_array)); //generate read transaction woro
                printf("Configured EM0 T_SYS_RDLAT=0x%08x\n",(T_SYS_RDLAT_configured));
                T_SYS_RDLAT_measured = ddr_get_T_SYS_RDLAT(EM0_DDR3LMC_DCR);
                printf("Measured EM0 T_SYS_RDLAT=0x%08x\n",T_SYS_RDLAT_measured);

                if (T_SYS_RDLAT_measured != T_SYS_RDLAT_configured)
                {
                    printf("EM0 T_SYS_RDLAT reconfiguration. Reason: measured != configured.\n");
                    printf("Enter self-refresh mode\n");
                    ddr_enter_self_refresh_mode(EM0_DDR3LMC_DCR);

                    T_SYS_RDLAT_configured = T_SYS_RDLAT_measured;
                    printf("Set new EM0 T_SYS_RDLAT = 0x%08x\n",T_SYS_RDLAT_configured);
                    uint32_t SDTR4_reg = ddr34lmc_dcr_read_DDR34LMC_SDTR4(EM0_DDR3LMC_DCR);
                    SDTR4_reg &= ~reg_field(15, 0b111111);
                    ddr34lmc_dcr_write_DDR34LMC_SDTR4(EM0_DDR3LMC_DCR,
                            SDTR4_reg | reg_field(15, T_SYS_RDLAT_configured));

                    printf("Exit self-refresh mode\n");
                    ddr_exit_self_refresh_mode(EM0_DDR3LMC_DCR);

                    //TODO: need to invalidate read address to avoid cache hit.
                    dcbi(ddr0_init_array); //woro
                    msync();
                    //(void)(MEM32(EM0_BASE + 0x100)); //generate new read transaction. New address to avoid cache hit.
                    (void)(MEM32(ddr0_init_array)); //generate read transaction woro
                    T_SYS_RDLAT_measured = ddr_get_T_SYS_RDLAT(EM0_DDR3LMC_DCR);
                    printf("Measured EM0 T_SYS_RDLAT=0x%08x\n",T_SYS_RDLAT_measured);
                    TEST_ASSERT(T_SYS_RDLAT_measured == T_SYS_RDLAT_configured, "EM0 DDR34LMC configuration error");
                }

            }
            
            iowrite32 (0x12345678, 0x40000000);
            if (ioread32 (0x40000000) != 0x12345678) {
                reinit_emi_0 = 1;
                printf("EM0 init error: 0x12345678 != 0x%x\n", ioread32 (0x40000000));
            }
            else {
                reinit_emi_0 = 0;
                printf("---> EM0 init OK (very simple test): 0x12345678 == 0x%x\n", ioread32 (0x40000000));
            }
        }
        

        //EM1
        if (reinit_emi_1) {
            crg_ddr_reset_ddr_1 ();
            
            if (hlbId & DdrHlbId_Em1)
            {
                printf("Init EM1 PLB6MCIF2\n");
                ddr_plb6mcif2_init(EM1_PLB6MCIF2_DCR, 0x00000002);

                printf("Init EM1 AXIMCIF2\n");
                ddr_aximcif2_init(EM1_AXIMCIF2_DCR);

                printf("Init EM1 MCIF2ARB4\n");
                ddr_mcif2arb4_init(EM1_MCIF2ARB_DCR);

                //printf("Init EM1 DDR3PHY\n");
                //ddr3phy_init(EM1_PHY_DCR, burstLength);

                if (hlbId & DdrHlbId_Em0) //some magic...
                {
                    printf("Init DDR3PHY\n");
                    ddr3phy_init(EM1_PHY__, burstLength);
                }
                else
                    crg_ddr_config_freq (EM1_PHY__, burstLength);  //& phy_init


                printf("Init EM1 DDR34LMC\n");
                ddr_ddr34lmc_init(EM1_DDR3LMC_DCR,
                        initMode,
                        eccMode,
                        powerManagementMode,
                        burstLength,
                        partialWriteMode);

                printf("Calibrate EM1 DDR3PHY\n");
                ddr3phy_calibrate(EM1_PHY__);
                
                // printf("Calibrate (manual) EM1 DDR3PHY\n");
                // ddr3phy_calibrate_manual_EM1(EM1_PHY__);
                
                if (hlbId & DdrHlbId_Em1)
                {
                    T_SYS_RDLAT_configured = burstLength == DdrBurstLength_4 ? T_SYS_RDLAT_BC4 : T_SYS_RDLAT_BL8;
                    asm volatile (
                        "stmw 0, 0(%0)\n\t"
                        ::"r"(ddr1_init_array)
                    );
                    msync();
                    dcbi(ddr1_init_array);
                    msync();
                    (void)(MEM32((uint32_t)ddr1_init_array)); //generate read transaction
                    printf("Configured EM1 T_SYS_RDLAT=0x%08x\n",(T_SYS_RDLAT_configured));
                    T_SYS_RDLAT_measured = ddr_get_T_SYS_RDLAT(EM1_DDR3LMC_DCR);
                    printf("Measured EM1 T_SYS_RDLAT=0x%08x\n",T_SYS_RDLAT_measured);

                    if (T_SYS_RDLAT_measured != T_SYS_RDLAT_configured)
                    {
                        printf("EM1 T_SYS_RDLAT reconfiguration. Reason: measured != configured.\n");
                        printf("Enter self-refresh mode\n");
                        ddr_enter_self_refresh_mode(EM1_DDR3LMC_DCR);

                        T_SYS_RDLAT_configured = T_SYS_RDLAT_measured;
                        printf("Set new EM1 T_SYS_RDLAT = 0x%08x\n", T_SYS_RDLAT_configured);
                        uint32_t SDTR4_reg = ddr34lmc_dcr_read_DDR34LMC_SDTR4(EM1_DDR3LMC_DCR);
                        SDTR4_reg &= ~reg_field(15, 0b111111);
                        ddr34lmc_dcr_write_DDR34LMC_SDTR4(EM1_DDR3LMC_DCR,
                                SDTR4_reg | reg_field(15, T_SYS_RDLAT_configured));

                        printf("Exit self-refresh mode\n");
                        ddr_exit_self_refresh_mode(EM1_DDR3LMC_DCR);

                        //TODO: need to invalidate read address to avoid cache hit.
                        dcbi(ddr1_init_array);
                        msync();
                        //(void)(MEM32(EM1_BASE + 0x100)); //generate new read transaction. New address to avoid cache hit.
                        (void)(MEM32((uint32_t)ddr1_init_array)); //generate read transaction
                        T_SYS_RDLAT_measured = ddr_get_T_SYS_RDLAT(EM1_DDR3LMC_DCR);
                        printf("Measured EM1 T_SYS_RDLAT=0x%08x\n", T_SYS_RDLAT_measured);
                        TEST_ASSERT(T_SYS_RDLAT_measured == T_SYS_RDLAT_configured, "EM1 DDR34LMC configuration error");
                    }

                }
            }
            
            iowrite32 (0x12345678, 0x80000000);
            if (ioread32 (0x80000000) != 0x12345678) {
                reinit_emi_1 = 1;
                printf("EM1 init error: 0x12345678 != 0x%x\n", ioread32 (0x80000000));
            }
            else {
                reinit_emi_1 = 0;
                printf("---> EM1 init OK (very simple test): 0x12345678 == 0x%x\n", ioread32 (0x80000000));
            }
        }
    }
        
	printf("BESR_em0 = 0x%08x\n", plb6mcif2_dcr_read_PLB6MCIF2_BESR(EM0_PLB6MCIF2_DCR));
    printf("BESR_em1 = 0x%08x\n", plb6mcif2_dcr_read_PLB6MCIF2_BESR(EM1_PLB6MCIF2_DCR));
    //-------------------
}

static void ddr_plb6mcif2_init(const uint32_t baseAddr, const uint32_t puaba)
{
    plb6mcif2_simple_init(baseAddr,puaba);
}

static void ddr_aximcif2_init(const uint32_t baseAddr)
{
    //Software must properly configure the AXIASYNC, AXICFG, AXIRNK0, AXIRNK1, AXIRNK2 and
    //AXIRNK3 registers before starting read/write transfers.

    //Mask errors, do not generate interrupt
    aximcif2_dcr_write_AXIMCIF2_AXIERRMSK(baseAddr, 0xff000000);

    aximcif2_dcr_write_AXIMCIF2_AXIASYNC(baseAddr,
              (0b000 << 25) //AXINUM,
            | (0b0000000 << 18) //MCIFNUM
            | (0b0000 << 8) //RDRDY_TUNE
            | (0b0 << 5) //DCR_CLK_MN
            | (0b00000 << 0) //DCR_CLK_RATIO
    );

    aximcif2_dcr_write_AXIMCIF2_AXICFG(baseAddr,
              (0b0 << 15) //IT_OOO,
            | (0b00 << 13) //TOM
            | (0b0 << 1) //EQ_ID
            | (0b0 << 0) // EXM_ID
    );

    //EM0 and EM1 use 4GB x 2rank external DDR memory configuration
    uint32_t R_RA = 0;
    uint32_t R_RS = 0;
    uint32_t R_RO = 0;
    switch(baseAddr)
    {
    case EM0_AXIMCIF2_DCR:
        {
            //EM0 R_RA - AXI Address Bits [35:24]
            //EM0 AXI address 0x0_40000000 is mapped to the starting 1GB of EM0 rank0
            R_RA = 0x040;
            R_RS = 0b0110; //1GB
            R_RO = 0; //zero offset
        }
        break;
    case EM1_AXIMCIF2_DCR:
        {
            //EM1 R_RA - AXI Address Bits [35:24]
            //EM1 AXI address 0x0_80000000 is mapped to the starting 2GB of EM1 rank0
            R_RA = 0x080;
            R_RS = 0b0111; break; //2GB
            R_RO = 0; //zero offset
        }
        break;
    default: TEST_ASSERT(0, "Unexpected base address");
    }

    aximcif2_dcr_write_AXIMCIF2_MR0CF(baseAddr,
              ((R_RA & 0xfff) << 20)
            | ((R_RS & 0xf) << 16)
            | ((R_RO & 0b111111111111111) << 1)
            | (0b1 << 0) //R_RV
    );

    //disable other ranks as not accessible in AXI address space
    aximcif2_dcr_write_AXIMCIF2_MR1CF(baseAddr, 0);
    aximcif2_dcr_write_AXIMCIF2_MR2CF(baseAddr, 0);
    aximcif2_dcr_write_AXIMCIF2_MR3CF(baseAddr, 0);
}
static void ddr_mcif2arb4_init(const uint32_t baseAddr)
{
    mcif2arb4_dcr_write_MCIF2ARB4_MACR(baseAddr,
              (0b0000 << 28) //MPM
            | (0b1 << 27) //WD_DLY
            | (0b00 << 8)); //ORDER_ATT
}
static void ddr3phy_init(uint32_t baseAddr, const DdrBurstLength burstLen)
{
    uint32_t reg = 0;
    reg = ddr3phy_read_DDR3PHY_PHYREG00(baseAddr);
    ddr3phy_write_DDR3PHY_PHYREG00(baseAddr,
            reg & ~(0b11 << 2)); // Soft reset enter

    wait_some_time ();
    
    reg = ddr3phy_read_DDR3PHY_PHYREG00(baseAddr);
    ddr3phy_write_DDR3PHY_PHYREG00(baseAddr,
            reg | (0b11 << 2)); // Soft reset exit

    wait_some_time ();
    
    reg = ddr3phy_read_DDR3PHY_PHYREG01(baseAddr);
    reg &= ~0b111;
    ddr3phy_write_DDR3PHY_PHYREG01(baseAddr,
            (burstLen == DdrBurstLength_4) ? (reg | 0b000) : (reg | 0b100) ); // Set DDR3 mode

    printf("phyreg_cl_phy = 0x%08x\n", DDR3PHY_PHYREG0b + baseAddr);
    reg = ddr3phy_read_DDR3PHY_PHYREG0b(baseAddr);
    reg &= ~0xff;
    ddr3phy_write_DDR3PHY_PHYREG0b(baseAddr,
            reg | (ddr_config.CL_PHY << 4) | (ddr_config.AL_PHY << 0)); // Set CL, AL

    reg = ddr3phy_read_DDR3PHY_PHYREG0c(baseAddr);
    reg &= ~0xf;
    ddr3phy_write_DDR3PHY_PHYREG0c(baseAddr,
            reg | (ddr_config.CWL_PHY << 0)); // CWL = 8

    
    //-------------------------------------------------------------------------
    //  odt configure bypass mode Off (?)
    //  Made no sense
    // iowrite32 (0x0, baseAddr + (0x3 << 2));
    // iowrite32 (0x0, baseAddr + (0x4 << 2));
    //-------------------------------------------------------------------------
    
    //2. Configure DDR PHY PLL output to provide stable clock to SDRAM
    //Setup PHY internal PLL
    //PHY.ddr_refclk = 400MHz
    //To generate 800MHz on PHY.CK0 we have to setup PLL on 1600MHz using M/N dividers
    //where M - PLL feedback divider (= 4 for 400MHz)
    //      N - PLL pre-divider (= 1 for 400MHz)

    ddr3phy_write_DDR3PHY_PHYREGEE(baseAddr,0x2); // set PLL Pre-divide

    switch (DDR_FREQ)
    {
      case DDR3_1066:
          ddr3phy_write_DDR3PHY_PHYREGEC(baseAddr, 0x20); // set PLL Feedback divide
          break;
      case DDR3_1333:
          ddr3phy_write_DDR3PHY_PHYREGEC(baseAddr, 0x28); // set PLL Feedback divide
          break;
      case DDR3_1600:
          ddr3phy_write_DDR3PHY_PHYREGEC(baseAddr, 0x20); // set PLL Feedback divide
          break;
      default: TEST_ASSERT(0,"Invalid DDR FreqMode");
    }

    ddr3phy_write_DDR3PHY_PHYREGED(baseAddr, 0x18); //PLL clock out enable
    
    printf("    _dbg_0\n");
    while (((ioread32(SCTL_BASE + SCTL_PLL_STATE)>>2)&1) != 1) //#ev Ожидаем установления частоты
    {
    }
    // usleep(5); //let's wait 5us
    printf("    _dbg_1\n");
    
    //usleep(4);

    //  select DDR PHY internal PLL
    ddr3phy_write_DDR3PHY_PHYREGEF(baseAddr, 0x1);

}
static void ddr34lmc_dfi_init(const uint32_t baseAddr)
{
    //DFI init must be done before step 7. enabling dfi_cke.

    printf("DFI_INIT_START\n");
    //9. DFI init.

    uint32_t reg = 0;

    reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            reg | reg_field(19, 0b1)); //MCOPT2[19] = 0b1 DFI_INIT_START

    //PHY spec says we must wait for 2500ps*2000dfi_clk1x = 5us
    //let's wait a bit more
    usleep(10);

    //check DFI_INIT_COMPLETE
    reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
    TEST_ASSERT(reg & reg_field(3, 0b1), "DFI_INIT_COMPLETE timeout");

    //MCOPT2[DFI_INIT_START] must stay LOW. See PHY spec notes.
    printf("DFI_INIT_COMPLETE\n");
}

void sdram_write_leveling_mode_on (const uint32_t baseAddr)
{
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ0(baseAddr,
    //Wait = TMRD*tCK/tI_MC_CLOCK = 4*1250/2500 = 2
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT                EN_MULTI_RANK_SELECT      RANK
            reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 0) |         reg_field(31, 0b0001));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD0(baseAddr,	//#ev bit30, 29: 00->11
    //See JEDEC 3.4.3 Mode Register MR1
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13        A12                  A11                  A10                A9                   A8                 A7                   A6                   A5                   A4 A3                						 A2                   A1                   A0
    //Value              0                      0                       0                   0                   0                    1                    0   0   0          Qoff                 TDQS                 0                  Rtt_Nom[2]           0                  Level                Rtt_Nom[1]           DIC[1]               AL                    						Rtt_Nom[0]           DIC[0]               DLL
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:2]          ADDR[3]              ADDR[4]              ADDR[5]            ADDR[6]              ADDR[7]            ADDR[8]              ADDR[9]              ADDR[10]             ADDR[11:12]           						ADDR[13]             ADDR[14]             ADDR[15]
                         reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b0) | reg_field(11, 0b1) | reg_field(18, 0) | reg_field(19, 0b0) | reg_field(20, 0b0) | reg_field(21, 0) | reg_field(22, 0b0) | reg_field(23, 0) | reg_field(24, 0b1) | reg_field(25, 0b1) | reg_field(26, 0b0) | reg_field(28, ddr_config.AL_CHIP) | reg_field(29, 0b0) | reg_field(30, 0b1) | reg_field(31, 0b0));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ1(baseAddr,
    //Wait = TMRD*tCK/tI_MC_CLOCK = 4*1250/2500 = 2
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT                EN_MULTI_RANK_SELECT      RANK
            reg_field(0, 0b1) | reg_field(15, 2) | reg_field(27, 0) |         reg_field(31, 0b0010));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD1(baseAddr,	//#ev bit30, 29: 00->11
    //See JEDEC 3.4.3 Mode Register MR1
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13        A12                  A11                  A10                A9                   A8                 A7                   A6                   A5                   A4 A3                						 A2                   A1                   A0
    //Value              0                      0                       0                   0                   0                    1                    0   0   0          Qoff                 TDQS                 0                  Rtt_Nom[2]           0                  Level                Rtt_Nom[1]           DIC[1]               AL                    						Rtt_Nom[0]           DIC[0]               DLL
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:2]          ADDR[3]              ADDR[4]              ADDR[5]            ADDR[6]              ADDR[7]            ADDR[8]              ADDR[9]              ADDR[10]             ADDR[11:12]           						ADDR[13]             ADDR[14]             ADDR[15]
                         reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b0) | reg_field(11, 0b1) | reg_field(18, 0) | reg_field(19, 0b1) | reg_field(20, 0b0) | reg_field(21, 0) | reg_field(22, 0b1) | reg_field(23, 0) | reg_field(24, 0b0) | reg_field(25, 0b0) | reg_field(26, 0b0) | reg_field(28, ddr_config.AL_CHIP) | reg_field(29, 0b1) | reg_field(30, 0b1) | reg_field(31, 0b0));

    // ddr34lmc_dcr_write_DDR34LMC_INITSEQ1(baseAddr, 0);
    // ddr34lmc_dcr_write_DDR34LMC_INITCMD1(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ2(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD2(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ3(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD3(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ4(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD4(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ5(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD5(baseAddr, 0);
    
    
    
    uint32_t mcopt2_reg;
    
    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            (mcopt2_reg & ~(reg_field(0, 0b1))) //MCOPT2[0] = 0b0 SELF_REF_EN
            | reg_field(2, 0b1)); //MCOPT2[2] = 0b1 INIT_START

    const uint32_t INIT_COMPLETE_TIMEOUT = 1000;
    uint32_t time = 0;
    uint32_t mcstat_reg = 0;

    do
    {
        mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
        if (mcstat_reg & reg_field(0, 0b1)) // INIT_COMPLETE
            break;
    }
    while (++time != INIT_COMPLETE_TIMEOUT);

    TEST_ASSERT(time != INIT_COMPLETE_TIMEOUT, "sdram_write_leveling_mode_on DDR INIT_COMPLETE_TIMEOUT");
}

void sdram_write_leveling_mode_off (const uint32_t baseAddr)
{
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ0(baseAddr,
    //Wait = TMRD*tCK/tI_MC_CLOCK = 4*1250/2500 = 2
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT                EN_MULTI_RANK_SELECT      RANK
            reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 0) |         reg_field(31, 0b1111));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD0(baseAddr,	//#ev bit30, 29: 00->11
    //See JEDEC 3.4.3 Mode Register MR1
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13        A12                  A11                  A10                A9                   A8                 A7                   A6                   A5                   A4 A3                						 A2                   A1                   A0
    //Value              0                      0                       0                   0                   0                    1                    0   0   0          Qoff                 TDQS                 0                  Rtt_Nom[2]           0                  Level                Rtt_Nom[1]           DIC[1]               AL                    						Rtt_Nom[0]           DIC[0]               DLL
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:2]          ADDR[3]              ADDR[4]              ADDR[5]            ADDR[6]              ADDR[7]            ADDR[8]              ADDR[9]              ADDR[10]             ADDR[11:12]           						ADDR[13]             ADDR[14]             ADDR[15]
                         reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b0) | reg_field(11, 0b1) | reg_field(18, 0) | reg_field(19, 0b0) | reg_field(20, 0b0) | reg_field(21, 0) | reg_field(22, 0b0) | reg_field(23, 0) | reg_field(24, 0b0) | reg_field(25, 0b1) | reg_field(26, 0b0) | reg_field(28, ddr_config.AL_CHIP) | reg_field(29, 0b1) | reg_field(30, 0b1) | reg_field(31, 0b0));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ1(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD1(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ2(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD2(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ3(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD3(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ4(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD4(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ5(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD5(baseAddr, 0);
    
    
    
    uint32_t mcopt2_reg;
    
    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            (mcopt2_reg & ~(reg_field(0, 0b1))) //MCOPT2[0] = 0b0 SELF_REF_EN
            | reg_field(2, 0b1)); //MCOPT2[2] = 0b1 INIT_START

    const uint32_t INIT_COMPLETE_TIMEOUT = 1000;
    uint32_t time = 0;
    uint32_t mcstat_reg = 0;

    do
    {
        mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
        if (mcstat_reg & reg_field(0, 0b1)) // INIT_COMPLETE
            break;
    }
    while (++time != INIT_COMPLETE_TIMEOUT);

    TEST_ASSERT(time != INIT_COMPLETE_TIMEOUT, "sdram_write_leveling_mode_off DDR INIT_COMPLETE_TIMEOUT");
}

void sdram_write_leveling_cs1_off (const uint32_t baseAddr)
{
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ0(baseAddr,
    //Wait = TMRD*tCK/tI_MC_CLOCK = 4*1250/2500 = 2
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT                EN_MULTI_RANK_SELECT      RANK
            reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 0) |         reg_field(31, 0b0010));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD0(baseAddr,	//#ev bit30, 29: 00->11
    //See JEDEC 3.4.3 Mode Register MR1
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13        A12                  A11                  A10                A9                   A8                 A7                   A6                   A5                   A4 A3                						 A2                   A1                   A0
    //Value              0                      0                       0                   0                   0                    1                    0   0   0          Qoff                 TDQS                 0                  Rtt_Nom[2]           0                  Level                Rtt_Nom[1]           DIC[1]               AL                    						Rtt_Nom[0]           DIC[0]               DLL
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:2]          ADDR[3]              ADDR[4]              ADDR[5]            ADDR[6]              ADDR[7]            ADDR[8]              ADDR[9]              ADDR[10]             ADDR[11:12]           						ADDR[13]             ADDR[14]             ADDR[15]
                         reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b0) | reg_field(11, 0b1) | reg_field(18, 0) | reg_field(19, 0b1) | reg_field(20, 0b0) | reg_field(21, 0) | reg_field(22, 0b0) | reg_field(23, 0) | reg_field(24, 0b0) | reg_field(25, 0b1) | reg_field(26, 0b0) | reg_field(28, ddr_config.AL_CHIP) | reg_field(29, 0b1) | reg_field(30, 0b1) | reg_field(31, 0b0));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ1(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD1(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ2(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD2(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ3(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD3(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ4(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD4(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ5(baseAddr, 0);
    ddr34lmc_dcr_write_DDR34LMC_INITCMD5(baseAddr, 0);
    
    
    
    uint32_t mcopt2_reg;
    
    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            (mcopt2_reg & ~(reg_field(0, 0b1))) //MCOPT2[0] = 0b0 SELF_REF_EN
            | reg_field(2, 0b1)); //MCOPT2[2] = 0b1 INIT_START

    const uint32_t INIT_COMPLETE_TIMEOUT = 1000;
    uint32_t time = 0;
    uint32_t mcstat_reg = 0;

    do
    {
        mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
        if (mcstat_reg & reg_field(0, 0b1)) // INIT_COMPLETE
            break;
    }
    while (++time != INIT_COMPLETE_TIMEOUT);

    TEST_ASSERT(time != INIT_COMPLETE_TIMEOUT, "sdram_write_leveling_mode_off DDR INIT_COMPLETE_TIMEOUT");
}

void sdram_phy_write_leveling_mode_on (const uint32_t baseAddr)
{
    iowrite32 (0x0E, baseAddr + 0x014); // !!!Changed
    iowrite32 (0x40, baseAddr + 0x018);
}

void sdram_phy_write_leveling_mode_off (const uint32_t baseAddr)
{
    iowrite32 (0x4A, baseAddr + 0x014);
    iowrite32 (0x40, baseAddr + 0x018);
}

void ddr_ddr34lmc_init(const uint32_t baseAddr,
        DdrInitMode initMode,
        DdrEccMode eccMode,
        DdrPmMode powerManagementMode,
        DdrBurstLength burstLength,
        DdrPartialWriteMode partialWriteMode
)
{
    TEST_ASSERT(powerManagementMode == DdrPmMode_DisableAllPowerSavingFeatures, "Power saving features not supported");

    //1. System reset
    //Done by the system hardware

    //9. Trigger DDR PHY initialization to provide stable clock
    ddr34lmc_dfi_init(baseAddr);

    //Must wait for 200us after I_MC_RESET was deactivated
    if (initMode == DdrInitMode_FollowSpecWaitRequirements)
    {
        printf("Wait 200us according to spec...\n");
        usleep(200); //in microseconds
    }
    else if (initMode == DdrInitMode_ViolateSpecWaitRequirements)
    {
        printf("Speed up: Ignore 200us requirement\n");
    }

    //3. Deactivate SDRAM reset
    uint32_t mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            mcopt2_reg & ~(reg_field(11, 0b1111))); //MCOPT2[8:11] = 0b0000 RESET_RANK

    //4. MC enters power-on mode
    // Done automatically after reset

    //5. Configure MC regs
    // Total EM0 DDR RAM 8GB + 2GB(ECC) = 10GB	#ev - 2GB + 0.5GB(ECC)
    // PHY DQ width = 32 + 8(ECC) = 40 bit	
    // Die DQ width = x8	#ev - x16
    // Rank num = 2 (this is the DDR PHY limit, not the MC)	#ev - 1
    // Die num = (4 + 1(ECC)) x 2rank = 10 dies	#ev - (2 + 1(ECC)) x 1rank
    // Bank num = 8 (See JEDEC for 1Gbx8)
    // Quarter mode
    // MCIF_DATA_WIDTH = 128
    // BUILT_IN_ECC = 1
    // No ECC path-through
    // Column address width = 11 (See JEDEC for 1Gbx8)
    // Addressing mode = 3. Details are below
    //      Based on column bits number for 1Gbx8 in JEDEC spec table 2.11.5 Column address: A0-A9, A11.
    //      Then choose addressing mode from MC db table 3-52 based on our config: column addr width = 11, bank num = 8, quarter mode, 128 bit
    //      This is 3 | Nx11 (8) | 128
    //MCIF commands queue depth = 4
    //ODT = OFF
    //DDR SDRAM1600 11-11-11
    //I_MC_CLOCK = 400MHz //MC clock
    //tI_MC_CLOCK = 2500ps
    //dfi_clk1x = 400MHz //PHY clock
    //tdfi_clk1x = 2500ps
    //dfi_clk4x = 1600MHz //another PHY clock
    //tdfi_clk4x = 625ps
    //CK0 = 800MHz //DDR SDRAM clock rank0
    //CK1 = 800MHz //DDR SDRAM clock rank1
    //tCK = 1250ps

    ddr34lmc_dcr_write_DDR34LMC_MCOPT1(baseAddr,	//#ev
//      PROTOCOL             DM_ENABLE                       [ECC_EN : ECC_COR]      RDIMM                 PMUM               WIDTH[0]              WIDTH[1]           PORT_ID_CHECK_EN      UIOS              QUADCS_RDIMM        ZQCL_EN               WD_DLY               QDEPTH              RWOO                 WOOO                 DCOO                  DEF_REF              DEV_TYPE             CA_PTY_DLY           ECC_MUX               CE_THRESHOLD
        reg_field(0, 0b0) | reg_field(1, partialWriteMode) | reg_field(3, eccMode) | reg_field(4, 0b0) | reg_field(6, 0b00) | reg_field(7, 0b0) | reg_field(12, 0b0) | reg_field(8, 0b0) | reg_field(9, 0) | reg_field(10, 0b0) | reg_field(11, 0b0) | reg_field(13, 0b1) | reg_field(15, 0b10) | reg_field(16, 0b0) | reg_field(17, 0b0) | reg_field(18, 0b1) | reg_field(19, 0b0) | reg_field(20, 0b0) | reg_field(21, 0b0) | reg_field(23, 0b00) | reg_field(31, 0b1));

    ddr34lmc_dcr_write_DDR34LMC_IOCNTL(baseAddr, 0);

    ddr34lmc_dcr_write_DDR34LMC_PHYCNTL(baseAddr, 0);

    //Init ODTR0 - ODTR3
    //ddr34lmc_dcr_write_DDR34LMC_ODTR0(baseAddr, 0x02000000);	//#ev - ODT write rank 0 enable 
	ddr34lmc_dcr_write_DDR34LMC_ODTR0(baseAddr, 0x00000000);

    //ddr34lmc_dcr_write_DDR34LMC_ODTR1(baseAddr, 0x08000000);
	ddr34lmc_dcr_write_DDR34LMC_ODTR1(baseAddr, 0x00000000);

    ddr34lmc_dcr_write_DDR34LMC_ODTR2(baseAddr, 0);

    ddr34lmc_dcr_write_DDR34LMC_ODTR3(baseAddr, 0);

    //Init CFGR0 - CFGR3
    ddr34lmc_dcr_write_DDR34LMC_CFGR0(baseAddr, //#ev - ADDR_MODE 11->10
//                    ROW_WIDTH                 ADDR_MODE                               MIRROR              RANK_ENABLE
        reg_field(19, ddr_config.T_ROW_WIDTH) | reg_field(23, ddr_config.T_ADDR_MODE) | reg_field(27, 0b0) | reg_field(31, 0b1));

    ddr34lmc_dcr_write_DDR34LMC_CFGR1(baseAddr,	//#ev - RANK_ENABLE 1->0
//                    ROW_WIDTH                 ADDR_MODE                               MIRROR              RANK_ENABLE
        //reg_field(19, ddr_config.T_ROW_WIDTH) | reg_field(23, ddr_config.T_ADDR_MODE) | reg_field(27, 0b0) | reg_field(31, 0b1));
		reg_field(31, 0b0));

    ddr34lmc_dcr_write_DDR34LMC_CFGR2(baseAddr,
//     RANK_ENABLE
       reg_field(31, 0b0));

    ddr34lmc_dcr_write_DDR34LMC_CFGR3(baseAddr,
    //   RANK_ENABLE
            reg_field(31, 0b0));
    printf("    DDR34LMC_CFGR0 = 0x%08x\n", read_dcr_reg(baseAddr + DDR34LMC_CFGR0));

    //Init SMR0 - SMR3
    ddr34lmc_dcr_write_DDR34LMC_SMR0(baseAddr,
        //WR = TWR/tCK = 15000/1250 = 12
        //CL = CL_TIME/tCK = 13750/1250 = 11
//      PPD                  WR                               DLL (not used)          TM (not used)                 CL[3:1]                       CL[0]                RBT                 BL
        reg_field(19, 0b1) | reg_field(22, ddr_config.WR) | reg_field(23, 0b1) | reg_field(24, 0b0) | reg_field(27, ddr_config.CL_MC) | reg_field(29, 0b0) | reg_field(28, 0b0) | reg_field(31, burstLength));

    ddr34lmc_dcr_write_DDR34LMC_SMR1(baseAddr,	//#ev bit 30, 29: 00->11, 31: 1->0
//      QOFF (not used)         TDQS (not used)     RTT_Nom[2]         RTT_Nom[1]           RTT_Nom[0]         WR_Level (not used)       DIC[1]             DIC[0]                 AL                   DLL (not used)
        reg_field(19, 0b0) | reg_field(20, 0b0) | reg_field(22, 0b0) | reg_field(25, 0b1) | reg_field(29, 0b1) | reg_field(24, 0b0) | reg_field(26, 0b0) | reg_field(30, 0b1) | reg_field(28, ddr_config.AL_MC) | reg_field(31, 0b0));

    ddr34lmc_dcr_write_DDR34LMC_SMR2(baseAddr,
    //For CWL calculation see function min_cwl in SDRAM parameters.
    //CWL = 8
    //      RTT_WR (not used)    SRT (not used)           ASR (not used)    CWL                     PASR (not used)
            reg_field(22, 0b00) | reg_field(24, 0b0) | reg_field(25, 0) |  reg_field(28, ddr_config.CWL_MC) | reg_field(31, 0b000));

    ddr34lmc_dcr_write_DDR34LMC_SMR3(baseAddr,
    //      MPR (not used)         MPR_SEL (not used)
            reg_field(29, 0b0) | reg_field(31, 0b00));
    printf("    DDR34LMC_SMR0 = 0x%08x\n", read_dcr_reg(baseAddr + DDR34LMC_SMR0));
    printf("    DDR34LMC_SMR1 = 0x%08x\n", read_dcr_reg(baseAddr + DDR34LMC_SMR1));
    printf("    DDR34LMC_SMR2 = 0x%08x\n", read_dcr_reg(baseAddr + DDR34LMC_SMR2));
    printf("    DDR34LMC_SMR3 = 0x%08x\n", read_dcr_reg(baseAddr + DDR34LMC_SMR3));

    //Init SDTR0 - SDTR5
    ddr34lmc_dcr_write_DDR34LMC_SDTR0(baseAddr,
    //TRFC_MIN=  260000ps
    //TRFC_MAX=70200000ps
    //Set refresh interval for example 25000000ps
    //T_REFI = 25000000/tI_MC_CLOCK = 25000000/2500 = 10000 (-1 is needed according to MC db)
    //T_RFC_XPR = (TRFC_MIN + 10ns)/tI_MC_CLOCK = (260000 + 10000)/2500 = 108
    //   T_REFI                             T_RFC_XPR
        reg_field(15, ddr_config.T_REFI) | reg_field(31, ddr_config.T_RFC_XPR));

    ddr34lmc_dcr_write_DDR34LMC_SDTR1(baseAddr,
    //TODO: must be consistent with PHY. See MC db Note
    //      T_LEADOFF           ODT_DELAY          ODT_WIDTH            T_WTRO                  T_RTWO                  T_RTW_ADJ               T_WTWO                  T_RTRO
            reg_field(0, 0b0) | reg_field(1, 0b0) | reg_field(2, 0b0) | reg_field(7, 0b0000) | reg_field(15, 0b0000) | reg_field(19, 0b0000) | reg_field(23, 0b0000) | reg_field(31, 0b0000));

    ddr34lmc_dcr_write_DDR34LMC_SDTR2(baseAddr,
    //T_RCD = tRCD/tCK = 13750/1250 = 11
    //T_RP = tRP/tCK = 13750/1250 = 11
    //T_RC = tRC/tCK = 48750/1250 = 39
    //T_RAS = tRAS/tCK = 35000/1250 = 28
    //      T_CWL (not used)           T_RCD[1:4]                           T_RCD[0]            T_PL                    T_RP[1:4]                        T_RP[0]               T_RC                         T_RAS
            reg_field(3, 0b1000) |     reg_field(7, ddr_config.T_RCD) | reg_field(17, 0b0) | reg_field(11, 0b0000) | reg_field(15, ddr_config.T_RP) | reg_field(16, 0b0) | reg_field(23, ddr_config.T_RC) | reg_field(31, ddr_config.T_RAS));

    ddr34lmc_dcr_write_DDR34LMC_SDTR3(baseAddr,
    //T_WTR_S = tWTR_DG/tCK = 3750/1250 = 3
    //T_WTR = tWTR/tCK = 7500/1250 = 6
    //T_RTP = tRTP/tCK = 7500/1250 = 6
    //T_RRD = TRRD_TCK = 4
    //TXSDLL = (tXSRD/PHY_to_MC_clock_ratio)/16 = (512/2)/16 = 16
    //T_FAWADJ = 2. See Table 3-19 MC db
    //      T_WTR_S (not used)      T_WTR                                T_FAWADJ                   T_RTP                   T_RRD_L (not used)      T_RRD                        T_XSDLL
            reg_field(3, 0b0011) | reg_field(7, ddr_config.T_WTR) | reg_field(11, 0b10) | reg_field(15, ddr_config.T_RTP) | reg_field(19, 0b0010) | reg_field(23, ddr_config.T_RRD) | reg_field(31, ddr_config.T_XSDLL));

    const uint8_t T_SYS_RDLAT = (burstLength == DdrBurstLength_4) ? T_SYS_RDLAT_BC4 : T_SYS_RDLAT_BL8; //Experimental values. TODO: update comments below after MMPPC-548
    const uint32_t T_RDDATA_EN = (burstLength == DdrBurstLength_4) ? ddr_config.T_RDDATA_EN_BC4 : ddr_config.T_RDDATA_EN_BL8; //Experimental values. TODO: update comments below after MMPPC-548
    ddr34lmc_dcr_write_DDR34LMC_SDTR4(baseAddr,
    //t_rddata_en = (RL-1)/2-1 in tdfi_clk1x. See PHY spec
    // where RL = AL+CL = 0+11. See JEDEC spec
    //T_RDDATA_EN = t_rddata_en*tdfi_clk1x/tCK = ((11-1)/2-1)*2500/1250 = 8
    //T_SYS_RDLAT = tdfi_read_latency - (AL+CL) = tdfi_read_latency - 11
    //where tdfi_read_latency is from read cmd (cs) to rtdata_valid and it can be get from wave form
    //T_SYS_RDLAT = 17 - experimental value. See T_SYS_RDLAT_DBG register
    //T_CCD = 4
    //T_CPDED = default = 1
    //T_MOD = tMOD/tCK = 15000/1250 = 12
    //      T_RDDATA_EN[0:6]            T_SYS_RDLAT                  T_CCD_L (not used)          T_CCD                 T_CPDED                 T_MOD
            reg_field(7, T_RDDATA_EN) | reg_field(15, T_SYS_RDLAT) | reg_field(19, 0b0100) | reg_field(23, 0b100) | reg_field(26, 0b000) | reg_field(31, ddr_config.T_MOD));
    printf("T_RDDATA_EN = 0x%08x, T_SYS_RDLAT = 0x%08x\n", T_RDDATA_EN, T_SYS_RDLAT);

    ddr34lmc_dcr_write_DDR34LMC_SDTR5(baseAddr,
    //MC uses T_WHY_WRDATA value like this T_PHY_WRLAT = (CWL - 2*T_PHY_WRDATA)/2 (units: I_MC_CLOCK), where T_PHY_WRLAT is a time from WRITE cmd to dfi_wrdata_en signal
    //TWRLAT = (WL-1)/2 (units: I_MC_CLOCK) = 3.5; T_WRDAT = (CWL - 2*T_PHY_WRLAT)/2 (units: I_MC_CLOCK) = 0.5 (units: I_MC_CLOCK)
            reg_field(7, 0b000)); //TODO: how to set 0.5?

    //Init INITSEQn and INITCMDn
    //According to Micron 8Gb_DDR3L.pdf Figure 46: Initialization Sequence must be as follows
    //MRS-MR2
    //MRS-MR3
    //MRS-MR1 DLL enable
    //MRS-MR0 with DLL reset
    //ZQCL

    //MRS-MR2
    ddr34lmc_dcr_write_DDR34LMC_INITSEQ0(baseAddr,
    //Wait = TMRD*tCK/tI_MC_CLOCK = 4*1250/2500 = 2
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT                EN_MULTI_RANK_SELECT      RANK
            reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 0) |         reg_field(31, 0b0011));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD0(baseAddr,

    //See JEDEC 3.4.4 Mode Register MR2
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13 A12 A11      A10 A9                A8                   A7                   A6                   A5 A4 A3               A2 A1 A0
    //Value              0                      0                       0                   0                   1                    0                    0   0   0   0   0        Rtt_WR                0                    SRT                  ASR                  CWL                    PASR
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:4]                ADDR[5:6]             ADDR[7]              ADDR[8]              ADDR[9]              ADDR[10:12]            ADDR[13:15]
					reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b1) | reg_field(11, 0b0) | reg_field(20, 0b00000) | reg_field(22, 0b00) | reg_field(23, 0b0) | reg_field(24, 0b0) | reg_field(25, 0b0) | reg_field(28, ddr_config.CWL_CHIP) | reg_field(31, 0b000));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ1(baseAddr,
    //Wait = TMRD*tCK/tI_MC_CLOCK = 4*1250/2500 = 2
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT                EN_MULTI_RANK_SELECT      RANK
            reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 0) |         reg_field(31, 0b0011));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD1(baseAddr,
    //See JEDEC 3.4.5 Mode Register MR3
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13 A12 A11 A10 A9 A8 A7 A6 A5 A4 A3      A2                    A1 A0
    //Value              0                      0                       0                   0                   1                    1                    0   0   0   0   0   0   0  0  0  0  0  0  0       MPR                   MPR_Loc
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:12]                                        ADDR[13]              ADDR[14:15]
                         reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b1) | reg_field(11, 0b1) | reg_field(28, 0) |                                reg_field(29, 0b0)  | reg_field(31, 0b00));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ2(baseAddr,
    //Wait = TMRD*tCK/tI_MC_CLOCK = 4*1250/2500 = 2
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT                EN_MULTI_RANK_SELECT      RANK
            reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 0) |         reg_field(31, 0b0011));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD2(baseAddr,	//#ev bit30, 29: 00->11
    //See JEDEC 3.4.3 Mode Register MR1
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13        A12                  A11                  A10                A9                   A8                 A7                   A6                   A5                   A4 A3                						 A2                   A1                   A0
    //Value              0                      0                       0                   0                   0                    1                    0   0   0          Qoff                 TDQS                 0                  Rtt_Nom[2]           0                  Level                Rtt_Nom[1]           DIC[1]               AL                    						Rtt_Nom[0]           DIC[0]               DLL
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:2]          ADDR[3]              ADDR[4]              ADDR[5]            ADDR[6]              ADDR[7]            ADDR[8]              ADDR[9]              ADDR[10]             ADDR[11:12]           						ADDR[13]             ADDR[14]             ADDR[15]
                         reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b0) | reg_field(11, 0b1) | reg_field(18, 0) | reg_field(19, 0b0) | reg_field(20, 0b0) | reg_field(21, 0) | reg_field(22, 0b0) | reg_field(23, 0) | reg_field(24, 0b0) | reg_field(25, 0b0) | reg_field(26, 0b0) | reg_field(28, ddr_config.AL_CHIP) | reg_field(29, 0b1) | reg_field(30, 0b1) | reg_field(31, 0b0));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ3(baseAddr,
    //Wait = tMOD/tI_MC_CLOCK = 15000/2500 = 6
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //      ENABLE              WAIT              EN_MULTI_RANK_SELECT       RANK
            reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 0) |         reg_field(31, 0b0011));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD3(baseAddr,	//#ev WR 110 -> ddr_config.WR
    //See JEDEC 3.4.2 Mode Register MR0
    //DDR pin            RAS#                   CAS#                    WE#                 BA2                 BA1                  BA0                  A15 A14 A13        A12                  A11 A10 A9             		A8                   A7                   A6 A5 A4               				A3                   A2                   A1 A0
    //Value              0                      0                       0                   0                   0                    0                    0   0   0          PPD                  WR                     		DLL                  TM                   CL[3:1]                				RBT                  CL[0]                BL
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              BANK[0]             BANK[1]              BANK[2]              ADDR[0:2]          ADDR[3]              ADDR[4:6]             		 ADDR[7]              ADDR[8]              ADDR[9:11]             				ADDR[12]             ADDR[13]             ADDR[14:15]
                        reg_field(0, 0b0) |    reg_field(1, 0b0) |     reg_field(2, 0b0) | reg_field(9, 0b0) | reg_field(10, 0b0) | reg_field(11, 0b0) | reg_field(18, 0) | reg_field(19, 0b1) | reg_field(22, ddr_config.WR) | reg_field(23, 0b1) | reg_field(24, 0b0) | reg_field(27, ddr_config.CL_CHIP) | reg_field(28, 0b0) | reg_field(29, 0b0) | reg_field(31, burstLength));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ4(baseAddr,
    //Wait = tZQINIT/tI_MC_CLOCK = 640000/2500 = 256
    //RANKx is anded with CFGRx[RANK_ENABLE]. Let's set all ranks here
    //          ENABLE              WAIT                EN_MULTI_RANK_SELECT       RANK
                reg_field(0, 0b1) | reg_field(15, 512) | reg_field(27, 1) |         reg_field(31, 0b0011));

    ddr34lmc_dcr_write_DDR34LMC_INITCMD4(baseAddr,
    //See JEDEC Table 6 Command Truth Table (ZQCS)
    //DDR pin            RAS#                   CAS#                    WE#                 A10
    //Value              1                      1                       0                   0
    //INITCMD            CMD[0]                 CMD[1]                  CMD[2]              ADDR[5]
    reg_field(0, 0b1) |    reg_field(1, 0b1) |     reg_field(2, 0b0) | reg_field(21, 0b1));

    ddr34lmc_dcr_write_DDR34LMC_INITSEQ5(baseAddr, 0);

    ddr34lmc_dcr_write_DDR34LMC_INITCMD5(baseAddr, 0);

    //6. Configure MC calibration timeout regs (optional)
    ddr34lmc_dcr_write_DDR34LMC_T_PHYUPD0(baseAddr, 0x00001000);
    ddr34lmc_dcr_write_DDR34LMC_T_PHYUPD1(baseAddr, 0x00001000);
    ddr34lmc_dcr_write_DDR34LMC_T_PHYUPD2(baseAddr, 0x00001000);
    ddr34lmc_dcr_write_DDR34LMC_T_PHYUPD3(baseAddr, 0x00001000);

    //7. Exit power-on mode

    //Must wait for 500us after SDRAM reset deactivated
    if (initMode == DdrInitMode_FollowSpecWaitRequirements)
    {
        printf("Wait 500us according to spec...\n");
        usleep(500); //in microseconds
    }
    else if (initMode == DdrInitMode_ViolateSpecWaitRequirements)
    {
        printf("Speed up: Ignore 500us requirement\n");
    }

    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            (mcopt2_reg & ~(reg_field(0, 0b1))) //MCOPT2[0] = 0b0 SELF_REF_EN
            | reg_field(2, 0b1)); //MCOPT2[2] = 0b1 INIT_START

    const uint32_t INIT_COMPLETE_TIMEOUT = 1000;
    uint32_t time = 0;
    uint32_t mcstat_reg = 0;

    do
    {
        mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
        if (mcstat_reg & reg_field(0, 0b1)) // INIT_COMPLETE
            break;
    }
    while (++time != INIT_COMPLETE_TIMEOUT);

    TEST_ASSERT(time != INIT_COMPLETE_TIMEOUT, "DDR INIT_COMPLETE_TIMEOUT");


    //10. Enable MC
    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            mcopt2_reg | reg_field(3, 0b1));//MCOPT2[3] = 0b1 MC_ENABLE
    
    // report_DDR_core_regs (baseAddr);
}

#ifdef DEBUG
static void print_reg(uint32_t val, int len)
{
    uint32_t x = val, mask;


    while(len)
    {
	mask = 1 << (len-1);
	if (x&mask)
	{
	    printf("1");
	}
	else
	{
	    printf("0");
	}
	len--;
    }
    printf("\n");

}
#endif

// static void _ddr3phy_calibrate(const uint32_t baseAddr)
// {
//    dcrwrite32(0x01, baseAddr + DDR3PHY_PHYREG02);
//    usleep(6);  //  in microseconds
//    dcrwrite32(0x00, baseAddr + DDR3PHY_PHYREG02);  //  Stop calibration
// }

static void ddr3phy_calibrate(const uint32_t baseAddr)
{
    const uint32_t PHY_CALIBRATION_TIMEOUT = 100; //in us
    int time = 0;
    uint32_t reg;
    
    
    // iowrite32 (0x00, baseAddr + (0x28 << 2));  // 
    // iowrite32 (0x07, baseAddr + (0x38 << 2));  // 
    // iowrite32 (0x07, baseAddr + (0x48 << 2));  // 
    // iowrite32 (0x07, baseAddr + (0x58 << 2));  // 
    
    // wait_some_time ();
    
    /*
    printf("	Start write-leveling\n");
    iowrite32 (0xA4, baseAddr + (0x02 << 2));  // 
    wait_some_time ();
    iowrite32 (0x00, baseAddr + (0x02 << 2));  // 
    */

    //  CMD drive strength - made no sense
    iowrite32 (0xCC, baseAddr + (0x11 << 2));
    //  CMD weak pull up enable, CMD weak pull down enable
    //  Looks like there is smth else undocumented on its address
    // iowrite32 (0x00, baseAddr + (0x12 << 2));
    //  CLK drive strength - made no sense - OSC: first peak increase a bit, if increase
    iowrite32 (0xCC, baseAddr + (0x16 << 2));
    
    //  DQ and DM drive strength - made no sense. Didnt work at all, if high resistance.
    iowrite32 (0xCC, baseAddr + (0x20 << 2));  //  (0-3)-РјР»Р°РґС€РёР№ Р±Р°Р№С‚ (СЃРїСЂР°РІР°)
    iowrite32 (0xCC, baseAddr + (0x30 << 2));  //  (0-3)
    iowrite32 (0xCC, baseAddr + (0x40 << 2));  //  (0-3)
    iowrite32 (0xCC, baseAddr + (0x50 << 2));  //  (0-3)-СЃС‚Р°СЂС€РёР№ Р±Р°Р№С‚ (СЃР»РµРІР°)
    
    //  DQ and DM ODT resistance - made no sense. Didnt work at all, if high resistance.
    iowrite32 (0x99, baseAddr + (0x21 << 2));  // 
    iowrite32 (0x99, baseAddr + (0x31 << 2));  // 
    iowrite32 (0x99, baseAddr + (0x41 << 2));  // 
    iowrite32 (0x99, baseAddr + (0x51 << 2));  // 
    
    wait_some_time ();
    
    if (DDR_FREQ == DDR3_1333)
    {
        iowrite32 (0xC + 0x2, baseAddr + (0x13 << 2));  // CMD AND ADDRESS DLL delay  (8-f)
        iowrite32 (0x2      , baseAddr + (0x14 << 2));  // CK DLL delay (0-7)
    }   
    if (DDR_FREQ == DDR3_1600)
    {
        iowrite32 (0x6      , baseAddr + (0x14 << 2));  // CK DLL delay (0-7)
    }   
    
    wait_some_time ();
    
    // ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0x8);	// CMD AND ADDRESS DLL delay  (8-f)
    // ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0x4);	// CK DLL delay (0-7)
    // wait_some_time ();
    
    //-----------------------------------------------------------------------------------------
    printf("	Start write-leveling\n");
    
    // if (baseAddr == 0x3800E000)
        // sdram_write_leveling_mode_on (EM0_DDR3LMC_DCR);
    // else
    // while (1)
    // {
        // // sdram_write_leveling_mode_on (EM1_DDR3LMC_DCR);
    // sdram_write_leveling_cs1_off (EM1_DDR3LMC_DCR);
    sdram_phy_write_leveling_mode_on (baseAddr);
    
    ddr3phy_write_DDR3PHY_PHYREG02(baseAddr, 0xA4); //Start write-leveling
    
    time = 0;
    do
    {
        reg = ddr3phy_read_DDR3PHY_PHYREGF0(baseAddr);
        if ((reg & 0x1f) == 0x1f)
            break;
        usleep(1);
    } while(++time != PHY_CALIBRATION_TIMEOUT);
    
    
    ddr3phy_write_DDR3PHY_PHYREG02(baseAddr, 0xA0); //Stop write-leveling
    
    TEST_ASSERT(time != PHY_CALIBRATION_TIMEOUT, "write-leveling timeout");
    printf("	write-leveling.time = %d\n", time);

    reg = ddr3phy_read_DDR3PHY_PHYREGF0(baseAddr);
    printf("	write-level.status (PHYREGF0) = 0x%08x\n", reg);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGF1(baseAddr);
    printf("	write-level.value ch_A (PHYREGF1) = 0x%08x\n", reg);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGF2(baseAddr);
    printf("	write-level.value ch_B (PHYREGF2) = 0x%08x\n", reg);
    
    wait_some_time ();
    
    // if (baseAddr == 0x3800E000)
        // sdram_write_leveling_mode_off (EM0_DDR3LMC_DCR);
    // else
        // sdram_write_leveling_mode_off (EM1_DDR3LMC_DCR);
    // sdram_phy_write_leveling_mode_off (baseAddr);
    // }
    //-----------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------
    // ddr3phy_write_DDR3PHY_PHYREG02(baseAddr, 0xA8); //  Write leveling bypass mode
    //
    //  Unstable results
    
    // ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0xA);	// CMD AND ADDRESS DLL delay  (8-f)
    // ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0xE);	// CK DLL delay (0-7)
    
    // iowrite32 (0x08, baseAddr + (0x26 << 2));  //  write DQ DLL delay
    // iowrite32 (0x08, baseAddr + (0x36 << 2));  // 
    // iowrite32 (0x08, baseAddr + (0x46 << 2));  // 
    // iowrite32 (0x08, baseAddr + (0x56 << 2));  // 
    
    // iowrite32 (0x0C, baseAddr + (0x27 << 2));  //  write DQS DLL delay
    // iowrite32 (0x0C, baseAddr + (0x37 << 2));  // 
    // iowrite32 (0x0C, baseAddr + (0x47 << 2));  // 
    // iowrite32 (0x0C, baseAddr + (0x57 << 2));  // 
    // wait_some_time ();
    //-----------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------
    //8. Configure PHY calibration regs
    //-------------------------------------------------------------------------------------
    //  It pass always (if we look on status reg)
    //    On CRUCIAL one-rank memories:
    //      one result register is not stable (PHYREGF8 - ECC byte) - both DDRs
    //      other register has very starnge value (PHYREGFE - high byte) - DDR1 only
    //    On HYPERX memroies:
    //      All registers are stable and had approximately the same values.
    //
    //  If we use bypass, then presence of this procedure has no matter.
    //-------------------------------------------------------------------------------------
    
    printf("	Start dqs-gating\n");
    
    ddr3phy_write_DDR3PHY_PHYREG02(baseAddr, 0xA1); //Start calibration
    
    // volatile uint32_t rdata;
    // volatile uint32_t i;
    

    do
    {
        reg = ddr3phy_read_DDR3PHY_PHYREGFF(baseAddr);
        if ((reg & 0x1f) == 0x1f)
            break;
        usleep(1);
    } while(++time != PHY_CALIBRATION_TIMEOUT);
    
    ddr3phy_write_DDR3PHY_PHYREG02(baseAddr, 0xA0); //Stop calibration

    TEST_ASSERT(time != PHY_CALIBRATION_TIMEOUT, "dqs-gating timeout");
    printf("	dqs-gating.time = %d\n", time);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGFF(baseAddr);
    printf("	dqs-gating.status = 0x%08x\n", reg);
    
    //-----------------------------------------------------------------------------------------

    
    //-----------------------------------------------------------------------------------------
    
    //-----------------------------------------------------------------------------------------
    // ddr3phy_write_DDR3PHY_PHYREG02(baseAddr, 0xA2); //  DQS gating bypass mode
    // iowrite32 (0x26, baseAddr + (0x2C << 2));  // read DQS DLL delay (0-3)-младший байт (справа)
    // iowrite32 (0x2C, baseAddr + (0x3C << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0x31, baseAddr + (0x4C << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0x28, baseAddr + (0x5C << 2));  // read DQS DLL delay (0-3)-старший байт (слева)
    // iowrite32 (0x2A, baseAddr + (0x6C << 2));  // read DQS DLL delay ECC
    //-----------------------------------------------------------------------------------------
    //  Data is unstable after this:
    //-----------------------------------------------------------------------------------------
    // iowrite32 (0x3, baseAddr + (0x28 << 2));  // read DQS DLL delay (0-3)-младший байт (справа)
    // iowrite32 (0x3, baseAddr + (0x38 << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0x3, baseAddr + (0x48 << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0x3, baseAddr + (0x58 << 2));  // read DQS DLL delay (0-3)-старший байт (слева)
    // iowrite32 (0x3, baseAddr + (0x29 << 2));  // read DQS DLL delay (0-3)-младший байт (справа)
    // iowrite32 (0x3, baseAddr + (0x39 << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0x3, baseAddr + (0x49 << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0x3, baseAddr + (0x59 << 2));  // read DQS DLL delay (0-3)-старший байт (слева)
    //-----------------------------------------------------------------------------------------
    
    
    // email from INNO, did
    // please configure the register 0x11b[7:4] to 0x07, this will decrease the skew of the clock
    // reg = MEM32(baseAddr + 0x11B);
    // reg &= ~0xf0;
    // MEM32(baseAddr + 0x11B) = reg | 0x70;

    // iowrite32 (0x3F, baseAddr + (0x00 << 2));  // turn channels off
    
    
    //  DQ and DM drive strength - made no sense. Didnt work at all, if high resistance.
    // iowrite32 (0xAA, baseAddr + (0x20 << 2));  // read DQS DLL delay (0-3)-младший байт (справа)
    // iowrite32 (0xAA, baseAddr + (0x30 << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0xAA, baseAddr + (0x40 << 2));  // read DQS DLL delay (0-3)
    // iowrite32 (0xAA, baseAddr + (0x50 << 2));  // read DQS DLL delay (0-3)-старший байт (слева)
    
    //  DQ and DM ODT resistance - made no sense. Didnt work at all, if high resistance.
    // iowrite32 (0xAA, baseAddr + (0x21 << 2));  // 
    // iowrite32 (0xAA, baseAddr + (0x31 << 2));  // 
    // iowrite32 (0xAA, baseAddr + (0x41 << 2));  // 
    // iowrite32 (0xAA, baseAddr + (0x51 << 2));  // 
    
    //  Read DLL settings - made no sense
    // iowrite32 (0xFF, baseAddr + (0x0b0));
    // iowrite32 (0xF7, baseAddr + (0x0f0));
    // iowrite32 (0x00, baseAddr + (0x130));
    // iowrite32 (0xF7, baseAddr + (0x170));
    
    //  CMD drive strength - made no sense
    // iowrite32 (0x11, baseAddr + (0x11 << 2));
    //  CLK drive strength - made no sense - OSC: first peak increase a bit, if increase
    // iowrite32 (0xFF, baseAddr + (0x16 << 2));
    
    wait_some_time ();
    


    printf("Calibration done.\n");

#ifdef DEBUG
    printf("\n\nINFO:\n");
    reg = ddr3phy_read_DDR3PHY_PHYREGF0(baseAddr);
    printf("\n	PHYREGF0:\n    0x%08x\n", reg);

    reg = ddr3phy_read_DDR3PHY_PHYREGF1(baseAddr);
    //printf("REG = 0x%08x\n", reg);
    printf("\n	PHYREGF1:\n	Channel A High 8bit write leveling dqs value	"); print_reg((reg&0xf0)>>4, 4);
    printf("	Channel A Low 8bit write leveling dqs value	"); print_reg((reg&0xf)>>0, 4);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGF2(baseAddr);
    //printf("REG = 0x%08x\n", reg);
    printf("\n	PHYREGF2:\n	Channel B High 8bit write leveling dqs value	"); print_reg((reg&0xf0)>>4, 4);
    printf("	Channel B Low 8bit write leveling dqs value	"); print_reg((reg&0xf)>>0, 4);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGFA(baseAddr);
    //printf("REG = 0x%08x\n", reg);
    printf("\n	PHYREGFA:\n	Channel B High 8bit dqs gate sample dqs value(idqs) (3)		%c\n", (reg&0x8)>>3 ? '1' : '0');
    printf("	Channel B Low 8bit dqs gate sample dqs value(idqs) (3)		%c\n", (reg&0x4)>>2 ? '1' : '0');
    printf("	Channel A High 8bit dqs gate sample dqs value(idqs) (3)		%c\n", (reg&0x2)>>1 ? '1' : '0');
    printf("	Channel A Low 8bit dqs gate sample dqs value(idqs) (3)		%c\n", (reg&0x1)>>0 ? '1' : '0');
    
    reg = ddr3phy_read_DDR3PHY_PHYREGFB(baseAddr);
    //printf("REG = 0x%08x\n", reg);
    printf("\n	PHYREGFB:\n	Calibration get the cyclesel configure channel A low 8bit(3)	"); print_reg((reg&0x70)>>5,3);
    printf("	Calibration get the ophsel configure channel A low 8bit(3)	"); print_reg((reg&0x18)>>3,2);
    printf("	Calibration get the dll configure channel A low 8bit(3) 	"); print_reg((reg&0x7)>>0,3);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGFC(baseAddr);
    //printf("REG = 0x%08x\n", reg);
    printf("\n	PHYREGFC:\n	Calibration get the cyclesel configure channel A high 8bit(3)	"); print_reg((reg&0x70)>>5,3);
    printf("	Calibration get the ophsel configure channel A high 8bit(3)	"); print_reg((reg&0x18)>>3,2);
    printf("	Calibration get the dll configure channel A high 8bit(3) 	"); print_reg((reg&0x7)>>0,3);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGFD(baseAddr);
    //printf("REG = 0x%08x\n", reg);
    printf("\n	PHYREGFD:\n	Calibration get the cyclesel configure channel B low 8bit(3)	"); print_reg((reg&0x70)>>5,3);
    printf("	Calibration get the ophsel configure channel B low 8bit(3)	"); print_reg((reg&0x18)>>3,2);
    printf("	Calibration get the dll configure channel B low 8bit(3) 	"); print_reg((reg&0x7)>>0,3);
    
    reg = ddr3phy_read_DDR3PHY_PHYREGFE(baseAddr);
    //printf("REG = 0x%08x\n", reg);
    printf("\n	PHYREGFE:\n	Calibration get the cyclesel configure channel B high 8bit(3)	"); print_reg((reg&0x70)>>5,3);
    printf("	Calibration get the ophsel configure channel B high 8bit(3)	"); print_reg((reg&0x18)>>3,2);
    printf("	Calibration get the dll configure channel B high 8bit(3) 	"); print_reg((reg&0x7)>>0,3);
    
    
        reg = ddr3phy_read_DDR3PHY_PHYREG00(baseAddr);
        printf("	DDR3PHY_PHYREG00 = 0x%08x\n", reg);

        printf("	DDR3PHY_PHYREG13 = 0x%08x\n", ioread32 (baseAddr + (0x13 << 2)));
        printf("	DDR3PHY_PHYREG14 = 0x%08x\n", ioread32 (baseAddr + (0x14 << 2)));
        
        printf("  Registers, related to 4 different DDR channels\n");
        printf("	DDR3PHY_PHYREG03 = 0x%08x\n", ioread32 (baseAddr + 0x00c));
        printf("	DDR3PHY_PHYREG04 = 0x%08x\n", ioread32 (baseAddr + 0x010));
        printf("    ----\n");
        
        printf("	DDR3PHY_PHYREG20 = 0x%08x\n", ioread32 (baseAddr + (0x20 << 2)));
        printf("	DDR3PHY_PHYREG21 = 0x%08x\n", ioread32 (baseAddr + (0x21 << 2)));
        printf("	DDR3PHY_PHYREG26 = 0x%08x\n", ioread32 (baseAddr + (0x26 << 2)));
        printf("	DDR3PHY_PHYREG27 = 0x%08x\n", ioread32 (baseAddr + (0x27 << 2)));
        printf("	DDR3PHY_PHYREG28 = 0x%08x\n", ioread32 (baseAddr + (0x28 << 2)));
        printf("    --\n");

        printf("	DDR3PHY_PHYREG30 = 0x%08x\n", ioread32 (baseAddr + (0x30 << 2)));
        printf("	DDR3PHY_PHYREG31 = 0x%08x\n", ioread32 (baseAddr + (0x31 << 2)));
        printf("	DDR3PHY_PHYREG36 = 0x%08x\n", ioread32 (baseAddr + (0x36 << 2)));
        printf("	DDR3PHY_PHYREG37 = 0x%08x\n", ioread32 (baseAddr + (0x37 << 2)));
        printf("	DDR3PHY_PHYREG38 = 0x%08x\n", ioread32 (baseAddr + (0x38 << 2)));
        printf("    --\n");

        printf("	DDR3PHY_PHYREG40 = 0x%08x\n", ioread32 (baseAddr + (0x40 << 2)));
        printf("	DDR3PHY_PHYREG41 = 0x%08x\n", ioread32 (baseAddr + (0x41 << 2)));
        printf("	DDR3PHY_PHYREG46 = 0x%08x\n", ioread32 (baseAddr + (0x46 << 2)));
        printf("	DDR3PHY_PHYREG47 = 0x%08x\n", ioread32 (baseAddr + (0x47 << 2)));
        printf("	DDR3PHY_PHYREG48 = 0x%08x\n", ioread32 (baseAddr + (0x48 << 2)));
        printf("    --\n");

        printf("	DDR3PHY_PHYREG50 = 0x%08x\n", ioread32 (baseAddr + (0x50 << 2)));
        printf("	DDR3PHY_PHYREG51 = 0x%08x\n", ioread32 (baseAddr + (0x51 << 2)));
        printf("	DDR3PHY_PHYREG56 = 0x%08x\n", ioread32 (baseAddr + (0x56 << 2)));
        printf("	DDR3PHY_PHYREG57 = 0x%08x\n", ioread32 (baseAddr + (0x57 << 2)));
        printf("	DDR3PHY_PHYREG58 = 0x%08x\n", ioread32 (baseAddr + (0x58 << 2)));
        printf("    ----\n");

        printf("	DDR3PHY_PHYREG2C = 0x%08x\n", ioread32 (baseAddr + (0x0b0)));
        printf("	DDR3PHY_PHYREG3C = 0x%08x\n", ioread32 (baseAddr + (0x0f0)));
        printf("	DDR3PHY_PHYREG4C = 0x%08x\n", ioread32 (baseAddr + (0x130)));
        printf("	DDR3PHY_PHYREG5C = 0x%08x\n", ioread32 (baseAddr + (0x170)));
        printf("    ----\n");

        printf("	DDR3PHY_PHYREG70 = 0x%08x\n", ioread32 (baseAddr + (0x70 << 2)));
        printf("	DDR3PHY_PHYREG71 = 0x%08x\n", ioread32 (baseAddr + (0x71 << 2)));
        printf("	DDR3PHY_PHYREG72 = 0x%08x\n", ioread32 (baseAddr + (0x72 << 2)));
        printf("	DDR3PHY_PHYREG73 = 0x%08x\n", ioread32 (baseAddr + (0x73 << 2)));
        printf("	DDR3PHY_PHYREG74 = 0x%08x\n", ioread32 (baseAddr + (0x74 << 2)));
        printf("	DDR3PHY_PHYREG75 = 0x%08x\n", ioread32 (baseAddr + (0x75 << 2)));
        printf("	DDR3PHY_PHYREG76 = 0x%08x\n", ioread32 (baseAddr + (0x76 << 2)));
        printf("	DDR3PHY_PHYREG77 = 0x%08x\n", ioread32 (baseAddr + (0x77 << 2)));
        printf("	DDR3PHY_PHYREG78 = 0x%08x\n", ioread32 (baseAddr + (0x78 << 2)));
        printf("	DDR3PHY_PHYREG79 = 0x%08x\n", ioread32 (baseAddr + (0x79 << 2)));
        printf("	DDR3PHY_PHYREG7A = 0x%08x\n", ioread32 (baseAddr + (0x7A << 2)));
        printf("	DDR3PHY_PHYREG7B = 0x%08x\n", ioread32 (baseAddr + (0x7B << 2)));
        printf("	DDR3PHY_PHYREG7C = 0x%08x\n", ioread32 (baseAddr + (0x7C << 2)));
        printf("	DDR3PHY_PHYREG7D = 0x%08x\n", ioread32 (baseAddr + (0x7D << 2)));
        printf("	DDR3PHY_PHYREG7E = 0x%08x\n", ioread32 (baseAddr + (0x7E << 2)));
        printf("	DDR3PHY_PHYREG7F = 0x%08x\n", ioread32 (baseAddr + (0x7F << 2)));
        printf("	DDR3PHY_PHYREG80 = 0x%08x\n", ioread32 (baseAddr + (0x80 << 2)));
        printf("	DDR3PHY_PHYREG81 = 0x%08x\n", ioread32 (baseAddr + (0x81 << 2)));
        printf("    ----\n");

        printf("	DDR3PHY_PHYREG11 = 0x%08x\n", ioread32 (baseAddr + (0x11 << 2)));
        printf("	DDR3PHY_PHYREG16 = 0x%08x\n", ioread32 (baseAddr + (0x16 << 2)));
        printf("    ----\n");

    printf("\nINFO END:\n");
#endif
    
}


// static void ddr3phy_calibrate_manual_EM0(const unsigned int baseAddr)	//#ev
// {
//
//    unsigned int reg;
//
//     //#ev ручная подстройка задержек для скорости 1066
//    if (DDR_FREQ == DDR3_1066) {
//
//    ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0xf);	// CMD AND ADDRESS DLL delay  (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0x7);	// CK DLL delay (0-7)
//
//    ddr3phy_write_DDR3PHY_PHYREG28(baseAddr,0x1);	// read DQS DLL delay (0-3)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG38(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG48(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG58(baseAddr,0x1);	// read DQS DLL delay (0-3)-старший байт (слева)
//
//    ddr3phy_write_DDR3PHY_PHYREG26(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG36(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG46(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG56(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//
//    ddr3phy_write_DDR3PHY_PHYREG27(baseAddr,0x1);  // Write DQS DLL delay (0-7)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG37(baseAddr,0x1);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG47(baseAddr,0x1);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG57(baseAddr,0x1);  // Write DQS DLL delay (0-7)-старший байт (слева)
//
//     }
// 
// 
//    //#ev ручная подстройка задержек для скорости 1333
//    if (DDR_FREQ == DDR3_1333) {
//    
//    ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0xc);	// CMD AND ADDRESS DLL delay  (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0x0);	// CK DLL delay (0-7)
//    
//    ddr3phy_write_DDR3PHY_PHYREG28(baseAddr,0x1);	// read DQS DLL delay (0-3)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG38(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG48(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG58(baseAddr,0x1);	// read DQS DLL delay (0-3)-старший байт (слева)
//    
//    ddr3phy_write_DDR3PHY_PHYREG26(baseAddr,0xe);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG36(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG46(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG56(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    
//    ddr3phy_write_DDR3PHY_PHYREG27(baseAddr,0x2);  // Write DQS DLL delay (0-7)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG37(baseAddr,0x1);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG47(baseAddr,0x0);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG57(baseAddr,0x1);  // Write DQS DLL delay (0-7)-старший байт (слева)
//       
//    }
//    
//    //#ev ручная подстройка задержек для скорости 1600
//    if (DDR_FREQ == DDR3_1600) {
//    
//    ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0xc);	// CMD AND ADDRESS DLL delay  (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0x0);	// CK DLL delay (0-7)
//    
//    ddr3phy_write_DDR3PHY_PHYREG28(baseAddr,0x1);	// read DQS DLL delay (0-3)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG38(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG48(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG58(baseAddr,0x1);	// read DQS DLL delay (0-3)-старший байт (слева)
//    
//    ddr3phy_write_DDR3PHY_PHYREG26(baseAddr,0xe);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG36(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG46(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG56(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    
//    ddr3phy_write_DDR3PHY_PHYREG27(baseAddr,0x2);  // Write DQS DLL delay (0-7)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG37(baseAddr,0x1);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG47(baseAddr,0x0);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG57(baseAddr,0x1);  // Write DQS DLL delay (0-7)-старший байт (слева)
//     
//    }
//    usleep(10);  //  in microseconds #ev 
//    
//    
//    
//    // email from INNO, did
//   // please configure the register 0x11b[7:4] to 0x07, this will decrease the skew of the clock
//   reg = MEM32(baseAddr + 0x11B);
//   reg &= ~0xf0;
//   MEM32(baseAddr + 0x11B) = reg | 0x70;
// }

// static void ddr3phy_calibrate_manual_EM1(const unsigned int baseAddr)	//#ev
// {
//     
//    unsigned int reg;
//    
//     //#ev ручная подстройка задержек для скорости 1066
//    if (DDR_FREQ == DDR3_1066) {
//    
//    ddr3phy_write_DDR3PHY_PHYREG02(baseAddr, 0xA8); //  Write leveling bypass mode
//     
//    ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0x8);	// CMD AND ADDRESS DLL delay  (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0x4);	// CK DLL delay (0-7)
//    
//    ddr3phy_write_DDR3PHY_PHYREG28(baseAddr,0x1);	// read DQS DLL delay (0-3)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG38(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG48(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG58(baseAddr,0x1);	// read DQS DLL delay (0-3)-старший байт (слева)
//    
//    ddr3phy_write_DDR3PHY_PHYREG26(baseAddr,0x8);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG36(baseAddr,0x8);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG46(baseAddr,0x8);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG56(baseAddr,0x8);	// Write DQ DLL delay (8-f)
//    
//    ddr3phy_write_DDR3PHY_PHYREG27(baseAddr,0x2);  // Write DQS DLL delay (0-7)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG37(baseAddr,0x2);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG47(baseAddr,0x2);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG57(baseAddr,0x2);  // Write DQS DLL delay (0-7)-старший байт (слева)
//    /*
//    ddr3phy_write_DDR3PHY_PHYREG70(baseAddr,0x77);	// CS0 A_DM0 TX de-skew - CS0 A_DM0 RX de-skew
//    ddr3phy_write_DDR3PHY_PHYREG7B(baseAddr,0x77);	// CS0 A_DM1 TX de-skew - CS0 A_DM1 RX de-skew
//    ddr3phy_write_DDR3PHY_PHYREG86(baseAddr,0x77);	// CS0 B_DM0 TX de-skew - CS0 B_DM0 RX de-skew
//    ddr3phy_write_DDR3PHY_PHYREG91(baseAddr,0x77);	// CS0 B_DM1 TX de-skew - CS0 B_DM1 RX de-skew
// 													//где C?
//    ddr3phy_write_DDR3PHY_PHYREGC0(baseAddr,0x77);	// CS1 A_DM0 TX de-skew - CS1 A_DM0 RX de-skew
//    ddr3phy_write_DDR3PHY_PHYREGCB(baseAddr,0x77);	// CS1 A_DM1 TX de-skew - CS1 A_DM1 RX de-skew
//    ddr3phy_write_DDR3PHY_PHYREGD6(baseAddr,0x77);	// CS1 B_DM0 TX de-skew - CS1 B_DM0 RX de-skew
//    ddr3phy_write_DDR3PHY_PHYREGE1(baseAddr,0x77);	// CS1 B_DM1 TX de-skew - CS1 B_DM1 RX de-skew
// 													//где C?
// 												
//      */
// 	 
// 	reg = ddr3phy_read_DDR3PHY_PHYREG70(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREG70 = 0x%08x\n", reg);
// 	reg = ddr3phy_read_DDR3PHY_PHYREG7B(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREG7B = 0x%08x\n", reg);
// 	reg = ddr3phy_read_DDR3PHY_PHYREG86(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREG86 = 0x%08x\n", reg);
// 	reg = ddr3phy_read_DDR3PHY_PHYREG91(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREG91 = 0x%08x\n", reg);
// 	
// 	reg = ddr3phy_read_DDR3PHY_PHYREGC0(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREGC0 = 0x%08x\n", reg);
// 	reg = ddr3phy_read_DDR3PHY_PHYREGCB(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREGCB = 0x%08x\n", reg);
// 	reg = ddr3phy_read_DDR3PHY_PHYREGD6(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREGD6 = 0x%08x\n", reg);
// 	reg = ddr3phy_read_DDR3PHY_PHYREGE1(baseAddr);
//     printf("	EM1 DDR3PHY_PHYREGE1 = 0x%08x\n", reg);
// 	
// 		
//    }
// 
// 
//    //#ev ручная подстройка задержек для скорости 1333
//    if (DDR_FREQ == DDR3_1333) {
//    
//    ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0xc);	// CMD AND ADDRESS DLL delay  (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0x0);	// CK DLL delay (0-7)
//    
//    ddr3phy_write_DDR3PHY_PHYREG28(baseAddr,0x1);	// read DQS DLL delay (0-3)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG38(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG48(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG58(baseAddr,0x1);	// read DQS DLL delay (0-3)-старший байт (слева)
//    
//    ddr3phy_write_DDR3PHY_PHYREG26(baseAddr,0xe);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG36(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG46(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG56(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    
//    ddr3phy_write_DDR3PHY_PHYREG27(baseAddr,0x2);  // Write DQS DLL delay (0-7)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG37(baseAddr,0x1);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG47(baseAddr,0x0);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG57(baseAddr,0x1);  // Write DQS DLL delay (0-7)-старший байт (слева)
//       
//    }
//    
//    //#ev ручная подстройка задержек для скорости 1600
//    if (DDR_FREQ == DDR3_1600) {
//    
//    ddr3phy_write_DDR3PHY_PHYREG13(baseAddr,0xc);	// CMD AND ADDRESS DLL delay  (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG14(baseAddr,0x0);	// CK DLL delay (0-7)
//    
//    ddr3phy_write_DDR3PHY_PHYREG28(baseAddr,0x1);	// read DQS DLL delay (0-3)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG38(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG48(baseAddr,0x1);	// read DQS DLL delay (0-3)
//    ddr3phy_write_DDR3PHY_PHYREG58(baseAddr,0x1);	// read DQS DLL delay (0-3)-старший байт (слева)
//    
//    ddr3phy_write_DDR3PHY_PHYREG26(baseAddr,0xe);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG36(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG46(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    ddr3phy_write_DDR3PHY_PHYREG56(baseAddr,0xc);	// Write DQ DLL delay (8-f)
//    
//    ddr3phy_write_DDR3PHY_PHYREG27(baseAddr,0x2);  // Write DQS DLL delay (0-7)-младший байт (справа)
//    ddr3phy_write_DDR3PHY_PHYREG37(baseAddr,0x1);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG47(baseAddr,0x0);  // Write DQS DLL delay (0-7)
//    ddr3phy_write_DDR3PHY_PHYREG57(baseAddr,0x1);  // Write DQS DLL delay (0-7)-старший байт (слева)
//     
//    }
//    usleep(10);  //  in microseconds #ev 
//    
//   
//    
//    // email from INNO, did
//   // please configure the register 0x11b[7:4] to 0x07, this will decrease the skew of the clock
//   reg = MEM32(baseAddr + 0x11B);
//   reg &= ~0xf0;
//   MEM32(baseAddr + 0x11B) = reg | 0x70;
//   
// }

static void ddr_set_main_config (CrgDdrFreq FreqMode)	//#ev
{   
    ddr_config.T_ROW_WIDTH = 0b100;
    ddr_config.T_ADDR_MODE = 0b10;
    ddr_config.T_RRD = 0b100;

    if (FreqMode == DDR3_1600)
    {
        printf("	FreqMode = DDR3_1600\n");
        ddr_config.T_RDDATA_EN_BC4 = 18;
        ddr_config.T_RDDATA_EN_BL8 = 16;

        ddr_config.CL_PHY =     11;
        ddr_config.CL_MC =      0b111;
        ddr_config.CL_CHIP =    0b111;

        ddr_config.CWL_PHY =    8;
        ddr_config.CWL_MC =     0b100;
        ddr_config.CWL_CHIP =   0b100;

        ddr_config.AL_PHY =     9;
        ddr_config.AL_MC =      0b10;
        ddr_config.AL_CHIP =    0b10;

        ddr_config.WR =         0b110;
        ddr_config.T_REFI =     3120;
        ddr_config.T_RFC_XPR =  128;
        ddr_config.T_RCD =      11;
        ddr_config.T_RP =       11;
        ddr_config.T_RC =       38;
        ddr_config.T_RAS =      28;
        ddr_config.T_RRD =      6;
        ddr_config.T_WTR =      0b0111;
        ddr_config.T_RTP =      0b0111;
        ddr_config.T_XSDLL =    32;
        ddr_config.T_MOD =      12;
    }
    
    if (FreqMode == DDR3_1333)
    {
        printf("	FreqMode = DDR3_1333\n");
        ddr_config.T_RDDATA_EN_BC4 = 14;	//?
        ddr_config.T_RDDATA_EN_BL8 = 14;	//?

        ddr_config.CL_PHY =     9;
        ddr_config.CL_MC =      0b101;	//001-5 010-6 011-7 100-8 101-9 110-10 111-11
        ddr_config.CL_CHIP =    0b101;	//001-5 010-6 011-7 100-8 101-9 110-10 111-11

        ddr_config.CWL_PHY =    7;
        ddr_config.CWL_MC =     0b001;
        ddr_config.CWL_CHIP =   0b010;	//000-5 001-6 010-7 011-8

        ddr_config.AL_PHY =     8;
        ddr_config.AL_MC =      0b01;	//CL-1 
        ddr_config.AL_CHIP =    0b01;	//CL-1 

        ddr_config.WR =         0b101; //10
        ddr_config.T_REFI =     7999;	//refresh interval (in cycles core clock)
        ddr_config.T_RFC_XPR =  90;	//tRFC - Refresh to ACT or REF
        ddr_config.T_RCD =      0b1110;	//ACTIVATE to READ/WRITE (in DDR clock cycles)
        ddr_config.T_RP =       0b1110;	//PRECHARGE to ACTIVATE (in DDR clock cycles)
        ddr_config.T_RC =       33;	//ACIVATE to ACIVATE (in DDR clock cycles)
        ddr_config.T_RAS =      24;	 //ACTIVATE to PRECHARGE (in DDR clock cycles)
        ddr_config.T_RRD =      5;
        ddr_config.T_WTR =      0b0101;	//delay from start of internal write transaction to internal read command(in DDR clock cycles)
        ddr_config.T_RTP =      0b0101;	//internal read command to precharge delay(in DDR clock cycles)
        ddr_config.T_XSDLL =    32;	//exit self refresh and DLL lock delay (in x16 clocks)
        ddr_config.T_MOD =      12;	//MRS command update delay (in DDR clock cycles)
    }

    if (FreqMode == DDR3_1066)
    {
        printf("	FreqMode = DDR3_1066\n");
        ddr_config.T_RDDATA_EN_BC4 = 12;
        ddr_config.T_RDDATA_EN_BL8 = 12;

        ddr_config.CL_PHY =     8;
        ddr_config.CL_MC =      0b100;
        ddr_config.CL_CHIP =    0b100;

        ddr_config.CWL_PHY =    6;
        ddr_config.CWL_MC =     0b000;
        ddr_config.CWL_CHIP =   0b001;

        ddr_config.AL_PHY =     7;
        ddr_config.AL_MC =      0b01;
        ddr_config.AL_CHIP =    0b01;

        ddr_config.WR =         0b100; //8
        ddr_config.T_REFI =     519; //  1.95us/1.875ns/2
        ddr_config.T_RFC_XPR =  72; //MC_CLOCK = 3750
        ddr_config.T_RCD =      0b1110;
        ddr_config.T_RP =       0b1110;
        ddr_config.T_RC =       30;
        ddr_config.T_RAS =      22;
        ddr_config.T_RRD =      4;
        ddr_config.T_WTR =      0b0100; //4
        ddr_config.T_RTP =      0b0100;
        ddr_config.T_XSDLL =    32; //dfi_clk_1x = 3750
        ddr_config.T_MOD =      12;
    }
    /*
    if (FreqMode == DDR3_800)
    {
        ddr_config.T_RDDATA_EN_BC4 = 8;
        ddr_config.T_RDDATA_EN_BL8 = 8;

        ddr_config.CL_PHY =     6;
        ddr_config.CL_MC =      0b010;
        ddr_config.CL_CHIP =    0b010;

        ddr_config.CWL_PHY =    5;
        ddr_config.CWL_MC =     0b000;
        ddr_config.CWL_CHIP =   0b001;

        ddr_config.AL_PHY =     5;
        ddr_config.AL_MC =      0b01;
        ddr_config.AL_CHIP =    0b01;

        ddr_config.WR =         0b110;
        ddr_config.T_REFI =     9999;
        ddr_config.T_RFC_XPR =  54;
        ddr_config.T_RCD =      0b1100;
        ddr_config.T_RP =       0b1100;
        ddr_config.T_RC =       21;
        ddr_config.T_RAS =      15;
        ddr_config.T_WTR =      0b0011;
        ddr_config.T_RTP =      0b0011;
        ddr_config.T_XSDLL =    32;
        ddr_config.T_MOD =      6;
    }
    */
};

//-------------------------------------------------MISCELLANEOUS FUNCTIONS-------------------------------------------------

uint8_t ddr_get_T_SYS_RDLAT(const uint32_t baseAddr)
{
    printf("base 0x%08x\n", baseAddr);
    return (uint8_t)(ddr34lmc_dcr_read_DDR34LMC_DBG0(baseAddr) >> 16) & 0b111111;
}

uint32_t ddr_get_ecc_error_status(const uint32_t baseAddr, const uint32_t portNum)
{
    switch(portNum)
    {
    case 0: return ddr34lmc_dcr_read_DDR34LMC_ECCERR_PORT0(baseAddr);
    case 1: return ddr34lmc_dcr_read_DDR34LMC_ECCERR_PORT1(baseAddr);
    case 2: return ddr34lmc_dcr_read_DDR34LMC_ECCERR_PORT2(baseAddr);
    case 3: return ddr34lmc_dcr_read_DDR34LMC_ECCERR_PORT3(baseAddr);
    default: TEST_ASSERT(0, "Unexpected port number"); return 0;
    }
}

void ddr_clear_ecc_error_status(const uint32_t baseAddr, const uint32_t portNum)
{
    switch(portNum)
    {
    case 0: ddr34lmc_dcr_clear_DDR34LMC_ECCERR_PORT0(baseAddr, 0xffffffff); break;
    case 1: ddr34lmc_dcr_clear_DDR34LMC_ECCERR_PORT1(baseAddr, 0xffffffff); break;
    case 2: ddr34lmc_dcr_clear_DDR34LMC_ECCERR_PORT2(baseAddr, 0xffffffff); break;
    case 3: ddr34lmc_dcr_clear_DDR34LMC_ECCERR_PORT3(baseAddr, 0xffffffff); break;
    default: TEST_ASSERT(0, "Unexpected port number");
    }
}

//-------------------------------------------------SELF REFRESH FUNCTIONS-------------------------------------------------

void ddr_enter_self_refresh_mode(const uint32_t baseAddr)
{
    uint32_t mcstat_reg = 0;
    (void)mcstat_reg; // prevent a warning

    mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
    TEST_ASSERT(!(mcstat_reg & reg_field(1, 0b1)), "DDR34LMC already in self-refresh mode");
    TEST_ASSERT(mcstat_reg & reg_field(2, 0b1), "DDR34LMC must be in IDLE state");

    uint32_t mcopt2_reg = 0;

    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            mcopt2_reg | reg_field(0, 0b1)); //SELF_REF_EN = 1
    //wait tRP + tRFC = 13750ps + 260000ps
    usleep(1); //let's wait 1us

    mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
    TEST_ASSERT(mcstat_reg & reg_field(1, 0b1), "Failed to enter self-refresh mode");

    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    TEST_ASSERT(!(mcopt2_reg & reg_field(3, 0b1)), "DDR34LMC must be disabled at this point");
}

void ddr_exit_self_refresh_mode(const uint32_t baseAddr)
{
    uint32_t mcstat_reg = 0;

    mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
    TEST_ASSERT(mcstat_reg & reg_field(1, 0b1), "DDR34LMC is not in self-refresh mode");

    uint32_t mcopt2_reg = 0;

    mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    mcopt2_reg &= ~reg_field(0, 0b1); //clear SELF_REF_EN
    mcopt2_reg &= ~reg_field(1, 0b1); //clear XSR_PREVENT
    mcopt2_reg |= reg_field(2, 0b1); //set INIT_START
    mcopt2_reg |= reg_field(3, 0b1); //set MC_ENABLE
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr, mcopt2_reg);

    //wait a little tRFC_XPR + tXSDLL = 260000ps + 16*16*2500ps =  260000ps + 640000ps = 900000ps
    usleep(2); //let's wait 2us

    const uint32_t INIT_COMPLETE_TIMEOUT = 1000;
    uint32_t time = 0;

    do
    {
        mcstat_reg = ddr34lmc_dcr_read_DDR34LMC_MCSTAT(baseAddr);
        if (mcstat_reg & reg_field(0, 0b1)) // INIT_COMPLETE
            break;
    }
    while (++time != INIT_COMPLETE_TIMEOUT);

    TEST_ASSERT(time != INIT_COMPLETE_TIMEOUT, "DDR INIT_COMPLETE_TIMEOUT");
    TEST_ASSERT(!(mcstat_reg & reg_field(1, 0b1)), "Failed to exit self-refresh mode");
}

//-------------------------------------------------LOW POWER MODE FUNCTIONS-------------------------------------------------

// void ddr_enter_low_power_mode ()
// {
//     printf ("Enter LowPower mode....................\n");
// 
//     uint32_t reg;
//     crg_remove_writelock ();
// 
//     printf ("Enter self_refresh mode............\n");
//     ddr_enter_self_refresh_mode(EM0_DDR3LMC_DCR);
//     printf ("Enter self_refresh mode............done\n");
// 
//    //isolation cells on
//     printf ("Isolation cells ON.................\n");
//     reg = pmu_read_PMU_PWR_EM0 (PMU_BASE);
//     reg |= (1<<0);
//     pmu_write_PMU_PWR_EM0 (PMU_BASE, reg); //set ISO_ON_EM0 in PWR_EM0
//     TEST_ASSERT((pmu_read_PMU_PWR_EM0 (PMU_BASE) & (1<<0)) , "Error writing to PMU_PWR_EM0");
//    //
// 
//    //stop clocks: I_DDR34LMC_CLKX1, I_DCR_CLOCK, I_PLB6_CLOCK
//     printf ("Stop I_DDR34LMC_CLKX1..............\n");
//     reg = crg_ddr_read_CRG_DDR_CKEN_DDR3_REFCLK (CRG_DDR_BASE);
//     reg &= ~((1<<0) | (1<<2));
//     crg_ddr_write_CRG_DDR_CKEN_DDR3_REFCLK (CRG_DDR_BASE, reg); //clear DDR3_EM0_PHY_CLKEN in CKEN_DDR3;
// 
//     printf ("Stop I_DCR_CLOCK...................\n");
//     reg = crg_cpu_read_CRG_CPU_CKEN_DCR (CRG_CPU_BASE);
//     reg &= ~(1<<0);
//     crg_cpu_write_CRG_CPU_CKEN_DCR (CRG_CPU_BASE, reg); //clear DDR3_EM0_DCRCLKEN in CKEN_DCR
// 
//     printf ("Stop I_PLB6_CLOCK..................\n");
//     reg = crg_cpu_read_CRG_CPU_CKEN_L2C_PLB6 (CRG_CPU_BASE);
//     reg &= ~(1<<6);
//     crg_cpu_write_CRG_CPU_CKEN_L2C_PLB6 (CRG_CPU_BASE, reg); //clear DDR3_EM0_DCRCLKEN in CKEN_PLB6
// 
//     crg_ddr_upd_cken ();
//     crg_cpu_upd_cken ();
//    //
// 
//     printf ("Set PWR_OFF_EM0....................\n");
//     reg = pmu_read_PMU_PWR_EM0 (PMU_BASE);
//     reg |= (1<<1);
//     pmu_write_PMU_PWR_EM0 (PMU_BASE, reg); //set PWR_OFF_EM0 in PWR_EM0
//     TEST_ASSERT((pmu_read_PMU_PWR_EM0 (PMU_BASE) & (1<<1)) , "Error writing to PMU_PWR_EM0");
// 
//     crg_set_writelock();
//     printf ("Enter LowPower mode....................done\n");
// }

// void ddr_exit_low_power_mode ()
// {
//     printf ("Exit LowPower mode....................\n");
// 
//     uint32_t reg;
//     crg_remove_writelock();
// 
//     printf ("Clear PWR_OFF_EM0.................\n");
//     reg = pmu_read_PMU_PWR_EM0 (PMU_BASE);
//     reg &= ~(1<<1);
//     pmu_write_PMU_PWR_EM0 (PMU_BASE, reg); //clear PWR_OFF_EM0 in PWR_EM0
//     TEST_ASSERT((!(pmu_read_PMU_PWR_EM0 (PMU_BASE) & (1<<1))), "Error writing to PMU_PWR_EM0");
// 
//     //*****I_DDR34LMC_RESET, I_DDR34LMC_SELFREF_RESET, I_DCR_RESET, I_PLB6_RESET
//      printf ("Set RESET.........................\n");
//      reg = crg_ddr_read_CRG_DDR_RST_EN (CRG_DDR_BASE);
//      reg &= ~((1<<2) | (1<<4) | (1<<15) | (1<<16));
//      crg_ddr_write_CRG_DDR_RST_EN (CRG_DDR_BASE, reg);
// 
//     //******
// 
//    //start clocks: I_DDR34LMC_CLKX1, I_DCR_CLOCK, I_PLB6_CLOCK
//     printf ("Start I_DDR34LMC_CLKX1............\n");
//     reg = crg_ddr_read_CRG_DDR_CKEN_DDR3_REFCLK (CRG_DDR_BASE);
//     reg |= (1<<0) | (1<<2);
//     crg_ddr_write_CRG_DDR_CKEN_DDR3_REFCLK (CRG_DDR_BASE, reg); //set DDR3_EM0_PHY_CLKEN in CKEN_DDR3
// 
//     printf ("Start I_DCR_CLOCK.................\n");
//     reg = crg_cpu_read_CRG_CPU_CKEN_DCR (CRG_CPU_BASE);
//     reg |= (1<<0);
//     crg_cpu_write_CRG_CPU_CKEN_DCR (CRG_CPU_BASE, reg); //set DDR3_EM0_DCRCLKEN in CKEN_DCR
// 
//     printf ("Start I_PLB6_CLOCK................\n");
//     reg = crg_cpu_read_CRG_CPU_CKEN_L2C_PLB6 (CRG_CPU_BASE);
//     reg |= (1<<6);
//     crg_cpu_write_CRG_CPU_CKEN_L2C_PLB6 (CRG_CPU_BASE, reg); //set DDR3_EM0_DCRCLKEN in CKEN_PLB6
// 
//     crg_ddr_upd_cken ();
//     crg_cpu_upd_cken ();
//     usleep(4); //workaround
// 
//    //
//     printf ("Clear RESET.......................\n");
//     reg = crg_ddr_read_CRG_DDR_RST_EN (CRG_DDR_BASE);
//     reg |= (1<<2) | (1<<4) | (1<<15) | (1<<16);
//     crg_ddr_write_CRG_DDR_RST_EN (CRG_DDR_BASE, reg);
// 
//    //isolation cells off
//     printf ("Clear ISO_ON_EM0..................\n");
//     reg = pmu_read_PMU_PWR_EM0 (PMU_BASE);
//     reg &= ~(1<<0);
//     pmu_write_PMU_PWR_EM0 (PMU_BASE, reg); //clear ISO_ON_EM0 in PWR_EM0
//     TEST_ASSERT((!(pmu_read_PMU_PWR_EM0 (PMU_BASE) & (1<<0))) , "Error writing to PMU_PWR_EM0");
//    //
// 
//     crg_set_writelock();
//     printf ("Exit LowPower mode....................done\n");
// }

//-------------------------------------------------DRAM CLK FUNCTIONS-------------------------------------------------

void ddr_enable_dram_clk(DdrHlbId hlbId)
{
    uint32_t baseAddr = 0;
    switch (hlbId)
    {
    case DdrHlbId_Em0: baseAddr = EM0_DDR3LMC_DCR; break;
    //case DdrHlbId_Em1: baseAddr = EM1_DDR3LMC_DCR; break;
    default: TEST_ASSERT(0, "Not supported");
    }

    uint32_t mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            mcopt2_reg & ~(reg_field(7, 0b1111))); //MCOPT2[4:7] = 0b0000 CLK_DISABLE
}

void ddr_disable_dram_clk(DdrHlbId hlbId)
{
    uint32_t baseAddr = 0;
    switch (hlbId)
    {
    case DdrHlbId_Em0: baseAddr = EM0_DDR3LMC_DCR; break;
    //case DdrHlbId_Em1: baseAddr = EM1_DDR3LMC_DCR; break;
    default: TEST_ASSERT(0, "Not supported");
    }

    uint32_t mcopt2_reg = ddr34lmc_dcr_read_DDR34LMC_MCOPT2(baseAddr);
    ddr34lmc_dcr_write_DDR34LMC_MCOPT2(baseAddr,
            mcopt2_reg | reg_field(7, 0b1111)); //MCOPT2[4:7] = 0b1111 CLK_DISABLE
}

//-------------------------------------------------CRG FUNCTIONS-------------------------------------------------



static void crg_ddr_config_freq(uint32_t phy_base_addr, DdrBurstLength burstLength)
{
    // uint32_t reg;
    // uint32_t i = 0, timeout = 1000;

    crg_remove_writelock();

/*
    crg_ddr_write_CRG_DDR_PLL_PRDIV (CRG_DDR_BASE, 0x01);
    crg_ddr_write_CRG_DDR_PLL_FBDIV (CRG_DDR_BASE, 0x40);
    crg_ddr_write_CRG_DDR_PLL_PSDIV (CRG_DDR_BASE, 0x01);
    crg_ddr_write_CRG_DDR_PLL_CTRL (CRG_DDR_BASE, 1);

    reg = crg_ddr_read_CRG_DDR_PLL_CTRL(CRG_DDR_BASE); // waiting dividers update
    while ((reg & (1<<0)) && (i < timeout)) {
        reg = crg_ddr_read_CRG_DDR_PLL_CTRL(CRG_DDR_BASE);
        i++;
    };
    printf("PLL reset %d\n", timeout);

    timeout = 1000;
    reg = crg_ddr_read_CRG_DDR_PLL_STATE(CRG_DDR_BASE); 
    while (!(reg & (1<<0)) && (i < timeout)) {
        reg = crg_ddr_read_CRG_DDR_PLL_STATE(CRG_DDR_BASE);
        i++;
    };
    printf("PLL wait %d\n", timeout);

    crg_remove_writelock();
*/
    switch (DDR_FREQ)
    {
      case DDR3_800:
          printf ("Set DDR-800 config\n");
          crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_4X (CRG_DDR_BASE, 0x00);
          break;
      case DDR3_1066:
          printf ("Set DDR-1066 config\n");
          //crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_REFCLK (CRG_DDR_BASE, 0x0B);   //  DDR3_REFCLK renamed here to DDR3_1X. How to correct this?
          crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_1X (CRG_DDR_BASE, 0x0B);
          // crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_1X (CRG_DDR_BASE, 0x09);  //  With this settings there are only random errors
          break;
      case DDR3_1333:
          printf ("Set DDR-1333 config\n");
          //crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_REFCLK (CRG_DDR_BASE, 0x0B);
          crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_1X (CRG_DDR_BASE, 0x0B);
          // crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_1X (CRG_DDR_BASE, 0x09);  //  This value better than 0x0B
          break;
      case DDR3_1600:
          printf ("Set DDR-1600 config\n");
          //crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_REFCLK (CRG_DDR_BASE, 0x07);
          crg_ddr_write_CRG_DDR_CKDIVMODE_DDR3_1X (CRG_DDR_BASE, 0x07);
          break;
    }
    wait_some_time ();
    crg_ddr_upd_ckdiv();
    wait_some_time ();
    //reg = 0x01;
    //crg_ddr_write_CRG_DDR_PLL_CTRL (CRG_DDR_BASE, reg); //restart PLL

    //uint32_t i = 0, timeout = 1000;
    //reg = crg_ddr_read_CRG_DDR_PLL_CTRL(CRG_DDR_BASE); //waiting restart
    //while ((reg & (1<<0)) && (i < timeout))
    //{
    //    reg = crg_ddr_read_CRG_DDR_PLL_CTRL(CRG_DDR_BASE);
    //    i++;
    //};

    crg_set_writelock();

    //TEST_ASSERT(i < timeout,"RESTART PHY PLL FAILED (TIMEOUT)!\n");

    printf("Init DDR3PHY\n");

    ddr3phy_init(phy_base_addr, burstLength);

}

// static void crg_cpu_upd_cken(void)
// {
//     uint32_t reg;
//     reg = crg_cpu_read_CRG_CPU_UPD_CK (CRG_CPU_BASE);
//     reg |= (1<<4);
//     crg_cpu_write_CRG_CPU_UPD_CK (CRG_CPU_BASE, reg);
// }

// static void crg_ddr_upd_cken (void)
// {
//     uint32_t reg;
//     reg = crg_ddr_read_CRG_DDR_UPD_CK (CRG_DDR_BASE);
//     reg |= (1<<4);
//     crg_ddr_write_CRG_DDR_UPD_CK (CRG_DDR_BASE, reg);
// }

// static void crg_cpu_upd_ckdiv(void)
// {
//     uint32_t reg;
//     reg = crg_cpu_read_CRG_CPU_UPD_CK (CRG_CPU_BASE);
//     reg |= (1<<0);
//     crg_cpu_write_CRG_CPU_UPD_CK (CRG_CPU_BASE, reg);
// }

static void crg_ddr_upd_ckdiv(void)
{
    uint32_t reg;
    reg = crg_ddr_read_CRG_DDR_UPD_CK (CRG_DDR_BASE);
    reg |= (1<<0);
    crg_ddr_write_CRG_DDR_UPD_CK (CRG_DDR_BASE, reg);
    
    uint32_t i = 0, timeout = 1000;
    reg = crg_ddr_read_CRG_DDR_UPD_CK(CRG_DDR_BASE); // waiting dividers update
    while ((reg & (1<<0)) && (i < timeout)) {
        reg = crg_ddr_read_CRG_DDR_UPD_CK(CRG_DDR_BASE);
        i++;
    };
}

void crg_remove_writelock ()
{
    //remove write lock
    const bool crg_ddr_write_access = (0 == crg_ddr_read_CRG_DDR_WR_LOCK(CRG_DDR_BASE));
    if (!crg_ddr_write_access)
    {
        //temporarily remove write lock
        crg_ddr_write_CRG_DDR_WR_LOCK(CRG_DDR_BASE, 0x1ACCE551);
    }

    const bool crg_cpu_write_access = (0 == crg_cpu_read_CRG_CPU_WR_LOCK(CRG_CPU_BASE));
    if (!crg_cpu_write_access)
    {
        //temporarily remove write lock
        crg_cpu_write_CRG_CPU_WR_LOCK(CRG_CPU_BASE, 0x1ACCE551);
    }
   //
}

static void crg_set_writelock(void)
{
    //set write lock back
    const bool crg_cpu_write_access = (0 == crg_ddr_read_CRG_DDR_WR_LOCK(CRG_DDR_BASE));
    if (!crg_cpu_write_access)
    {
        //set write lock back
        crg_cpu_write_CRG_CPU_WR_LOCK(CRG_CPU_BASE, 0); //any value except 0x1ACCE551
    }

    const bool crg_ddr_write_access = (0 == crg_cpu_read_CRG_CPU_WR_LOCK(CRG_CPU_BASE));
    if (!crg_ddr_write_access)
    {
        //set write lock back
        crg_ddr_write_CRG_DDR_WR_LOCK(CRG_DDR_BASE, 0); //any value except 0x1ACCE551
    }
   //
}


void crg_ddr_reset_ddr_0 ()
{
    uint32_t reg;
    
    crg_remove_writelock ();
    
    reg = crg_ddr_read_CRG_DDR_RST_EN (CRG_DDR_BASE);
    //       EM0_MC_RESET
    //                EM0_PHY_SYSRESET  
    //                         EM0_PHY_PRESET
    reg &= ~((1<<2) | (1<<1) | (1<<0));
    //  Set reset
    crg_ddr_write_CRG_DDR_RST_EN (CRG_DDR_BASE, reg);
    
    wait_some_time ();
    
    reg = crg_ddr_read_CRG_DDR_RST_EN (CRG_DDR_BASE);
    reg |= ((1<<2) | (1<<1) | (1<<0));
    //  Clear reset
    crg_ddr_write_CRG_DDR_RST_EN (CRG_DDR_BASE, reg);
    
    wait_some_time ();
    
    crg_set_writelock ();
}

void crg_ddr_reset_ddr_1 ()
{
    uint32_t reg;
    
    crg_remove_writelock ();
    
    reg = crg_ddr_read_CRG_DDR_RST_EN (CRG_DDR_BASE);
    //      EM1_MC_RESET
    //               EM1_PHY_SYSRESET  
    //                        EM1_PHY_PRESET
    reg &= ~((1<<7) | (1<<6) | (1<<5));
    //  Set reset
    crg_ddr_write_CRG_DDR_RST_EN (CRG_DDR_BASE, reg);
    
    wait_some_time ();
    
    reg = crg_ddr_read_CRG_DDR_RST_EN (CRG_DDR_BASE);
    reg |= ((1<<7) | (1<<6) | (1<<5));
    //  Clear reset
    crg_ddr_write_CRG_DDR_RST_EN (CRG_DDR_BASE, reg);
    
    wait_some_time ();
    
    crg_set_writelock ();
}
