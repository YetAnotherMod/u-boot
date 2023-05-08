#include <common.h>
#include <asm/tlb47x.h>


int checkcpu(void)
{
    uint pvr = get_pvr();

    puts("CPU:   ");
    if(pvr == 0x7ff520c0)
        printf("RC Module PowerPC 476FP core\n");
    else
        printf("Unknown, PVR: %08X\n", pvr);

    return 0;
}

int print_cpuinfo(void)
{
	return checkcpu();
}

int tlb47x_read_entry(uint32_t cpu_adr, uint32_t *valid, uint32_t *tsize, unsigned long *logical, phys_addr_t *physical)
{
	uint32_t tlb[3] = { 0, 0, 0 };
	uint32_t ea = (cpu_adr >> 12);	

	if (_read_tlb_entry(ea, tlb, MEM_WINDOW_SHARED))
	{
		tlb_entr_0 *tlb0 = (tlb_entr_0 *) &tlb[0];
		tlb_entr_1 *tlb1 = (tlb_entr_1 *) &tlb[1];
		*valid = tlb0->v;
		*tsize = tlb47x_get_tlb_sid_size(tlb0->dsiz);
		*logical = tlb0->epn << 12;
		*physical = ((phys_addr_t)(tlb1->rpn) << 12) + ((phys_addr_t)(tlb1->erpn) << 32);
		return 0;
	}
	return -1;
}

void tlb47x_inval(uint32_t cpu_adr, tlb_size_id tlb_sid)
{
	tlb_entr_data_0 tlb_0;
	tlb_0.data = 0x00000000;
	tlb_0.entr.epn = (cpu_adr >> 12);
	tlb_0.entr.v = 0;
	tlb_0.entr.ts = 0;
	tlb_0.entr.dsiz = tlb_sid;
		
	_invalidate_tlb_entry(tlb_0.data, 0);
}

void tlb47x_map_entry(uint64_t physical, uint32_t logical, uint32_t il1i, uint32_t il1d, uint32_t wimg, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode, mem_window_t window)
{
	tlb_entr_data_0 tlb_0;
	tlb_0.data = 0x00000000;
    tlb_0.entr.epn = (logical >> 12); 
    tlb_0.entr.v = 1;
    tlb_0.entr.ts = 0;
    tlb_0.entr.dsiz = size; 

	tlb_entr_data_1 tlb_1;
	tlb_1.data = 0x00000000;
    tlb_1.entr.rpn = (uint32_t)((physical>> 12) & 0xFFFFFFFF);
    tlb_1.entr.erpn = (physical>>32) & 0x3F;

	tlb_entr_data_2 tlb_2;
	tlb_2.data = 0x00000000;
	tlb_2.entr.il1i = il1i;
	tlb_2.entr.il1d = il1d;
    tlb_2.entr.u = 0;
    tlb_2.entr.wimg = wimg;
    tlb_2.entr.e = 0;  // BE   
    tlb_2.entr.uxwr = umode;     
    tlb_2.entr.sxwr = smode;     
	
	_write_tlb_entry(tlb_0.data, tlb_1.data, tlb_2.data, window); 
}

void tlb47x_map_nocache(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode)
{
	tlb47x_map_entry(physical, logical, 1, 1, 0x4, size, umode, smode, MEM_WINDOW_SHARED);
}

void tlb47x_map_guarded(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode)
{
	tlb47x_map_entry(physical, logical, 1, 1, 0x5, size, umode, smode, MEM_WINDOW_SHARED);
}

void tlb47x_map_cached(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode)
{
	tlb47x_map_entry(physical, logical, 0, 0, 0x2, size, umode, smode, MEM_WINDOW_SHARED);
}

void tlb47x_map_coherent(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode)
{
	tlb47x_map_entry(physical, logical, 1, 1, 0x2, size, umode, smode, MEM_WINDOW_SHARED);
}

tlb_size_id tlb47x_get_tlb_sid_by_size(uint32_t size)
{
	if(size <= (4 * 1024)) return TLBSID_4K;
	if(size <= (16 * 1024)) return TLBSID_16K;
	if(size <= (64 * 1024)) return TLBSID_64K;
	if(size <= (1 * 1024 * 1024)) return TLBSID_1M;
	if(size <= (16 * 1024 * 1024)) return TLBSID_16M;
	if(size <= (256 * 1024 * 1024)) return TLBSID_256M;
	if(size <= (1 * 1024 * 1024 * 1024)) return TLBSID_1G;
	return TLBSID_ERR;
}

const uint32_t tlb47x_get_tlb_sid_size(tlb_size_id tlb_sid)
{
	switch ( tlb_sid ) {
	case TLBSID_4K :   return 4 * 1024;
	case TLBSID_16K :  return 16 * 1024;
	case TLBSID_64K :  return 64 * 1024;
	case TLBSID_1M :   return 1 * 1024 * 1024;
	case TLBSID_16M :  return 16 * 1024 * 1024;
	case TLBSID_256M : return 256 * 1024 * 1024;
	case TLBSID_1G :   return 1 * 1024 * 1024 * 1024;
	default :
		break;		
	}
	return 0;
}
