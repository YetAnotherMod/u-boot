#ifndef TLB47X_H_
#define TLB47X_H_

//#include <common.h>

typedef enum 
{
	TLBSID_4K   = 0x00,
	TLBSID_16K  = 0x01,
	TLBSID_64K  = 0x03,
	TLBSID_1M   = 0x07,
	TLBSID_16M  = 0x0F,
	TLBSID_256M = 0x1F,
	TLBSID_1G   = 0x3F,
	TLBSID_ERR  = 0xFF
} tlb_size_id;

typedef enum
{
  TLB_MODE_NONE = 0x0,
  TLB_MODE_R  = 0x1,
  TLB_MODE_W  = 0x2,
  TLB_MODE_RW = 0x3,
  TLB_MODE_X  = 0x4,
  TLB_MODE_RX = 0x5,
  TLB_MODE_WX  = 0x6,
  TLB_MODE_RWX = 0x7
} tlb_rwx_mode;

typedef enum
{
  MEM_WINDOW_SHARED = 0x0000,
  MEM_WINDOW_0    = ( 0x8000 | 0 ),
  MEM_WINDOW_1    = ( 0x8000 | 1 ),
  MEM_WINDOW_2    = ( 0x8000 | 2 ),
  MEM_WINDOW_3    = ( 0x8000 | 3 ),
  MEM_WINDOW_4    = ( 0x8000 | 4 ),
  MEM_WINDOW_5    = ( 0x8000 | 5 ),
  MEM_WINDOW_6    = ( 0x8000 | 6 ),
  MEM_WINDOW_7    = ( 0x8000 | 7 ),
  MEM_WINDOW_8    = ( 0x8000 | 8 ),
  MEM_WINDOW_9    = ( 0x8000 | 9 ),
  MEM_WINDOW_10   = ( 0x8000 | 10 ),
  MEM_WINDOW_11   = ( 0x8000 | 11 ),
  MEM_WINDOW_12   = ( 0x8000 | 12 ),
  MEM_WINDOW_13   = ( 0x8000 | 13 ),
  MEM_WINDOW_14   = ( 0x8000 | 14 ),
  MEM_WINDOW_15   = ( 0x8000 | 15 )
} mem_window_t;

typedef struct 
{
  uint32_t epn   : 20;  // [0:19]
  uint32_t v     : 1;   // [20]
  uint32_t ts    : 1;   // [21]
  uint32_t dsiz  : 6;   // [22:27]  set 0x0, 0x1, 0x3, 0x7, 0xF, 0x1F, 0x3F (4K, 16K, 64K, 1M, 16M, 256M, 1GB) 
  uint32_t blank : 4;   // [28:31]
} tlb_entr_0;

typedef union
{
	tlb_entr_0 entr;
	uint32_t   data;
} tlb_entr_data_0;

typedef struct 
{
  uint32_t rpn   : 20;  // [0:19]
  uint32_t blank : 2;   // [20:21]
  uint32_t erpn  : 10;  // [22:31]
} tlb_entr_1;

typedef union
{
	tlb_entr_1 entr;
	uint32_t   data;
} tlb_entr_data_1;

typedef struct 
{
  uint32_t blank1 : 14;  // [0:13]
  uint32_t il1i   : 1;   // [14]
  uint32_t il1d   : 1;   // [15]
  uint32_t u      : 4;   // [16:19]
  uint32_t wimg   : 4;   // [20:23]
  uint32_t e      : 1;   // [24]  0-BE, 1-LE
  uint32_t blank2 : 1;   // [25]
  uint32_t uxwr   : 3;   // [26:28]
  uint32_t sxwr   : 3;   // [29:31]
} tlb_entr_2;

typedef union
{
	tlb_entr_2 entr;
	uint32_t   data;
} tlb_entr_data_2;

#define SPR_PID         48

/*SPR access*/
#define spr_write( spr_reg, value )\
    asm volatile (\
        "mtspr %1, %0 \n\t"\
        ::"r"(value), "i"(spr_reg)\
    )

#define spr_read( spr_reg ) ({\
    uint32_t rval = 0;\
    asm volatile (\
        "mfspr %0, %1 \n\t"\
        :"=r"(rval)\
        :"i"(spr_reg)\
    );\
    rval;\
})

inline void set_mem_window( mem_window_t const window ) {
    spr_write( SPR_PID, window );
    asm volatile (
        "isync \n\t"
    );
}

inline mem_window_t get_mem_window(void) {
    return ( mem_window_t )spr_read( SPR_PID );
}

uint32_t _invalidate_tlb_entry(uint32_t tlb, uint32_t mmucr);
void _write_tlb_entry(uint32_t tlb0, uint32_t tlb1, uint32_t tlb2, uint32_t mmucr);
int _read_tlb_entry(uint32_t ea, uint32_t * tlb, uint32_t mmucr);

void tlb47x_inval(uint32_t cpu_adr, tlb_size_id tlb_sid);

void tlb47x_map_entry(uint64_t physical, uint32_t logical, uint32_t il1i, uint32_t il1d, uint32_t wimg, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode, mem_window_t window);

void tlb47x_map_nocache(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode);
void tlb47x_map_guarded(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode);
void tlb47x_map_cached(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode);
void tlb47x_map_coherent(uint64_t physical, uint32_t logical, tlb_size_id size, tlb_rwx_mode umode, tlb_rwx_mode smode);

#endif