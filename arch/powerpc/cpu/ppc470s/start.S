#include <asm-offsets.h>
#include <config.h>
#include <mpc83xx.h>
#include <version.h>
#include <ppc_asm.tmpl>
#include <ppc_defs.h>
#include <asm/cache.h>
#include <asm/mmu.h>
#include <asm/u-boot.h>
#include <asm/arch/ppc470s_regs.h>


.macro  load_const_link rD, constant
    lis     \rD, \constant@h
    ori     \rD, \rD, \constant@l
.endm

.global board_init_f

.section ".text.start","ax",@progbits
.global _start
_start:

// setup stack
#ifdef CONFIG_SPL_BUILD
#ifdef CONFIG_SPL_STACK
	load_const_link sp, CONFIG_SPL_STACK
#endif
#else
	load_const_link sp, RCM_PPC_STACK
#endif

// fix FPU issue
#ifdef CONFIG_SPL_BUILD
	bl _fix_fpu
#endif

clear_bss:
    /*
    * Now clear BSS segment
    */
    load_const_link  r3,__bss_start
    load_const_link  r4,__bss_end

    cmplw   0, r3, r4
    beq     6f

    li      r0, 0
5:
    stw     r0, 0(r3)
    addi    r3, r3, 4
    cmplw   0, r3, r4
    bne     5b
6:
#ifndef CONFIG_SPL_BUILD
    /* now copy data to new place */
    load_const_link r3, CONFIG_SYS_INIT_RAM_ADDR
    load_const_link r4, __init_start
    load_const_link r5, __init_size
    bl memcpy
#endif 

	bl	cpu_init_early_f

    load_const_link r3, (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_RAM_SIZE)

    bl board_init_f_alloc_reserve
    ori r2, r3, r3
    bl board_init_f_init_reserve

	bl	cpu_init_f	/* return boot_flag for calling board_init_f */
	bl	board_init_f
	bl	board_init_r    
    isync

# ASTRO TODO
.global relocate_code
relocate_code:
    b relocate_code

endless_loop:
    b endless_loop

.global crash
crash:
    div 0,0,0


    .globl	dcache_enable
dcache_enable:
    blr

    .globl	dcache_disable
dcache_disable:
    blr

    .globl	dcache_status
dcache_status:
    blr

   .globl  trap_init
trap_init:
// all traps  initiated by romboot
    blr

.global _invalidate_tlb_entry
_invalidate_tlb_entry:
	// set TID, TS according input arg
	li r12, 0
	mtspr SPR_MMUCR, r4

	tlbsx. r5, r0, r3      // [r3] - EA, [r5] - Way, Index
	bne    ite_end         // branch if not found
	oris   r5, r5, 0x8000  // r5[0]=1, r5[4]=0
	tlbwe  r3, r5, 0       // [r3] - EA[0:19], V[20], [r5]- Way[1:2], Index[8:15], index is NU
ite_end:
	or     r3, r5, r5      // return [r3]
	
	isync
	msync
	blr

.global _write_tlb_entry
_write_tlb_entry:
	// set TID, TS according input arg
	li r12, 0
	mtspr SPR_MMUCR, r6

	tlbwe r3, r12, 0
	tlbwe r4, r12, 1
	tlbwe r5, r12, 2

	isync
	msync
	blr

.global _read_tlb_entry
_read_tlb_entry:
	// set TID, TS according input arg
	mtspr SPR_MMUCR, r5

	tlbsx. r6, r0, r3  // [r3] - EA, [r6] - Way, Index
	bne    rte_nf      // branch if not found
	
	mtspr SPR_MMUCR, r5
	tlbre r3, r6, 0
	stw   r3, 0(r4)
	tlbre r3, r6, 1
	stw   r3, 4(r4)
	tlbre r3, r6, 2
	stw   r3, 8(r4)

	add r3, r0, r6
	blr
	
rte_nf:
	addi r3, r0, 0
	blr


#ifdef CONFIG_SPL_BUILD
_fix_fpu:
    mfmsr r3
    ori r3, r3, 0x2000
    mtmsr r3
    isync
    load_const_link  r3,__doublezero
    lfs f0,0(r3)
    lfs f1,0(r3)
    lfs f2,0(r3)
    lfs f3,0(r3)
    lfs f4,0(r3)
    lfs f5,0(r3)
    lfs f6,0(r3)
    lfs f7,0(r3)
    lfs f8,0(r3)
    lfs f9,0(r3)
    lfs f10,0(r3)
    lfs f11,0(r3)
    lfs f12,0(r3)
    lfs f13,0(r3)
    lfs f14,0(r3)
    lfs f15,0(r3)
    lfs f16,0(r3)
    lfs f17,0(r3)
    lfs f18,0(r3)
    lfs f19,0(r3)
    lfs f20,0(r3)
    lfs f21,0(r3)
    lfs f22,0(r3)
    lfs f23,0(r3)
    lfs f24,0(r3)
    lfs f25,0(r3)
    lfs f26,0(r3)
    lfs f27,0(r3)
    lfs f28,0(r3)
    lfs f29,0(r3)
    lfs f30,0(r3)
    lfs f31,0(r3)
 
    load_const_link r4, ~0x2000
    mfmsr r3
    and r3, r3, r4
    mtmsr r3
 
    blr
 
__doublezero:
    .double 0.0
#endif // CONFIG_SPL_BUILD

#ifdef CONFIG_SPL_BUILD
#ifdef CONFIG_TARGET_1888TX018
.section ".text.start_secondary","ax",@progbits
.global _start_secondary
_start_secondary:
#ifdef RCM_PPC_SPL_STACK_SECONDARY
	load_const_link sp, RCM_PPC_SPL_STACK_SECONDARY
#endif

	bl _fix_fpu
	bl process_spintable

#endif // CONFIG_TARGET_1888TX018
#endif // CONFIG_SPL_BUILD
