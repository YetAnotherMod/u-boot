#ifndef __1888TX018_H
#define __1888TX018_H

#include <linux/sizes.h>

/* Memory map for 1888tx018:
 * 
 * SPL:
 *	0x00040000 0x18    - romboot header
 *	0x00040018 0x35FE0 - start of SPL (code/data/FDT)
 *	0x00075FE0 0x2000  - heap
 *	0x00077FE0 0x4000  - core 0 initial pointer
 *	0x0007BFE0 0x4000  - core 1 initial pointer
 *	0x0007FFE0 0x20    - spin table
 *
 * Main bootloader:
 *	0x4D000000 0x40     - U-Boot header
 *	0x4D000040 0xFFFFC0 - code
 *	0x4E000000 0xF00000 - data, heap etc.
 *	0x4EF00000 0x100000 - initial stack
 */

#define RCM_1888TX018_IM0_START CONFIG_SPL_TEXT_BASE
#define RCM_1888TX018_IM0_SIZE 0x40000

#define RCM_1888TX018_SPL_SPINTABLE_SIZE 32
#define RCM_PPC_SPL_SPINTABLE (RCM_1888TX018_IM0_START + RCM_1888TX018_IM0_SIZE - RCM_1888TX018_SPL_SPINTABLE_SIZE)

#define RCM_PPC_SPL_STACK_SIZE 0x4000
#define RCM_PPC_SPL_STACK_SECONDARY RCM_PPC_SPL_SPINTABLE // bottom of the stack
#define CONFIG_SPL_STACK (RCM_PPC_SPL_STACK_SECONDARY - RCM_PPC_SPL_STACK_SIZE) // bottom of the stack

#define CONFIG_SYS_SPL_MALLOC_SIZE 0x2000
#define CONFIG_SYS_SPL_MALLOC_START (CONFIG_SPL_STACK - RCM_PPC_SPL_STACK_SIZE - CONFIG_SYS_SPL_MALLOC_SIZE)

#define RCM_1888TX018_SPL_FDT_MAX_LEN 0x1000
#define RCM_PPC_SPL_ADDR_LIMIT (CONFIG_SYS_SPL_MALLOC_START - RCM_1888TX018_SPL_FDT_MAX_LEN)


#define CONFIG_SYS_UBOOT_BASE CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_UBOOT_START CONFIG_SYS_TEXT_BASE

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SYS_INIT_RAM_ADDR CONFIG_SYS_SPL_MALLOC_START // actually it is a fake value for prevent compilation errors
#define CONFIG_SYS_INIT_RAM_SIZE (RCM_1888TX018_IM0_START + RCM_1888TX018_IM0_SIZE - CONFIG_SYS_SPL_MALLOC_START)
#else
#define CONFIG_SYS_INIT_RAM_ADDR 0x4E000000
#define CONFIG_SYS_INIT_RAM_SIZE 0x01000000
#endif

#define RCM_PPC_STACK_SIZE 0x100000
#define RCM_PPC_STACK (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_RAM_SIZE) // bottom of the stack

#define CONFIG_SYS_MONITOR_LEN SZ_256K
#define CONFIG_SYS_MALLOC_LEN (4 * 1024 * 1024)

#define CONFIG_VERY_BIG_RAM
#define CONFIG_SYS_INIT_DDR0_ADDR_PHYS (0x0000000000ULL)
#define CONFIG_SYS_INIT_DDR1_ADDR_PHYS (0x0200000000ULL)
#define CONFIG_SYS_DDR_SDRAM_BASE      0x40000000
#define CONFIG_SYS_SDRAM_BASE          CONFIG_SYS_DDR_SDRAM_BASE
#define CONFIG_SYS_DDR_SIZE            SZ_4G
#define CONFIG_MAX_MEM_MAPPED          SZ_2G

#define CONFIG_SYS_LOAD_ADDR           CONFIG_SYS_TEXT_BASE

#define CONFIG_SYS_MEMTEST_START (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_RAM_SIZE - RCM_PPC_STACK_SIZE) // top of the internal stack
#define CONFIG_SYS_MEMTEST_END (CONFIG_SYS_MEMTEST_START + SZ_256K - 1)
/*#define CONFIG_SYS_DRAM_TEST*/

#define CONFIG_SYS_MAX_FLASH_BANKS      2       // if NOR via LSIF need 1!!! (little window)+correct mtdpart
#define CONFIG_SYS_MAX_FLASH_SECT       1024
#define MAX_SRAM_BANKS                  4       // SRAM chip select number for board mb115-02

#define CONFIG_CHIP_SELECTS_PER_CTRL    2
#define CONFIG_DIMM_SLOTS_PER_CTLR      2

#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_XMODEM_SUPPORT // needed for CONFIG_SPL_XMODEM_EDCL_LOAD
#define CONFIG_SPL_XMODEM_EDCL_LOAD

#define BOOT_DEVICE_NAND 10
#define BOOT_DEVICE_SPI 11
#define BOOT_DEVICE_UART 12 // needed for BOOT_DEVICE_XMODEM_EDCL
#define BOOT_DEVICE_XMODEM_EDCL 13

#define CONFIG_SYS_SPI_U_BOOT_OFFS      0x40000
#define CONFIG_SYS_SPI_CLK 100000000

#define CONFIG_SYS_FLASH_QUIET_TEST 1

#define CONFIG_IPADDR 192.168.0.2
#define CONFIG_SERVERIP 192.168.0.1
#define CONFIG_NETMASK 255.255.255.0
#define CONFIG_HOSTNAME "tx018"
#define CONFIG_LOADADDR 50000000
#define CONFIG_EXTRA_ENV_SETTINGS \
        "baudrate=1000000\0" \
        "bootfile=tx018/uImage\0" \
        "bootm_low=08000000\0" \
        "bootm_size=04000000\0" \
        "fdt_addr_r=50f00000\0" \
        "fileaddr=50f00000\0" \
        "kernel=run setmem; run loadfdt; run loadkernel; bootm ${loadaddr} - ${fdt_addr_r}\0" \
        "kernelsd=run setmem; run loadsd; bootm ${loadaddr} - ${fdt_addr_r}\0" \
        "loadfdt=tftp ${fdt_addr_r} tx018/tx018.dtb\0" \
        "loadfdtsd=ext4load mmc 0:2 ${fdt_addr_r} /boot/uImage-tx018.dtb\0" \
        "loadkernel=tftp ${loadaddr} ${bootfile}\0" \
        "loadkernelsd=ext4load mmc 0:2 ${loadaddr} /boot/uImage-tx018.bin\0" \
        "loadsd=run loadfdtsd; run loadkernelsd\0" \
        "setmem=mmap drop all; mmap drop 0 1m; mmap set 0 256m 00000000; mmap set ${loadaddr} 16m 10000000\0" \
        "tftptimeout=1000\0" \
        "tftptimeoutcountmax=100\0" \
        "sramnortest=sramtest run rand; nortest run\0"

#define CONFIG_PL01X_SERIAL
#define CONFIG_BAUDRATE 1000000
/*#define CONFIG_DEBUG_UART_SKIP_INIT 1*/

#define CONFIG_SYS_PBSIZE 1024

/* todo - count by freq values */
#define TIMER_TICKS_PER_US  800

#define CONFIG_USB_MUSB_PIO_ONLY

#ifndef CONFIG_TFTP_BLOCKSIZE
#define CONFIG_TFTP_BLOCKSIZE		1466
#endif
#define CONFIG_TFTP_TSIZE

#define CONFIG_SYS_BOOTM_LEN 0x4000000

#define CONFIG_SPD_EEPROM

#ifndef CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_PARTITIONS
#endif

#define CONFIG_SYS_MAX_NAND_DEVICE 8
#define CONFIG_SYS_NAND_MAX_CHIPS 8

#define CONFIG_CMD_UBIFS
/*
#ifndef CONFIG_NAND
#define CONFIG_NAND 
#endif
*/
/*
#ifndef CONFIG_CMD_NAND
#define CONFIG_CMD_NAND
#endif
*/
#ifndef CONFIG_CMD_UBI
#define CONFIG_CMD_UBI
#endif

#ifndef CONFIG_RBTREE
#define CONFIG_RBTREE
#endif

/*#ifndef CONFIG_MTD_DEVICE*/
/*#define CONFIG_MTD_DEVICE*/
/*#endif*/

#ifndef CONFIG_CMD_MTDPARTS
#define CONFIG_CMD_MTDPARTS
#endif

#ifndef CONFIG_LZO
#define CONFIG_LZO
#endif

#define CONFIG_MTD_UBI_WL_THRESHOLD 4096
#define CONFIG_MTD_UBI_BEB_LIMIT 20

#define CONFIG_SYS_MMC_ENV_DEV 0

/* in board/rcm/mb115-01/Kconfig now
#ifdef CONFIG_ENV_IS_IN_NAND 
    #define CONFIG_ENV_OFFSET               0x40000
    #define CONFIG_ENV_SIZE                 0x4000
    #define CONFIG_ENV_SECT_SIZE            0x20000 
#elif defined CONFIG_ENV_IS_IN_FLASH
    #define CONFIG_ENV_OFFSET               0x1040000
    #define CONFIG_ENV_SIZE                 0x4000
    #define CONFIG_ENV_SECT_SIZE            0x40000
    #define CONFIG_SPL_ENV_SUPPORT
    #define CONFIG_TPL_ENV_SUPPORT
#elif defined CONFIG_ENV_IS_IN_MMC
    #define CONFIG_ENV_OFFSET               0x50000
    #define CONFIG_ENV_SIZE                 0x4000
    #define CONFIG_ENV_SECT_SIZE            0x10000
#else
    #define CONFIG_ENV_OFFSET               0x140000
    #define CONFIG_ENV_SIZE                 0x4000
    #define CONFIG_ENV_SECT_SIZE            0x10000
#endif
*/
#ifdef CONFIG_SPL_NAND_SUPPORT
    #define CONFIG_SYS_NAND_U_BOOT_OFFS     0x880000
    #define CONFIG_SYS_NAND_U_BOOT_SIZE     0x160000
#endif

#define CONFIG_SYS_NAND_BASE_LIST           {1}; /*v2020.01: it's dummy!!!but need...*/

#ifdef CONFIG_MTD_RCM_NOR
    /* #define CONFIG_CFI_FLASH */
    #define CONFIG_FLASH_CFI_DRIVER
    #define CONFIG_FLASH_CFI_MTD
    #define CONFIG_SYS_FLASH_CFI
    #define CONFIG_SYS_FLASH_CFI_WIDTH      FLASH_CFI_16BIT
    #define CONFIG_SYS_FLASH_EMPTY_INFO     /* flinfo show E and/or RO */
    #define CONFIG_FLASH_SHOW_PROGRESS      100
    #define CONFIG_SYS_FLASH_USE_BUFFER_WRITE
    #define CONFIG_SYS_WRITE_SWAPPED_DATA
    #ifndef CONFIG_PPC_DCR
        #define CONFIG_PPC_DCR
    #endif

    #define CONFIG_SYS_FLASH_BASE0          0x20000000      /* base address 0 for work via MCIF and LSIF */
    #define CONFIG_SYS_FLASH_BASE1          0x10000000      /* base address 1 for work via MCIF only */
    #define CONFIG_SYS_FLASH_BANKS_LIST     {CONFIG_SYS_FLASH_BASE0,CONFIG_SYS_FLASH_BASE1}
    #define CONFIG_SYS_FLASH_BASE           CONFIG_SYS_FLASH_BASE0
    #define CONFIG_SYS_MONITOR_BASE         0x40000000      /* CONFIG_SYS_FLASH_BASE */

    #ifdef CONFIG_SYS_UBOOT_BASE
        #undef CONFIG_SYS_UBOOT_BASE
    	#define CONFIG_SYS_UBOOT_BASE       0x20040000 
    #endif

#endif

#endif /* __1888TX018_H */
