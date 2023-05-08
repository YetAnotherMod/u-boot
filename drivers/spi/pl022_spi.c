// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2012
 * Armando Visconti, ST Microelectronics, armando.visconti@st.com.
 *
 * (C) Copyright 2018
 * Quentin Schulz, Bootlin, quentin.schulz@bootlin.com
 *
 * (C) Copyright 2019
 * Alexey Spirkov, Astrosoft, alexeis@astrosoft.ru
 *
 * (C) Copyright 2020
 * Vladimir Shalyt, Astrosoft, Vladimir.Shalyt@astrosoft.ru
 *
 * Driver for ARM PL022 SPI Controller.
 */

#include <config.h>
#include <clk.h>
#include <common.h>
#include <dm.h>
#include <dm/platform_data/spi_pl022.h>
#include <linux/io.h>
#include <spi.h>
#include <hang.h>

#define SSP_CR0		0x000
#define SSP_CR1		0x004
#define SSP_DR		0x008
#define SSP_SR		0x00C
#define SSP_CPSR	0x010
#define SSP_IMSC	0x014
#define SSP_RIS		0x018
#define SSP_MIS		0x01C
#define SSP_ICR		0x020
#define SSP_DMACR	0x024
#define SSP_CSR		0x030 /* vendor extension */
#define SSP_ITCR	0x080
#define SSP_ITIP	0x084
#define SSP_ITOP	0x088
#define SSP_TDR		0x08C

#ifdef CONFIG_ARCH_RCM_ARM
#define SSP_CS 		0x140
#endif

#define SSP_PID0	0xFE0
#define SSP_PID1	0xFE4
#define SSP_PID2	0xFE8
#define SSP_PID3	0xFEC

#define SSP_CID0	0xFF0
#define SSP_CID1	0xFF4
#define SSP_CID2	0xFF8
#define SSP_CID3	0xFFC

/* SSP Control Register 0  - SSP_CR0 */
#define SSP_CR0_SPO		(0x1 << 6)
#define SSP_CR0_SPH		(0x1 << 7)
#define SSP_CR0_BIT_MODE(x)	((x) - 1)
#define SSP_SCR_MIN		(0x00)
#define SSP_SCR_MAX		(0xFF)
#define SSP_SCR_SHFT		8
#define DFLT_CLKRATE		2

/* SSP Control Register 1  - SSP_CR1 */
#define SSP_CR1_MASK_SSE	(0x1 << 1)

#define SSP_CPSR_MIN		(0x02)
#define SSP_CPSR_MAX		(0xFE)
#define DFLT_PRESCALE		(0x40)

/* SSP Status Register - SSP_SR */
#define SSP_SR_MASK_TFE		(0x1 << 0) /* Transmit FIFO empty */
#define SSP_SR_MASK_TNF		(0x1 << 1) /* Transmit FIFO not full */
#define SSP_SR_MASK_RNE		(0x1 << 2) /* Receive FIFO not empty */
#define SSP_SR_MASK_RFF		(0x1 << 3) /* Receive FIFO full */
#define SSP_SR_MASK_BSY		(0x1 << 4) /* Busy Flag */

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)
#include <asm-generic/gpio.h>

#define SSP_CR0_8BIT_MODE	(0x07)
#define MAX_CS_COUNT		4
#define SSP_SPI_IRQ_MASK	0x0d8
/* SSP Interrupt mask register */
#define SSP_IMSC_TXIM_n		3
#define SSP_IMSC_RXIM_n		2
#define SSP_IMSC_RTIM_n		1
#define SSP_IMSC_RORIM_n	0

/* SSP interrupt reset register */
#define SSP_ICR_RTIC_n		1
#define SSP_ICR_RORIC_n		0

/* SSP DMA control register */
#define SSP_DMACR_TXDMAE_n	1
#define SSP_DMACR_RXDMAE_n	0

/* SSP CR1 control register */
#define SSP_CR1_SOD_n		3
#define SSP_CR1_MS_n		2
#define SSP_CR1_SSE_n		1
#define SSP_CR1_LBM_n		0

#define MAX_CS_COUNT 4
#define SPI_BUF_LEN 1024

#endif // defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)

struct pl022_spi_slave {
	void *base;
	unsigned int freq;
#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)
	struct gpio_desc cs_gpios[MAX_CS_COUNT];
#endif
#ifdef CONFIG_ARCH_RCM_ARM
	struct udevice *bus;
	unsigned int mode;
	unsigned int speed;
	unsigned char cmd[SPI_BUF_LEN];
	unsigned long cmd_len;	
#endif
};

/*
 * ARM PL022 exists in different 'flavors'.
 * This drivers currently support the standard variant (0x00041022), that has a
 * 16bit wide and 8 locations deep TX/RX FIFO.
 */
static int pl022_is_supported(struct pl022_spi_slave *ps)
{
	/* PL022 version is 0x00041022 */
	if ((readw(ps->base + SSP_PID0) == 0x22) &&
	    (readw(ps->base + SSP_PID1) == 0x10) &&
	    ((readw(ps->base + SSP_PID2) & 0xf) == 0x04) &&
	    (readw(ps->base + SSP_PID3) == 0x00))
		return 1;

	return 0;
}

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)
static int pl022_setup_gpio(struct udevice *bus)
{
	int i;
	struct pl022_spi_slave *ps = dev_get_priv(bus);
	int ret = gpio_request_list_by_name(bus, "cs-gpios", ps->cs_gpios,
										ARRAY_SIZE(ps->cs_gpios), 0);
	if (ret < 0) {
			pr_err("Can't get %s gpios! Error: %d\n", bus->name, ret);
			return ret;
	}

	for(i = 0; i < ARRAY_SIZE(ps->cs_gpios); i++) {
			if (!dm_gpio_is_valid(&ps->cs_gpios[i]))
					continue;

			dm_gpio_set_dir_flags(&ps->cs_gpios[i],
									GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);

			// and disable
			dm_gpio_set_value(&ps->cs_gpios[i], 1);
	}
	debug("Priv regs: %08x\n", (int) ps->base);
	debug("SPI ver: %02x %02x %02x %02x\n", readb(ps->base + SSP_PID0), readb(ps->base + SSP_PID1), readb(ps->base + SSP_PID2), readb(ps->base + SSP_PID3));

	return 0;
}
#endif // defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_ARCH_RCM_ARM)
static void pl022_setup_dma(void* base)
{
	debug("SPI ver is correct, setup of DMA\n");

	// disable interrupts
	u16 imsc = (0b0 << SSP_IMSC_TXIM_n)
            | (0b0 << SSP_IMSC_RXIM_n)
            | (0b0 << SSP_IMSC_RTIM_n)
            | (0b0 << SSP_IMSC_RORIM_n);

	writew(imsc, base + SSP_IMSC);

	// reset interrupts state
	u16 icr = (0b1 << SSP_ICR_RTIC_n)
            | (0b1 << SSP_ICR_RORIC_n);

	writew(icr, base + SSP_ICR);

	// disable DMA
	u16 dmacr = (0b0 << SSP_DMACR_TXDMAE_n)
            | (0b0 << SSP_DMACR_RXDMAE_n);

	writew(dmacr, base + SSP_DMACR);

	// enable SSP
	u16 cr1 = (0b1 << SSP_CR1_SOD_n)
            | (0b0 << SSP_CR1_MS_n)
            | (0b1 << SSP_CR1_SSE_n)
            | (0b0 << SSP_CR1_LBM_n);

	writew(cr1, base + SSP_CR1);

	// disable GSPI DMA
	//writel(0, ps->base + SSP_SPI_IRQ_MASK);
}
#endif // defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_ARCH_RCM_ARM)

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)
static void spi_cs_activate(struct udevice *dev, uint cs)
{
	struct udevice *bus = dev_get_parent(dev);
	struct pl022_spi_slave *priv = dev_get_priv(bus);

	if (!dm_gpio_is_valid(&priv->cs_gpios[cs])) {
#ifdef CONFIG_TARGET_1888BM18
		u32 val = readl(priv->base + 0xCC);
		val |= 0x2;
		val &= ~0x1;
		writel(val, priv->base + 0xCC);
#endif
		return;
	}

	dm_gpio_set_value(&priv->cs_gpios[cs], 0);

}

static void spi_cs_deactivate(struct udevice *dev, uint cs)
{
	struct udevice *bus = dev_get_parent(dev);
	struct pl022_spi_slave *priv = dev_get_priv(bus);

	if (!dm_gpio_is_valid(&priv->cs_gpios[cs])) {
#ifdef CONFIG_TARGET_1888BM18
		u32 val = readl(priv->base + 0xCC);
		val |= 0x3;
		writel(val, priv->base + 0xCC);
#endif
		return;
	}

	dm_gpio_set_value(&priv->cs_gpios[cs], 1);
}
#endif // defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)

static int pl022_spi_set_speed(struct udevice *bus, uint speed);
static int pl022_spi_set_mode(struct udevice *bus, uint mode);
static int pl022_spi_xfer_int(struct pl022_spi_slave *priv, unsigned int len,
							  const void *dout, void *din);

#ifdef CONFIG_ARCH_RCM_ARM
__weak int sdcard_init(void)
{
	return 0;
}
#endif

static int pl022_spi_probe(struct udevice *bus)
{
	struct pl022_spi_pdata *plat = dev_get_platdata(bus);
	struct pl022_spi_slave *ps = dev_get_priv(bus);

	ps->base = ioremap(plat->addr, plat->size);
	ps->freq = plat->freq;

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_TARGET_1888BC048)
	int ret = pl022_setup_gpio(bus);
	if (ret < 0)
	{
		pr_err("Can't get %s gpios! Error: %d\n", bus->name, ret);
		return ret;
	}
#elif defined(CONFIG_ARCH_RCM_ARM)
	ps->bus = bus;
	pl022_setup_gpio(bus);
	// sd card should be switched to SPI mode before any activity on SPI bus
	sdcard_init();
#endif

	/* Check the PL022 version */
	if (!pl022_is_supported(ps))
		return -ENOTSUPP;

	/* 8 bits per word, high polarity and default clock rate */
	writew(SSP_CR0_BIT_MODE(8), ps->base + SSP_CR0);
	writew(DFLT_PRESCALE, ps->base + SSP_CPSR);

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_ARCH_RCM_ARM)
	pl022_setup_dma(ps->base);
#endif
	return 0;
}

static void flush(struct pl022_spi_slave *ps)
{
	do {
		while (readw(ps->base + SSP_SR) & SSP_SR_MASK_RNE)
			readw(ps->base + SSP_DR);
	} while (readw(ps->base + SSP_SR) & SSP_SR_MASK_BSY);
}

static int pl022_spi_claim_bus(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct pl022_spi_slave *ps = dev_get_priv(bus);
	u16 reg;
#ifdef CONFIG_ARCH_RCM_ARM
	struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);

	// several devices on same bus with different modes requires change speed and mode on claiming
	if (slave_plat->mode != ps->mode)
	{
		pl022_spi_set_mode(bus, slave_plat->mode);
		ps->mode = slave_plat->mode;
	}
	if (slave_plat->max_hz != ps->speed)
	{
		pl022_spi_set_speed(bus, slave_plat->max_hz);
		ps->speed = slave_plat->max_hz;
	}

	writew(0x1 << slave_plat->cs, ps->base + SSP_CS);
	ps->cmd_len = 0;	
#endif
	/* Enable the SPI hardware */
	reg = readw(ps->base + SSP_CR1);
	reg |= SSP_CR1_MASK_SSE;
	writew(reg, ps->base + SSP_CR1);

	flush(ps);

	return 0;
}

static int pl022_spi_release_bus(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct pl022_spi_slave *ps = dev_get_priv(bus);
	u16 reg;
#ifdef CONFIG_ARCH_RCM_ARM
	if(ps->cmd_len)
	{
		pl022_spi_xfer_int(ps, ps->cmd_len, ps->cmd, 0);
		ps->cmd_len = 0;
	}
#endif
	flush(ps);

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_ARCH_RCM_ARM)
	{
		// release cs
		struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);
		if (dm_gpio_is_valid(&ps->cs_gpios[slave_plat->cs]))
			dm_gpio_set_value(&ps->cs_gpios[slave_plat->cs], 1);
	}
#endif
	/* Disable the SPI hardware */
	reg = readw(ps->base + SSP_CR1);
	reg &= ~SSP_CR1_MASK_SSE;
	writew(reg, ps->base + SSP_CR1);

	return 0;
}

static int pl022_spi_xfer_int(struct pl022_spi_slave *ps, unsigned int len,
							  const void *dout, void *din)
{
	unsigned int len_tx = 0, len_rx = 0;
	unsigned short value;
	int ret = 0;
	const unsigned char *txp = dout;
	unsigned char *rxp = din;

	debug("alive. len_tx = %d, len = %d\n", len_tx, len);

	// Checking: read remaining bytes
	if (readw(ps->base + SSP_SR) & SSP_SR_MASK_RNE)
		value = readw(ps->base + SSP_DR);

	while (len_tx < len) {
		if (readw(ps->base + SSP_SR) & SSP_SR_MASK_TNF) {
			value = txp ? *txp++ : 0xFF;
			writew(value, ps->base + SSP_DR);
			len_tx++;
		}

		if (readw(ps->base + SSP_SR) & SSP_SR_MASK_RNE) {
			value = readw(ps->base + SSP_DR);
			if (rxp)
				*rxp++ = value;
			len_rx++;
		}
	}

	while (len_rx < len_tx) {
		if (readw(ps->base + SSP_SR) & SSP_SR_MASK_RNE) {
			value = readw(ps->base + SSP_DR);
			if (rxp)
				*rxp++ = value;
			len_rx++;
		}
	}

	if (readw(ps->base + SSP_SR) & SSP_SR_MASK_RNE)
		value = readw(ps->base + SSP_DR);	

	return ret;
}


static int pl022_spi_xfer(struct udevice *dev, unsigned int bitlen,
			  const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct pl022_spi_slave *ps = dev_get_priv(bus);
	u32		len;
	u32		ret = 0;
#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)
	struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);
	debug("pl022_spi_xfer %d, %d, 0x%x, 0x%x, 0x%x\n", bitlen, (int)flags, (int)dout, (int)ps->base, (int)slave_plat);
#endif

	if (bitlen == 0)
		/* Finish any previously submitted transfers */
		return 0;

	/*
	 * TODO: The controller can do non-multiple-of-8 bit
	 * transfers, but this driver currently doesn't support it.
	 *
	 * It's also not clear how such transfers are supposed to be
	 * represented as a stream of bytes...this is a limitation of
	 * the current SPI interface.
	 */
	if (bitlen % 8) {
		/* Errors always terminate an ongoing transfer */
		flags |= SPI_XFER_END;
		return -1;
	}

	len = bitlen / 8;

#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)
	if (flags & SPI_XFER_BEGIN)
		spi_cs_activate(dev, slave_plat->cs);
#endif

#ifdef CONFIG_ARCH_RCM_ARM
	if (dout && (flags & SPI_XFER_BEGIN) && !(flags & SPI_XFER_END)) // sending cmd?
	{
		if (len <= SPI_BUF_LEN)
		{
			memcpy(ps->cmd, dout, len);
			ps->cmd_len = len;
			ret = 0;
		}
		else
		{
			ret = -1;
			flags |= SPI_XFER_END;
		}
	}
	else if (ps->cmd_len && (!flags || (!(flags & SPI_XFER_BEGIN) && (flags & SPI_XFER_END)))) // getting cmd result?
	{
		unsigned char *newout = 0;
		unsigned char *newin = 0;

		newout = malloc(ps->cmd_len + len);
		memset(newout, 0, ps->cmd_len + len);
		memcpy(newout, ps->cmd, ps->cmd_len);
		if (dout)
			memcpy(newout + ps->cmd_len, dout, len);

		if (din)
			newin = malloc(ps->cmd_len + len);

		ret = pl022_spi_xfer_int(ps, ps->cmd_len + len, newout, newin);

		if (din)
			memcpy(din, newin + ps->cmd_len, len);

		free(newout);
		if (newin)
			free(newin);

		ps->cmd_len = 0;
	}
	else if (!flags || ((flags & SPI_XFER_BEGIN) && (flags & SPI_XFER_END))) // single action
	{
		ret = pl022_spi_xfer_int(ps, len, dout, din);
	}
	else
	{
		hang(); // for debug
	}
#else // CONFIG_ARCH_RCM_ARM
	ret = pl022_spi_xfer_int(ps, len, dout, din);
#endif


#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_TARGET_1888BM18) || defined(CONFIG_ARCH_RCM_ARM)
	if (flags & SPI_XFER_END)
		spi_cs_deactivate(dev, slave_plat->cs);
#endif
	return ret;
}

static inline u32 spi_rate(u32 rate, u16 cpsdvsr, u16 scr)
{
	return rate / (cpsdvsr * (1 + scr));
}

static int pl022_spi_set_speed(struct udevice *bus, uint speed)
{
	struct pl022_spi_slave *ps = dev_get_priv(bus);
	u16 scr = SSP_SCR_MIN, cr0 = 0, cpsr = SSP_CPSR_MIN, best_scr = scr,
	    best_cpsr = cpsr;
	u32 min, max, best_freq = 0, tmp;
	u32 rate = ps->freq;
	bool found = false;

	max = spi_rate(rate, SSP_CPSR_MIN, SSP_SCR_MIN);
	min = spi_rate(rate, SSP_CPSR_MAX, SSP_SCR_MAX);

	if (speed > max || speed < min) {
		pr_err("Tried to set speed to %dHz but min=%d and max=%d\n",
		       speed, min, max);
		return -EINVAL;
	}

	while (cpsr <= SSP_CPSR_MAX && !found) {
		while (scr <= SSP_SCR_MAX) {
			tmp = spi_rate(rate, cpsr, scr);

			if (abs(speed - tmp) < abs(speed - best_freq)) {
				best_freq = tmp;
				best_cpsr = cpsr;
				best_scr = scr;

				if (tmp == speed) {
					found = true;
					break;
				}
			}

			scr++;
		}
		cpsr += 2;
		scr = SSP_SCR_MIN;
	}

	writew(best_cpsr, ps->base + SSP_CPSR);
	cr0 = readw(ps->base + SSP_CR0);
	writew(cr0 | (best_scr << SSP_SCR_SHFT), ps->base + SSP_CR0);

	return 0;
}

static int pl022_spi_set_mode(struct udevice *bus, uint mode)
{
	struct pl022_spi_slave *ps = dev_get_priv(bus);
	u16 reg;

	reg = readw(ps->base + SSP_CR0);
	reg &= ~(SSP_CR0_SPH | SSP_CR0_SPO);
	if (mode & SPI_CPHA)
		reg |= SSP_CR0_SPH;
	if (mode & SPI_CPOL)
		reg |= SSP_CR0_SPO;
	writew(reg, ps->base + SSP_CR0);

	return 0;
}

static int pl022_cs_info(struct udevice *bus, uint cs,
			 struct spi_cs_info *info)
{
	return 0;
}

static const struct dm_spi_ops pl022_spi_ops = {
	.claim_bus      = pl022_spi_claim_bus,
	.release_bus    = pl022_spi_release_bus,
	.xfer           = pl022_spi_xfer,
	.set_speed      = pl022_spi_set_speed,
	.set_mode       = pl022_spi_set_mode,
	.cs_info        = pl022_cs_info,
};

#if !CONFIG_IS_ENABLED(OF_PLATDATA)
static int pl022_spi_ofdata_to_platdata(struct udevice *bus)
{
	struct pl022_spi_pdata *plat = bus->platdata;
	const void *fdt = gd->fdt_blob;
	int node = dev_of_offset(bus);
	struct clk clkdev;
	int ret;

	plat->addr = fdtdec_get_addr_size(fdt, node, "reg", &plat->size);

	ret = clk_get_by_index(bus, 0, &clkdev);
	if (ret)
		return ret;

	plat->freq = clk_get_rate(&clkdev);

	return 0;
}

static const struct udevice_id pl022_spi_ids[] = {
	{ .compatible = "arm,pl022-spi" },
#if defined(CONFIG_TARGET_1888TX018) || defined(CONFIG_ARCH_RCM_ARM)
	{ .compatible = "rcm,pl022-spi" },
#endif
	{ }
};
#endif

U_BOOT_DRIVER(pl022_spi) = {
	.name   = "pl022_spi",
	.id     = UCLASS_SPI,
#if !CONFIG_IS_ENABLED(OF_PLATDATA)
	.of_match = pl022_spi_ids,
	.ofdata_to_platdata = pl022_spi_ofdata_to_platdata,
#endif
	.ops    = &pl022_spi_ops,
	.platdata_auto_alloc_size = sizeof(struct pl022_spi_pdata),
	.priv_auto_alloc_size = sizeof(struct pl022_spi_slave),
	.probe  = pl022_spi_probe,
};
