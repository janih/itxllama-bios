/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2013 DMP Electronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <console/console.h>
#include <device/device.h>
#include <device/pci.h>
#include <device/pci_ops.h>
#include <device/pci_ids.h>
#include <pc80/mc146818rtc.h>
#include <pc80/keyboard.h>
#include <string.h>
#include "arch/io.h"
#include "chip.h"
#include "southbridge.h"
#include <device/pci_def.h>

/* IRQ number to S/B PCI Interrupt routing table reg(0x58/0xb4) mapping table. */
static const unsigned char irq_to_int_routing[16] = {
	0x0, 0x0, 0x0, 0x2,	// IRQ0-2 is unmappable, IRQ3 = 2.
	0x4, 0x5, 0x7, 0x6,	// IRQ4-7 = 4, 5, 7, 6.
	0x0, 0x1, 0x3, 0x9,	// IRQ8 is unmappable, IRQ9-11 = 1, 3, 9.
	0xb, 0x0, 0xd, 0xf	// IRQ12 = b, IRQ13 is unmappable, IRQ14-15 = d, f.
};


#ifndef PCI_CAP_ID_PM
#define PCI_CAP_ID_PM   0x01
#endif
#ifndef PCI_CAP_ID_AGP
#define PCI_CAP_ID_AGP  0x02
#endif


#define DISABLE_S3_AGP_FEATURES 1
#define FIXUP_PCI_IRQ_LINES 1
#define DISABLE_VGA_PALETTE_SNOOPING 1

/* S/B PCI Interrupt routing table reg(0x58) field bit shift. */
#define EHCIH_IRQ_SHIFT 28
#define OHCII_IRQ_SHIFT 24
#define MAC_IRQ_SHIFT 16
#define RT3_IRQ_SHIFT 12
#define RT2_IRQ_SHIFT 8
#define RT1_IRQ_SHIFT 4
#define RT0_IRQ_SHIFT 0

/* S/B Extend PCI Interrupt routing table reg(0xb4) field bit shift. */
#define CAN_IRQ_SHIFT 28
#define HDA_IRQ_SHIFT 20
#define USBD_IRQ_SHIFT 16
#define SIDE_IRQ_SHIFT 12
#define PIDE_IRQ_SHIFT 8

/* S/B function 1 Extend PCI Interrupt routing table reg 2(0xb4)
 * field bit shift.
 */
#define SPI1_IRQ_SHIFT 8
#define MOTOR_IRQ_SHIFT 0

/* in-chip PCI device IRQs(0 for disabled). */
#define EHCII_IRQ 10
#define OHCII_IRQ 14
#define MAC_IRQ 9

#define CAN_IRQ 5
#define HDA_IRQ 11
#define USBD_IRQ 11
#define PIDE_IRQ 11

#define SPI1_IRQ 3
#define I2C0_IRQ 4
#define MOTOR_IRQ 6
#define PCIET_IRQ 4

/* RT0-3 IRQs. */
#define RT3_IRQ 5
#define RT2_IRQ 5
#define RT1_IRQ 5
/*VGA cards sits on root bridge 00:01.0 -> RT0 (INTA). Map to a real (and prefferably free) 8259 IRQ. */
#define RT0_IRQ 5

/* IDE legacy mode IRQs. */
#define IDE1_LEGACY_IRQ 14
#define IDE2_LEGACY_IRQ 15

/* Internal parallel port */
#define LPT_INT_C 0
#define LPT_INT_ACK_SET 0
#define LPT_UE 1
#define LPT_PDMAS 0
#define LPT_DREQS 0

/* internal I2C */
#define I2C_BASE 0xfb00

/* Post codes */
#define POST_KBD_FW_UPLOAD 0x06
#define POST_KBD_CHK_READY 0x07
#define POST_KBD_IS_READY 0x08
#define POST_KBD_FW_VERIFY_FAILURE 0x82

static u8 get_pci_dev_func(device_t dev)
{
	return PCI_FUNC(dev->path.pci.devfn);
}

static void verify_dmp_keyboard_error(void)
{
	post_code(POST_KBD_FW_VERIFY_FAILURE);
	die("Internal keyboard firmware verify error!\n");
}


/* Return offset of a capability, or 0 if not present */
static u8 find_capability(struct device *d, u8 cap_id)
{
    u16 status;
    u8 pos;
    u8 id;
    u8 nxt;
    int i;

    status = pci_read_config16(d, PCI_STATUS);
    if (!(status & 0x0010))
        return 0;

    pos = pci_read_config8(d, PCI_CAPABILITY_LIST); /* 0x34 */
    for (i = 0; i < 48 && pos >= 0x40; i++)
    {
        id  = pci_read_config8(d, pos + 0x0);
        nxt = pci_read_config8(d, pos + 0x1);
        if (id == cap_id)
            return pos;
        if (nxt == 0 || nxt == pos)
            break;
        pos = nxt;
    }
    return 0;
}

/* Zero the AGP Command dword to discourage the driver from enabling AGP */
static void disable_agp_command(struct device *d)
{
    u8 agp;
    agp = find_capability(d, PCI_CAP_ID_AGP);
    if (!agp)
        return;

    /* cap + 0x04 = AGP Status (RO), cap + 0x08 = AGP Command (RW) */
    (void)pci_read_config32(d, agp + 0x04);
    pci_write_config32(d, agp + 0x08, 0x00000000);
}


/* Find S3 VGA on the secondary bus and try to disable AGP features */
static void kill_agp_on_bus2_if_s3(void)
{
 if (DISABLE_S3_AGP_FEATURES)
 {
    const u8 bus = 0x02;
    int devn;
    struct device *d;
    u32 classrev;
    u8 classcode;
    u16 vendor;

    for (devn = 0; devn < 32; devn++)
    {
        d = dev_find_slot(bus, PCI_DEVFN(devn, 0));
        if (!d)
            continue;

        classrev = pci_read_config32(d, PCI_CLASS_REVISION);
        classcode = (u8)(classrev >> 24);
        vendor = pci_read_config16(d, PCI_VENDOR_ID);

        /* Only display class from S3 vendor */
        if (classcode == 0x03 && vendor == 0x5333)
            disable_agp_command(d);
    }
  }
}


static void fixup_secondary_bus_irq_lines(u8 sec_bus)
{
 if (FIXUP_PCI_IRQ_LINES)
 {
    static const u8 rt_irqs[4] = { RT0_IRQ, RT1_IRQ, RT2_IRQ, RT3_IRQ };
    int devn;
    int fn;
    struct device *d;
    u8 pin;
    u8 swz;
    u8 irq;

    for (devn = 0; devn < 32; devn++)
     {
        for (fn = 0; fn < 8; fn++)
          {
            d = dev_find_slot(sec_bus, PCI_DEVFN(devn, fn));
            if (!d)
                continue;

            pin = pci_read_config8(d, PCI_INTERRUPT_PIN); /* 1..4, or 0 if none */
            if (!pin)
                continue;

            /* (pin-1 + device) % 4 => RT0..RT3 */
            swz = (u8)(((pin - 1) + devn) & 3);
            irq = rt_irqs[swz];

            pci_write_config8(d, PCI_INTERRUPT_LINE, irq);
        }
    }
  }
}

static struct device *find_display_on_bus(u8 bus)
{
    int devn;
    int fn;
    struct device *d;
    u32 classrev;
    u16 classsub;

    for (devn = 0; devn < 32; devn++)
    {
        for (fn = 0; fn < 8; fn++)
        {
            d = dev_find_slot(bus, PCI_DEVFN(devn, fn));
            if (!d)
                continue;

            classrev = pci_read_config32(d, PCI_CLASS_REVISION);
            classsub = (u16)((classrev >> 16) & 0xFFFF); /* class<<8 | subclass */

            /* 0x03 = Display controller; 0x0300 = VGA compatible */
            if ((classsub & 0xFF00) == 0x0300)
                return d;
        }
    }
    return NULL;
}

static void stamp_vga_irq_hint(void)
{
    /* Endpoint VGA */
    struct device *vga = find_display_on_bus(0x02);
    if (vga) pci_write_config8(vga, PCI_INTERRUPT_LINE, RT0_IRQ);

    /* Upstream bridges */
    struct device *peri = dev_find_slot(0x01, PCI_DEVFN(0x00, 0));
    if (peri) pci_write_config8(peri, PCI_INTERRUPT_LINE, RT0_IRQ);

    struct device *root = dev_find_slot(0x00, PCI_DEVFN(0x01, 0));
    if (root) pci_write_config8(root, PCI_INTERRUPT_LINE, RT0_IRQ);
}

/*
   ELCR: 0x4D0 -> IRQ0..7, 0x4D1 -> IRQ8..15; bit=1 => level.
   IRQ0  System timer                EDGE
   IRQ1  Keyboard                    EDGE
   IRQ2  PIC                         EDGE
   IRQ3  COM2                        EDGE
   IRQ4  COM1                        EDGE
   IRQ5  PCI VGA/STERING/BRIDGES     LEVEL
   IRQ6  (X)                         EDGE
   IRQ7  ISA SND                     EDGE
   IRQ8  RTC                         EDGE
   IRQ9  PCI ETH/STEERING            LEVEL
   IRQ10 PCI USB/STEERING            LEVEL
   IRQ11 PCI IDE/STEERING            LEVEL
   IRQ12 PS/2 MOUSE                  EDGE
   IRQ13 FPU                         EDGE
   IRQ14 PCI USB/STEERING            LEVEL
   IRQ15 (X)                         EDGE
 */
static void program_elcr_for_pci_isa_irqs(void)
{
    uint8_t elcr0 = inb(0x4D0);
    uint8_t elcr1 = inb(0x4D1);

    /* Set all IRQs to edge trigger */
    elcr0 &= 0x00;
    elcr1 &= 0x00;

    /* ELCR0: IRQ5 level, others edge */
    elcr0 |= (1u<<5);          // IRQ5 = level  (PCI VGA/STEERING/BRIDGES)


    /* ELCR1: IRQ9/10/11/14 level, others edge */
    elcr1 |= (1u<<1)           // IRQ9  level (PCI ETH/STEERING)
          |  (1u<<2)           // IRQ10 level (PCI USB/STEERING)
          |  (1u<<3)           // IRQ11 level (PCI IDE/STEERING)
          |  (1u<<6);          // IRQ14 level (PCI USB/STEERING)

    outb(elcr0, 0x4D0);
    outb(elcr1, 0x4D1);
}

/* Enable legacy VGA forwarding on both bridges: VGA Enable + VGA 16-bit decode */
static void enable_legacy_vga_forwarding(void)
{
    /* Bridge control bits */
    const uint16_t VGA_ENABLE_BIT       = (1u << 3);
    const uint16_t VGA_16BIT_DECODE_BIT = (1u << 4);
    const uint16_t VGA_MASK = VGA_ENABLE_BIT | VGA_16BIT_DECODE_BIT;

    /* Root bridge 00:01.0 */
    struct device *br_root = dev_find_slot(0x00, PCI_DEVFN(0x01, 0));
    if (br_root)
    {
        uint16_t bc = pci_read_config16(br_root, PCI_BRIDGE_CONTROL);
        bc |= VGA_MASK;
        pci_write_config16(br_root, PCI_BRIDGE_CONTROL, bc);
    }

    /* Pericom bridge 01:00.0 */
    struct device *br_peri = dev_find_slot(0x01, PCI_DEVFN(0x00, 0));
    if (br_peri)
    {
        uint16_t bc = pci_read_config16(br_peri, PCI_BRIDGE_CONTROL);
        bc |= VGA_MASK;
        pci_write_config16(br_peri, PCI_BRIDGE_CONTROL, bc);
    }
}


/* Ensure VGA endpoint and both bridges are fully enabled */
static void tune_vga_endpoint_and_bridges(void)
{
    struct device *vga  = find_display_on_bus(0x02);
    struct device *br0  = dev_find_slot(0x00, PCI_DEVFN(0x01, 0));
    struct device *br1  = dev_find_slot(0x01, PCI_DEVFN(0x00, 0));

    if (vga)
    {
        u16 cmd = pci_read_config16(vga, PCI_COMMAND);
        /* IO+MEM+BusMaster */
        cmd |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
        /* Clear Interrupt Disable */
        cmd &= ~0x0400;
        pci_write_config16(vga, PCI_COMMAND, cmd);

        /* Latency timer for endpoint */
        pci_write_config8(vga, PCI_LATENCY_TIMER, 0x40);

        /* Soften error signaling */
        cmd = pci_read_config16(vga, PCI_COMMAND);
        /* bit6=Parity Error Response, bit8=SERR# Enable */
        cmd &= ~((1u << 6) | (1u << 8));
        pci_write_config16(vga, PCI_COMMAND, cmd);

        /* Arbitration hints Min_Gnt, Max_Lat */
        pci_write_config8(vga, 0x3E, 0x08);
        pci_write_config8(vga, 0x3F, 0xF8);


       if(DISABLE_VGA_PALETTE_SNOOPING)
        {
         cmd = pci_read_config16(vga, PCI_COMMAND);
         cmd &= ~(1u << 5);
         pci_write_config16(vga, PCI_COMMAND, cmd);
        }
       else
        {
         cmd = pci_read_config16(vga, PCI_COMMAND);
         cmd |= (1u << 5);
         pci_write_config16(vga, PCI_COMMAND, cmd);
        }
    }

    if (br0) pci_write_config8(br0, PCI_LATENCY_TIMER, 0x40);
    if (br1) pci_write_config8(br1, PCI_LATENCY_TIMER, 0x40);
}


static void enable_isa_forwarding(void)
{
    const uint16_t ISA_ENABLE_BIT = (1u << 2);
    struct device *br0 = dev_find_slot(0x00, PCI_DEVFN(0x01, 0));
    struct device *br1 = dev_find_slot(0x01, PCI_DEVFN(0x00, 0));

    if (br0)
     {
      uint16_t bc = pci_read_config16(br0, PCI_BRIDGE_CONTROL);
      bc |= ISA_ENABLE_BIT;
      pci_write_config16(br0, PCI_BRIDGE_CONTROL, bc);
     }

    if (br1)
     {
      uint16_t bc = pci_read_config16(br1, PCI_BRIDGE_CONTROL);
      bc |= ISA_ENABLE_BIT;
      pci_write_config16(br1, PCI_BRIDGE_CONTROL, bc);
     }
}

static void upload_dmp_keyboard_firmware(struct device *dev)
{
	u32 reg_sb_c0;
	u32 fwptr;

	// enable firmware uploading function by set bit 10.
	post_code(POST_KBD_FW_UPLOAD);
	reg_sb_c0 = pci_read_config32(dev, SB_REG_IPFCR);
	pci_write_config32(dev, SB_REG_IPFCR, reg_sb_c0 | 0x400);

	outw(0, 0x62);		// reset upload address to 0.
	// upload 4096 bytes from 0xFFFFE000.
	outsb(0x66, (u8 *) 0xffffe000, 4096);
	// upload 4096 bytes from 0xFFFFC000.
	outsb(0x66, (u8 *) 0xffffc000, 4096);

	outw(0, 0x62);		// reset upload address to 0.
	// verify 4096 bytes from 0xFFFFE000.
	for (fwptr = 0xffffe000; fwptr < 0xfffff000; fwptr++) {
		if (inb(0x66) != *(u8 *) fwptr) {
			verify_dmp_keyboard_error();
		}
	}
	// verify 4096 bytes from 0xFFFFC000.
	for (fwptr = 0xffffc000; fwptr < 0xffffd000; fwptr++) {
		if (inb(0x66) != *(u8 *) fwptr) {
			verify_dmp_keyboard_error();
		}
	}

	// disable firmware uploading.
	pci_write_config32(dev, SB_REG_IPFCR, reg_sb_c0 & ~0x400L);
}

static void kbc_wait_system_flag(void)
{
	/* wait keyboard controller ready by checking system flag
	 * (status port bit 2).
	 */
	post_code(POST_KBD_CHK_READY);
	while ((inb(0x64) & 0x4) == 0) {
	}
	post_code(POST_KBD_IS_READY);
}

static void pci_routing_fixup(struct device *dev)
{
	const unsigned slot[3] = { 0 };
	const unsigned char slot_irqs[1][4] = {
		{RT0_IRQ, RT1_IRQ, RT2_IRQ, RT3_IRQ},
	};
	const int slot_num = 1;
	int i;
	u32 int_routing = 0;
	u32 ext_int_routing = 0;

	/* assign PCI-e bridge (bus#0, dev#1, fn#0) IRQ to RT0. */
	pci_assign_irqs(0, 1, slot_irqs[0]);

	int_routing |= irq_to_int_routing[RT0_IRQ] << RT0_IRQ_SHIFT;
	int_routing |= irq_to_int_routing[RT1_IRQ] << RT1_IRQ_SHIFT;
	int_routing |= irq_to_int_routing[RT2_IRQ] << RT2_IRQ_SHIFT;
	int_routing |= irq_to_int_routing[RT3_IRQ] << RT3_IRQ_SHIFT;

	/* assign PCI slot IRQs. */
	for (i = 0; i < slot_num; i++) {
		pci_assign_irqs(1, slot[i], slot_irqs[i]);
	}

	/* Read PCI slot IRQs to see if RT1-3 is used, and enables it */

	for (i = 0; i < slot_num; i++) {
	/* Skip this loop entirely */
	continue;
		unsigned int funct;
		device_t pdev;
		u8 irq;

		/* Each slot may contain up to eight functions. */
		for (funct = 0; funct < 8; funct++) {
			pdev = dev_find_slot(1, (slot[i] << 3) + funct);
			if (!pdev)
				continue;
			irq = pci_read_config8(pdev, PCI_INTERRUPT_LINE);
			if (irq == RT1_IRQ) {
				int_routing |= irq_to_int_routing[RT1_IRQ] << RT1_IRQ_SHIFT;
			} else if (irq == RT2_IRQ) {
				int_routing |= irq_to_int_routing[RT2_IRQ] << RT2_IRQ_SHIFT;
			} else if (irq == RT3_IRQ) {
				int_routing |= irq_to_int_routing[RT3_IRQ] << RT3_IRQ_SHIFT;
			}
		}
	}

	/* Setup S/B PCI Interrupt routing table reg(0x58). */
	int_routing |= irq_to_int_routing[EHCII_IRQ] << EHCIH_IRQ_SHIFT;
	int_routing |= irq_to_int_routing[OHCII_IRQ] << OHCII_IRQ_SHIFT;
	int_routing |= irq_to_int_routing[MAC_IRQ] << MAC_IRQ_SHIFT;
	pci_write_config32(dev, SB_REG_PIRQ_ROUTE, int_routing);


	/* Setup S/B PCI Extend Interrupt routing table reg(0xb4). */
	// ext_int_routing |= irq_to_int_routing[CAN_IRQ] << CAN_IRQ_SHIFT;
	// ext_int_routing |= irq_to_int_routing[HDA_IRQ] << HDA_IRQ_SHIFT;
	ext_int_routing |= irq_to_int_routing[USBD_IRQ] << USBD_IRQ_SHIFT;
#if CONFIG_IDE_NATIVE_MODE
	/* IDE in native mode, only uses one IRQ. */
	ext_int_routing |= irq_to_int_routing[0] << SIDE_IRQ_SHIFT;
	ext_int_routing |= irq_to_int_routing[PIDE_IRQ] << PIDE_IRQ_SHIFT;
#else
	/* IDE in legacy mode, use IRQ 14, 15. */
	ext_int_routing |= irq_to_int_routing[IDE2_LEGACY_IRQ] << SIDE_IRQ_SHIFT;
	ext_int_routing |= irq_to_int_routing[IDE1_LEGACY_IRQ] << PIDE_IRQ_SHIFT;
#endif
	pci_write_config32(dev, SB_REG_EXT_PIRQ_ROUTE, ext_int_routing);

	fixup_secondary_bus_irq_lines(0x02);

	/* Assign in-chip PCI device IRQs. */
	if (MAC_IRQ) {
		unsigned char irqs[4] = { MAC_IRQ, 0, 0, 0 };
		pci_assign_irqs(0, 0x8, irqs);
	}
	if (OHCII_IRQ && EHCII_IRQ) {
		unsigned char irqs[4] = { OHCII_IRQ, EHCII_IRQ, 0, 0 };
		pci_assign_irqs(0, 0xa, irqs);
	}
	if (CONFIG_IDE_NATIVE_MODE && PIDE_IRQ) {
		/* IDE in native mode, setup PCI IRQ. */
		unsigned char irqs[4] = { PIDE_IRQ, 0, 0, 0 };
		pci_assign_irqs(0, 0xc, irqs);
	}
	// if (CAN_IRQ) {
	// 	unsigned char irqs[4] = { CAN_IRQ, 0, 0, 0 };
	// 	pci_assign_irqs(0, 0x11, irqs);
	// }
	// if (HDA_IRQ) {
	// 	unsigned char irqs[4] = { HDA_IRQ, 0, 0, 0 };
	// 	pci_assign_irqs(0, 0xe, irqs);
	// }
	if (USBD_IRQ) {
		unsigned char irqs[4] = { USBD_IRQ, 0, 0, 0 };
		pci_assign_irqs(0, 0xf, irqs);
	}

    program_elcr_for_pci_isa_irqs();
    enable_legacy_vga_forwarding();
    enable_isa_forwarding();
    tune_vga_endpoint_and_bridges();
    stamp_vga_irq_hint();
    kill_agp_on_bus2_if_s3();
}

static void vortex_sb_init(struct device *dev)
{
	u32 lpt_reg = 0;

#if CONFIG_LPT_ENABLE
	int ppmod = 0;
#if CONFIG_LPT_MODE_BPP
	ppmod = 0;
#elif CONFIG_LPT_MODE_EPP_19_AND_SPP
	ppmod = 1;
#elif CONFIG_LPT_MODE_ECP
	ppmod = 2;
#elif CONFIG_LPT_MODE_ECP_AND_EPP_19
	ppmod = 3;
#elif CONFIG_LPT_MODE_SPP
	ppmod = 4;
#elif CONFIG_LPT_MODE_EPP_17_AND_SPP
	ppmod = 5;
#elif CONFIG_LPT_MODE_ECP_AND_EPP_17
	ppmod = 7;
#else
#error CONFIG_LPT_MODE error.
#endif

	/* Setup internal parallel port */
	lpt_reg |= (LPT_INT_C << 28);
	lpt_reg |= (LPT_INT_ACK_SET << 27);
	lpt_reg |= (ppmod << 24);
	lpt_reg |= (LPT_UE << 23);
	lpt_reg |= (LPT_PDMAS << 22);
	lpt_reg |= (LPT_DREQS << 20);
	lpt_reg |= (irq_to_int_routing[CONFIG_LPT_IRQ] << 16);
	lpt_reg |= (CONFIG_LPT_IO << 0);
#endif				// CONFIG_LPT_ENABLE
	pci_write_config32(dev, SB_REG_IPPCR, lpt_reg);
}

#define SETUP_GPIO_ADDR(n) \
	u32 cfg##n = (CONFIG_GPIO_P##n##_DIR_ADDR << 16) | (CONFIG_GPIO_P##n##_DATA_ADDR);\
	outl(cfg##n, base + 4 + (n * 4));\
	gpio_enable_mask |= (1 << n);

#define INIT_GPIO(n) \
	outb(CONFIG_GPIO_P##n##_INIT_DIR, CONFIG_GPIO_P##n##_DIR_ADDR);\
	outb(CONFIG_GPIO_P##n##_INIT_DATA, CONFIG_GPIO_P##n##_DATA_ADDR);

static void ex_sb_gpio_init(struct device *dev)
{
	const int base = 0xb00;
	u32 gpio_enable_mask = 0;
	/* S/B register 63h - 62h : GPIO Port Config IO Base Address */
	pci_write_config16(dev, SB_REG_GPIO_CFG_IO_BASE, base | 1);
	/* Set GPIO port 0~9 base address.
	 * Config Base + 04h, 08h, 0ch... : GPIO port 0~9 data/dir decode addr.
	 * Bit 31-16 : DBA, GPIO direction base address.
	 * Bit 15-0  : DPBA, GPIO data port base address.
	 * */
#if CONFIG_GPIO_P0_ENABLE
	SETUP_GPIO_ADDR(0)
#endif
#if CONFIG_GPIO_P1_ENABLE
	SETUP_GPIO_ADDR(1)
#endif
#if CONFIG_GPIO_P2_ENABLE
	SETUP_GPIO_ADDR(2)
#endif
#if CONFIG_GPIO_P3_ENABLE
	SETUP_GPIO_ADDR(3)
#endif
#if CONFIG_GPIO_P4_ENABLE
	SETUP_GPIO_ADDR(4)
#endif
#if CONFIG_GPIO_P5_ENABLE
	SETUP_GPIO_ADDR(5)
#endif
#if CONFIG_GPIO_P6_ENABLE
	SETUP_GPIO_ADDR(6)
#endif
#if CONFIG_GPIO_P7_ENABLE
	SETUP_GPIO_ADDR(7)
#endif
#if CONFIG_GPIO_P8_ENABLE
	SETUP_GPIO_ADDR(8)
#endif
#if CONFIG_GPIO_P9_ENABLE
	SETUP_GPIO_ADDR(9)
#endif
	/* Enable GPIO port 0~9. */
	outl(gpio_enable_mask, base);
	/* Set GPIO port 0-9 initial dir and data. */
#if CONFIG_GPIO_P0_ENABLE
	INIT_GPIO(0)
#endif
#if CONFIG_GPIO_P1_ENABLE
	INIT_GPIO(1)
#endif
#if CONFIG_GPIO_P2_ENABLE
	INIT_GPIO(2)
#endif
#if CONFIG_GPIO_P3_ENABLE
	INIT_GPIO(3)
#endif
#if CONFIG_GPIO_P4_ENABLE
	INIT_GPIO(4)
#endif
#if CONFIG_GPIO_P5_ENABLE
	INIT_GPIO(5)
#endif
#if CONFIG_GPIO_P6_ENABLE
	INIT_GPIO(6)
#endif
#if CONFIG_GPIO_P7_ENABLE
	INIT_GPIO(7)
#endif
#if CONFIG_GPIO_P8_ENABLE
	INIT_GPIO(8)
#endif
#if CONFIG_GPIO_P9_ENABLE
	INIT_GPIO(9)
#endif
	/* Disable GPIO Port Config IO Base Address. */
	pci_write_config16(dev, SB_REG_GPIO_CFG_IO_BASE, 0x0);
}

static u32 make_uart_config(u16 base, u8 irq)
{
	u8 mapped_irq = irq_to_int_routing[irq];
	u32 cfg = 0;
	cfg |= 1 << 23;			// UE = enabled.
	cfg |= (mapped_irq << 16);	// UIRT.
	cfg |= base;			// UIOA.
	return cfg;
}

#define SETUP_UART(n) \
	uart_cfg = make_uart_config(CONFIG_UART##n##_IO, CONFIG_UART##n##_IRQ);\
	outl(uart_cfg, base + (n - 1) * 4);\
	uart8250_init(CONFIG_UART##n##_IO, 115200 / CONFIG_UART##n##_BAUD);

static void ex_sb_uart_init(struct device *dev)
{
	const int base = 0xc00;
	u32 uart_cfg = 0;
	/* S/B register 61h - 60h : UART Config IO Base Address */
	pci_write_config16(dev, SB_REG_UART_CFG_IO_BASE, base | 1);
	/* setup UART */
#if CONFIG_UART1_ENABLE
	SETUP_UART(1)
#endif
#if CONFIG_UART2_ENABLE
	SETUP_UART(2)
#endif
#if CONFIG_UART3_ENABLE
	SETUP_UART(3)
#endif
#if CONFIG_UART4_ENABLE
	SETUP_UART(4)
#endif
#if CONFIG_UART5_ENABLE
	SETUP_UART(5)
#endif
#if CONFIG_UART6_ENABLE
	SETUP_UART(6)
#endif
#if CONFIG_UART7_ENABLE
	SETUP_UART(7)
#endif
#if CONFIG_UART8_ENABLE
	SETUP_UART(8)
#endif
#if CONFIG_UART9_ENABLE
	SETUP_UART(9)
#endif
#if CONFIG_UART10_ENABLE
	SETUP_UART(10)
#endif
	/* Keep UART Config I/O base address */
	//pci_write_config16(SB, SB_REG_UART_CFG_IO_BASE, 0x0);
}

static void i2c_init(struct device *dev)
{
	u8 mapped_irq = irq_to_int_routing[I2C0_IRQ];
	u32 cfg = 0;
	cfg |= 1 << 31;			// UE = enabled.
	cfg |= (mapped_irq << 16);	// IIRT0.
	cfg |= I2C_BASE;		// UIOA.
	pci_write_config32(dev, SB_REG_II2CCR, cfg);
}

static int get_rtc_update_in_progress(void)
{
	if (cmos_read(RTC_REG_A) & RTC_UIP)
		return 1;
	return 0;
}

static void unsafe_read_cmos_rtc(u8 rtc[7])
{
	rtc[0] = cmos_read(RTC_CLK_ALTCENTURY);
	rtc[1] = cmos_read(RTC_CLK_YEAR);
	rtc[2] = cmos_read(RTC_CLK_MONTH);
	rtc[3] = cmos_read(RTC_CLK_DAYOFMONTH);
	rtc[4] = cmos_read(RTC_CLK_HOUR);
	rtc[5] = cmos_read(RTC_CLK_MINUTE);
	rtc[6] = cmos_read(RTC_CLK_SECOND);
}

static void read_cmos_rtc(u8 rtc[7])
{
	/* Read RTC twice and check update-in-progress flag, to make
	 * sure RTC is correct */
	u8 rtc_old[7], rtc_new[7];
	while (get_rtc_update_in_progress()) ;
	unsafe_read_cmos_rtc(rtc_new);
	do {
		memcpy(rtc_old, rtc_new, 7);
		while (get_rtc_update_in_progress()) ;
		unsafe_read_cmos_rtc(rtc_new);
	} while (memcmp(rtc_new, rtc_old, 7) != 0);
}

/*
 * Convert a number in decimal format into the BCD format.
 * Return 255 if not a valid BCD value.
 */
static u8 bcd2dec(u8 bcd)
{
	u8 h, l;
	h = bcd >> 4;
	l = bcd & 0xf;
	if (h > 9 || l > 9)
		return 255;
	return h * 10 + l;
}

static void fix_cmos_rtc_time(void)
{
	/* Read RTC data. */
	u8 rtc[7];
	read_cmos_rtc(rtc);

	/* Convert RTC from BCD format to binary. */
	u8 bin_rtc[7];
	int i;
	for (i = 0; i < 8; i++) {
		bin_rtc[i] = bcd2dec(rtc[i]);
	}

	/* If RTC date is invalid, fix it. */
	if (bin_rtc[0] > 99 || bin_rtc[1] > 99 || bin_rtc[2] > 12 || bin_rtc[3] > 31) {
		/* Set PC compatible timing mode. */
		cmos_write(0x26, RTC_REG_A);
		cmos_write(0x02, RTC_REG_B);
		/* Now setup a default date 2008/08/08 08:08:08. */
		cmos_write(0x8, RTC_CLK_SECOND);
		cmos_write(0x8, RTC_CLK_MINUTE);
		cmos_write(0x8, RTC_CLK_HOUR);
		cmos_write(0x6, RTC_CLK_DAYOFWEEK);	/* Friday */
		cmos_write(0x8, RTC_CLK_DAYOFMONTH);
		cmos_write(0x8, RTC_CLK_MONTH);
		cmos_write(0x8, RTC_CLK_YEAR);
		cmos_write(0x20, RTC_CLK_ALTCENTURY);
	}
}

static void vortex86_sb_set_io_resv(device_t dev, u32 io_resv_size)
{
	struct resource *res;
	res = new_resource(dev, 1);
	res->base = 0x0UL;
	res->size = io_resv_size;
	res->limit = 0xffffUL;
	res->flags = IORESOURCE_IO | IORESOURCE_ASSIGNED | IORESOURCE_FIXED;
}

static void vortex86_sb_set_spi_flash_size(device_t dev, u32 flash_size)
{
	/* SPI flash is in topmost of 4G memory space */
	struct resource *res;
	res = new_resource(dev, 2);
	res->base = 0x100000000LL - flash_size;
	res->size = flash_size;
	res->limit = 0xffffffffUL;
	res->flags = IORESOURCE_MEM | IORESOURCE_FIXED | IORESOURCE_STORED | IORESOURCE_ASSIGNED;
}

static void vortex86_sb_read_resources(device_t dev)
{
	u32 flash_size = 8 * 1024 * 1024;

	pci_dev_read_resources(dev);

	if (dev->device == 0x6011) {
		/* It is EX CPU southbridge */
		if (get_pci_dev_func(dev) != 0) {
			/* only for function 0, skip function 1 */
			return;
		}
		/* default SPI flash ROM is 64MB */
		flash_size = 64 * 1024 * 1024;
	}

	/* Reserve space for I/O */
	vortex86_sb_set_io_resv(dev, 0x1000UL);

	/* Reserve space for flash */
	vortex86_sb_set_spi_flash_size(dev, flash_size);
}

static void southbridge_init_func1(struct device *dev)
{
	/* Handle S/B function 1 PCI IRQ routing. (SPI1/MOTOR) */
	u32 ext_int_routing2 = 0;
	/* Setup S/B function 1 PCI Extend Interrupt routing table reg 2(0xb4). */
	ext_int_routing2 |= irq_to_int_routing[SPI1_IRQ] << SPI1_IRQ_SHIFT;
	ext_int_routing2 |= irq_to_int_routing[MOTOR_IRQ] << MOTOR_IRQ_SHIFT;
	pci_write_config32(dev, SB1_REG_EXT_PIRQ_ROUTE2, ext_int_routing2);
	/* Setup S/B function 1 PCI-e Target Config Reg 1(0x64). */
	u32 pciet_cfg1 = irq_to_int_routing[PCIET_IRQ];
	pci_write_config32(dev, SB1_REG_PCIET_CFG1, pciet_cfg1);

	/* Assign in-chip PCI device IRQs. */
	if (SPI1_IRQ || MOTOR_IRQ) {
		unsigned char irqs[4] = { MOTOR_IRQ, SPI1_IRQ, 0, 0 };
		pci_assign_irqs(0, 0x10, irqs);
	}
}

static void southbridge_init(struct device *dev)
{
	/* Check it is function 0 or 1. (Same Vendor/Device ID) */
	if (get_pci_dev_func(dev) != 0) {
		southbridge_init_func1(dev);
		return;
	}
	upload_dmp_keyboard_firmware(dev);
	vortex_sb_init(dev);
	if (dev->device == 0x6011) {
		ex_sb_gpio_init(dev);
		ex_sb_uart_init(dev);
		i2c_init(dev);
	}
	pci_routing_fixup(dev);

	fix_cmos_rtc_time();
	rtc_init(0);
	kbc_wait_system_flag();
	pc_keyboard_init(0);
}

static struct device_operations vortex_sb_ops = {
	.read_resources   = vortex86_sb_read_resources,
	.set_resources    = pci_dev_set_resources,
	.enable_resources = pci_dev_enable_resources,
	.init             = &southbridge_init,
	.scan_bus         = scan_static_bus,
	.enable           = 0,
	.ops_pci          = 0,
};

static const struct pci_driver pci_driver_6011 __pci_driver = {
	.ops    = &vortex_sb_ops,
	.vendor = PCI_VENDOR_ID_RDC,
	.device = 0x6011,	/* EX CPU S/B ID */
};

struct chip_operations southbridge_dmp_vortex86ex_ops = {
	CHIP_NAME("DMP Vortex86EX Southbridge")
	.enable_dev = 0
};
