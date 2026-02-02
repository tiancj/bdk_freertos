/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usb_beken_musb_reg.h"
#include "gpio_pub.h"
#include "sys_ctrl.h"
#include "sys_ctrl_pub.h"
#include "usb_pub.h"
#include "intc_pub.h"
#include "arm_arch.h"
#include "icu_pub.h"
//#include "driver/usb/usb.h"

#define HWREG(x) \
    (*((volatile uint32_t *)(x)))
#define HWREGH(x) \
    (*((volatile uint16_t *)(x)))
#define HWREGB(x) \
    (*((volatile uint8_t *)(x)))

#ifndef USBD_IRQHandler
#define USBD_IRQHandler USB_Handler //use actual usb irq name instead
#endif

#ifndef USB_BASE_ADDR
#define USB_BASE_ADDR           (0x00804000)
#endif
#define REG_USB_BASE_ADDR       USB_BASE_ADDR

#define USB_DPLL_DIVISION                 (2)

#define REG_AHB2_USB_OTG_CFG              (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x80)))
#define REG_AHB2_USB_DMA_ENDP             (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x84)))
#define REG_AHB2_USB_VTH                  (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x88)))
#define REG_AHB2_USB_GEN                  (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x8C)))
#define REG_AHB2_USB_STAT                 (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x90)))
#define REG_AHB2_USB_INT                  (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x94)))
#define REG_AHB2_USB_RESET                (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x98)))
#define REG_AHB2_USB_DEV_CFG              (*((volatile unsigned char *) (REG_USB_BASE_ADDR + 0x9C)))

/* Common USB Registers */
#define MUSB_FADDR_OFFSET        0x00  /* Function Address */
#define MUSB_POWER_OFFSET        0x01  /* POWER */
#define MUSB_TXIS_OFFSET         0x02  /* Indicates Tx interrupts are currently active for Endpoint 0 and Tx Endpoints 1-7*/
#define MUSB_RXIS_OFFSET         0x04  /* Indicates Rx interrupts are currently active Endpoints 1-7 */
#define MUSB_IS_OFFSET           0x06  /* Indicates which USB interrupts are currently active */
#define MUSB_TXIEL_OFFSET        0x07  /* TX interrupt enable for ep0-7 */
#define MUSB_TXIEH_OFFSET        0x08
#define MUSB_RXIEL_OFFSET        0x09  /* RX interrupt enable for ep1-7 */
#define MUSB_RXIEH_OFFSET        0x0A
#define MUSB_IE_OFFSET           0x0B  /* Interrupt enable for USB */
#define MUSB_EPIDX_OFFSET        0x0E  /* Endpoint Index */
#define MUSB_DEVCTL_OFFSET       0x0F  /* DEV control: Host/Peripheral, VBUS, etc. */

/* Endpoint Control/Status registers */
#define MUSB_IND_TXMAP_OFFSET    0x10  /* TxMaxP: maximum packet size in unit of 8 bytes */
#define MUSB_IND_TXCSRL_OFFSET   0x11  /* Tx control and status lower bits */
#define MUSB_IND_TXCSRH_OFFSET   0x12  /* Tx control and status higher bits */
#define MUSB_IND_RXMAP_OFFSET    0x13  /* RxMaxP: Maximum Packet size in unit of 8 bytes */
#define MUSB_IND_RXCSRL_OFFSET   0x14  /* Rx control and status lower bits */
#define MUSB_IND_RXCSRH_OFFSET   0x15  /* Rx control and status higher bits */
#define MUSB_IND_RXCOUNT_OFFSET  0x16  /* Rx counters: High & Low */
/* 0x18 ~ 0x1B: reserved */
#define MUSB_TX_DYNA_CONG_OFFSET 0x1C  /* Tx Dynamic FIFO Sizing */
#define MUSB_RX_DYNA_CONG_OFFSET 0x1E  /* Rx Dynamic FIFO Sizing */
#define MUSB_TXRXFIFOSZ_OFFSET   0x1F  /* TX or RX FIFO Size */


#define MUSB_TX_FIFO1            0x1C  /* Tx Dynamic FIFO Sizing */
#define MUSB_TX_FIFO2            0x1D  /* Tx Dynamic FIFO Sizing */
#define MUSB_RX_FIFO1            0x1E  /* Rx Dynamic FIFO Sizing */
#define MUSB_RX_FIFO2            0x1F  /* Rx Dynamic FIFO Sizing */


/* FIFOs */
#define MUSB_FIFO_OFFSET         0x20

#define USB_FIFO_BASE(ep_idx) (USB_BASE + MUSB_FIFO_OFFSET + 0x4 * ep_idx)

#ifndef USB_NUM_BIDIR_ENDPOINTS
#define USB_NUM_BIDIR_ENDPOINTS  8
#endif

typedef enum {
    USB_EP0_STATE_SETUP = 0x0,      /**< SETUP DATA */
    USB_EP0_STATE_IN_DATA = 0x1,    /**< IN DATA */
    USB_EP0_STATE_OUT_DATA = 0x3,   /**< OUT DATA */
    USB_EP0_STATE_IN_STATUS = 0x4,  /**< IN status */
    USB_EP0_STATE_OUT_STATUS = 0x5, /**< OUT status */
    USB_EP0_STATE_IN_ZLP = 0x6,     /**< OUT status */
    USB_EP0_STATE_STALL = 0x7,      /**< STALL status */
} ep0_state_t;

/* Endpoint state */
struct musb_ep_state {
    uint16_t ep_mps;    /* Endpoint max packet size */
    uint8_t ep_type;    /* Endpoint type */
    uint8_t ep_stalled; /* Endpoint stall flag */
    uint8_t ep_enable;  /* Endpoint enable */
    uint8_t *xfer_buf;
    uint32_t xfer_len;
    uint32_t actual_xfer_len;
};

/* Driver state */
struct musb_udc {
    volatile uint8_t dev_addr;
    volatile uint32_t fifo_size_offset;
    __attribute__((aligned(32))) struct usb_setup_packet setup;
    struct musb_ep_state in_ep[USB_NUM_BIDIR_ENDPOINTS];  /*!< IN endpoint parameters*/
    struct musb_ep_state out_ep[USB_NUM_BIDIR_ENDPOINTS]; /*!< OUT endpoint parameters */
} g_musb_udc;

void USBD_IRQHandler(void);

static volatile uint8_t usb_ep0_state = USB_EP0_STATE_SETUP;
volatile bool zlp_flag = 0;

/* get current active ep */
static uint8_t musb_get_active_ep(void)
{
    return HWREGB(USB_BASE + MUSB_EPIDX_OFFSET);
}

/* set the active ep */
static void musb_set_active_ep(uint8_t ep_index)
{
    HWREGB(USB_BASE + MUSB_EPIDX_OFFSET) = ep_index;
}

/* write @buffer to @ep_idx FIFO */
static void musb_write_packet(uint8_t ep_idx, uint8_t *buffer, uint16_t len)
{
    uint32_t *buf32;
    uint8_t *buf8;
    uint32_t count32;
    uint32_t count8;
    int i;

    if ((uint32_t)buffer & 0x03) {
        buf8 = buffer;
        for (i = 0; i < len; i++) {
            HWREGB(USB_FIFO_BASE(ep_idx)) = *buf8++;
        }
    } else {
        count32 = len >> 2;
        count8 = len & 0x03;

        buf32 = (uint32_t *)buffer;

        while (count32--) {
            HWREG(USB_FIFO_BASE(ep_idx)) = *buf32++;
        }

        buf8 = (uint8_t *)buf32;

        while (count8--) {
            HWREGB(USB_FIFO_BASE(ep_idx)) = *buf8++;
        }
    }
}

/* Read @ep_idx FIFO to @buffer */
static void musb_read_packet(uint8_t ep_idx, uint8_t *buffer, uint16_t len)
{
    uint32_t *buf32;
    uint8_t *buf8;
    uint32_t count32;
    uint32_t count8;
    int i;

    if ((uint32_t)buffer & 0x03) {
        buf8 = buffer;
        for (i = 0; i < len; i++) {
            *buf8++ = HWREGB(USB_FIFO_BASE(ep_idx));
        }
    } else {
        count32 = len >> 2;
        count8 = len & 0x03;

        buf32 = (uint32_t *)buffer;

        while (count32--) {
            *buf32++ = HWREG(USB_FIFO_BASE(ep_idx));
        }

        buf8 = (uint8_t *)buf32;

        while (count8--) {
            *buf8++ = HWREGB(USB_FIFO_BASE(ep_idx));
        }
    }
}

#ifdef CONFIG_MUSB_DYNFIFO
/* Dynamic FIFO Sizing: @mps max packet size */
static uint32_t musb_get_fifo_size(uint16_t mps, uint16_t *used)
{
    uint32_t size;

    for (uint8_t i = USB_TXFIFOSZ_SIZE_8; i <= USB_TXFIFOSZ_SIZE_2048; i++) {
        size = (8 << i);
        if (mps <= size) {
            *used = size;
            return i;
        }
    }

    *used = 0;
    return USB_TXFIFOSZ_SIZE_8;
}
#endif


#define GUWENFU_SETTING

extern void delay(int num);


// usb_open
__WEAK void usb_dc_low_level_init(void)
{
#if 0//def ARMINO
    uint8_t reg = 0;

    sys_drv_usb_clock_ctrl(true, NULL);
    sys_drv_usb_analog_phy_en(true, NULL);
    sys_drv_usb_analog_speed_en(true, NULL);
    sys_drv_usb_analog_ckmcu_en(true, NULL);
    sys_drv_usb_analog_deepsleep_en(false);
    sys_drv_int_enable(USB_INTERRUPT_CTRL_BIT);

    bk_int_isr_register(INT_SRC_USB, USBD_IRQHandler, NULL);
    bk_int_set_priority(INT_SRC_USB, 2);

    REG_AHB2_USB_OTG_CFG = 0x9;
    REG_AHB2_USB_STAT = 0x07;
    reg = REG_AHB2_USB_INT;
    REG_AHB2_USB_INT = reg;
    /*dp and dn driver current selection */
    REG_AHB2_USB_GEN = (0x7 << 4) | (0x7 << 0);
    REG_AHB2_USB_RESET = 0x01;
#else
    UINT32 op_flag = USB_DEVICE_MODE;
    UINT8 reg;
    UINT32 param;
    UINT32 usb_mode = op_flag;

    USB_LOG_INFO("usb_open\r\n");

#if ((SOC_BK7231U == CFG_SOC_NAME) || (SOC_BK7221U == CFG_SOC_NAME))
    USB_LOG_INFO("gpio_usb_second_function\r\n");
    gpio_usb_second_function();
#endif

    /*step0.0: power up usb subsystem*/
    param = 0;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_USB_POWERUP, &param);

    /*step 1.0: reset usb module*/
    param = 0;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_USB_SUBSYS_RESET, &param);

    /*step1.1: open clock*/
    param = BLK_BIT_DPLL_480M | BLK_BIT_USB;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLK_ENABLE, &param);

    param = MCLK_SELECT_DPLL;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_MCLK_SELECT, &param);

    param = USB_DPLL_DIVISION;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_MCLK_DIVISION, &param);

    /*step2: config clock power down for peripheral unit*/
    param = PWD_USB_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);

//#ifndef GUWENFU_SETTING
//    VREG_USB_TEST_MODE = 0x01;
//#endif

//#ifdef GUWENFU_SETTING
//    VREG_USB_INTRRX1E = 0x0;
//    VREG_USB_INTRTX1E = 0x0;
//    VREG_USB_INTRUSBE = 0x0;
    REG_AHB2_USB_VTH &= ~(1 << 7); /* disable INT_DEV_VBUS_EN */
//#endif

//#ifndef GUWENFU_SETTING
//#ifdef MUSB_FORCE_FULLSPEED
//    VREG_USB_POWER = 0x01;
//#else
//    VREG_USB_POWER |= 0x21;
//#endif
//
//    VREG_USB_FADDR = 0;
//    VREG_USB_DEVCTL = 0x01;
//#endif


//#ifdef GUWENFU_SETTING
    if (usb_mode == USB_HOST_MODE)
    {
        USB_LOG_INFO("usb host\r\n");
        REG_WRITE(SCTRL_ANALOG_CTRL2, REG_READ(SCTRL_ANALOG_CTRL2) & (~(1 << 25)));  // ???
        REG_AHB2_USB_OTG_CFG = 0x50;        // host
        REG_AHB2_USB_DEV_CFG = 0x00;
    }
    else
    {
        USB_LOG_INFO("usb device\r\n");
        REG_WRITE(SCTRL_ANALOG_CTRL2, REG_READ(SCTRL_ANALOG_CTRL2) | (1 << 25));  // ???
        //VREG_USB_INTRRX1E = 0x07;
        //VREG_USB_INTRTX1E = 0x07;
        //VREG_USB_INTRUSBE = 0x3F;

        REG_AHB2_USB_OTG_CFG = 0x08;        // dp pull up
        REG_AHB2_USB_DEV_CFG = 0xF4;        // ????
        REG_AHB2_USB_OTG_CFG |= 0x01;        // device
    }

    reg = REG_AHB2_USB_INT;
    delay(100);
    REG_AHB2_USB_INT = reg;
    delay(100);

    /* dp and dn driver current selection */
    REG_AHB2_USB_GEN = (0x7 << 4) | (0x7 << 0);
//#endif

    /*step2: interrupt setting about usb*/
    intc_service_register(IRQ_USB, PRI_IRQ_USB, USBD_IRQHandler);

    intc_enable(IRQ_USB);

    param = GINTR_IRQ_BIT;
    sddev_control(ICU_DEV_NAME, CMD_ICU_GLOBAL_INT_ENABLE, &param);

//    return USB_SUCCESS;
#endif
}

// usb_close
__WEAK void usb_dc_low_level_deinit(void)
{
    uint32_t param;

    param = IRQ_USB_BIT;
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);

    param = PWD_USB_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, &param);
}

int usb_dc_init(void)
{
    usb_dc_low_level_init();

#ifdef CONFIG_USB_HS
    HWREGB(USB_BASE + MUSB_POWER_OFFSET) |= USB_POWER_HSENAB;
#else
    HWREGB(USB_BASE + MUSB_POWER_OFFSET) &= ~USB_POWER_HSENAB;
#endif

    musb_set_active_ep(0);
    HWREGB(USB_BASE + MUSB_FADDR_OFFSET) = 0;

    HWREGB(USB_BASE + MUSB_DEVCTL_OFFSET) |= USB_DEVCTL_SESSION;

    /* Enable USB interrupts */
    HWREGB(USB_BASE + MUSB_IE_OFFSET) = USB_IE_RESET;
    HWREGB(USB_BASE + MUSB_TXIEL_OFFSET) = USB_TXIE_EP0;
    HWREGB(USB_BASE + MUSB_RXIEL_OFFSET) = 0;

    HWREGB(USB_BASE + MUSB_POWER_OFFSET) |= USB_POWER_SOFTCONN;
    return 0;
}

int usb_dc_deinit(void)
{
    usb_dc_low_level_deinit();
    return 0;
}

int usbd_set_address(const uint8_t addr)
{
    if (addr == 0) {
        HWREGB(USB_BASE + MUSB_FADDR_OFFSET) = 0;
    }

    g_musb_udc.dev_addr = addr;
    return 0;
}

int usbd_ep_open(const struct usbd_endpoint_cfg *ep_cfg)
{
    uint16_t used = 0;
#ifdef CONFIG_MUSB_DYNFIFO
    uint16_t fifo_size = 0;
#endif
    uint8_t ep_idx = USB_EP_GET_IDX(ep_cfg->ep_addr);
    uint8_t old_ep_idx;
    uint32_t ui32Flags = 0;
    uint16_t ui32Register = 0;

    if (ep_idx == 0) {
        g_musb_udc.out_ep[0].ep_mps = USB_CTRL_EP_MPS;
        g_musb_udc.out_ep[0].ep_type = 0x00;
        g_musb_udc.out_ep[0].ep_enable = true;
        g_musb_udc.in_ep[0].ep_mps = USB_CTRL_EP_MPS;
        g_musb_udc.in_ep[0].ep_type = 0x00;
        g_musb_udc.in_ep[0].ep_enable = true;
        return 0;
    }

    if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
        USB_LOG_ERR("Ep addr %d overflow\r\n", ep_cfg->ep_addr);
        return -1;
    }

    old_ep_idx = musb_get_active_ep();
    musb_set_active_ep(ep_idx);

    if (USB_EP_DIR_IS_OUT(ep_cfg->ep_addr)) {
        g_musb_udc.out_ep[ep_idx].ep_mps = ep_cfg->ep_mps;
        g_musb_udc.out_ep[ep_idx].ep_type = ep_cfg->ep_type;
        g_musb_udc.out_ep[ep_idx].ep_enable = true;

        HWREGB(USB_BASE + MUSB_IND_RXMAP_OFFSET) = ep_cfg->ep_mps >> 3;

        //
        // Allow auto clearing of RxPktRdy when packet of size max packet
        // has been unloaded from the FIFO.
        //
        if (ui32Flags & USB_EP_AUTO_CLEAR) {
            ui32Register = USB_RXCSRH1_AUTOCL;
        }
        //
        // Configure the DMA mode.
        //
        if (ui32Flags & USB_EP_DMA_MODE_1) {
            ui32Register |= USB_RXCSRH1_DMAEN | USB_RXCSRH1_DMAMOD;
        } else if (ui32Flags & USB_EP_DMA_MODE_0) {
            ui32Register |= USB_RXCSRH1_DMAEN;
        }
        //
        // If requested, disable NYET responses for high-speed bulk and
        // interrupt endpoints.
        //
        if (ui32Flags & USB_EP_DIS_NYET) {
            ui32Register |= USB_RXCSRH1_DISNYET;
        }

        //
        // Enable isochronous mode if requested.
        //
        if (ep_cfg->ep_type == 0x01) {
            ui32Register |= USB_RXCSRH1_ISO;
        }

        HWREGB(USB_BASE + MUSB_IND_RXCSRH_OFFSET) = ui32Register;

        // Reset the Data toggle to zero.
        if (HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) & USB_RXCSRL1_RXRDY)
            HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) = (USB_RXCSRL1_CLRDT | USB_RXCSRL1_FLUSH);
        else
            HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) = USB_RXCSRL1_CLRDT;

#ifdef CONFIG_MUSB_DYNFIFO
        fifo_size = musb_get_fifo_size(ep_cfg->ep_mps, &used);
        //HWREGH(USB_BASE + MUSB_RX_DYNA_CONG_OFFSET) = (fifo_size << 13)
        //                                              + (g_musb_udc.fifo_size_offset >> 3);
        //USB_LOG_INFO("RXDYNA: %x, fifo_size %d, used %d, orig %x\n", HWREGH(USB_BASE + MUSB_RX_DYNA_CONG_OFFSET), fifo_size, used,
        //    (fifo_size << 13) + (g_musb_udc.fifo_size_offset >> 3));
        HWREGB(USB_BASE + MUSB_RX_FIFO1) = g_musb_udc.fifo_size_offset >> 3;
        HWREGB(USB_BASE + MUSB_RX_FIFO2) = fifo_size << 5;
        USB_LOG_DBG("RXDYNA: %x/%x, orig %x %x\n", HWREGB(USB_BASE + MUSB_RX_FIFO1), HWREGB(USB_BASE + MUSB_RX_FIFO2),
            g_musb_udc.fifo_size_offset >> 3, fifo_size << 5);

        g_musb_udc.fifo_size_offset += used;
#endif
    } else {
        g_musb_udc.in_ep[ep_idx].ep_mps = ep_cfg->ep_mps;
        g_musb_udc.in_ep[ep_idx].ep_type = ep_cfg->ep_type;
        g_musb_udc.in_ep[ep_idx].ep_enable = true;

        HWREGB(USB_BASE + MUSB_IND_TXMAP_OFFSET) = ep_cfg->ep_mps >> 3;

        //
        // Allow auto setting of TxPktRdy when max packet size has been loaded
        // into the FIFO.
        //
        if (ui32Flags & USB_EP_AUTO_SET) {
            ui32Register |= USB_TXCSRH1_AUTOSET;
        }

        //
        // Configure the DMA mode.
        //
        if (ui32Flags & USB_EP_DMA_MODE_1) {
            ui32Register |= USB_TXCSRH1_DMAEN | USB_TXCSRH1_DMAMOD;
        } else if (ui32Flags & USB_EP_DMA_MODE_0) {
            ui32Register |= USB_TXCSRH1_DMAEN;
        }

        //
        // Enable isochronous mode if requested.
        //
        if (ep_cfg->ep_type == 0x01) {
            ui32Register |= USB_TXCSRH1_ISO;
        }

        ui32Register |= USB_TXCSRH1_MODE;

        HWREGB(USB_BASE + MUSB_IND_TXCSRH_OFFSET) = ui32Register;

        // Reset the Data toggle to zero.
        if (HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) & USB_TXCSRL1_TXRDY)
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = (USB_TXCSRL1_CLRDT | USB_TXCSRL1_FLUSH);
        else
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_TXCSRL1_CLRDT;

#ifdef CONFIG_MUSB_DYNFIFO
        fifo_size = musb_get_fifo_size(ep_cfg->ep_mps, &used);

        // Dynamic FIFO Sizing: FIFO Address, FIFO size for ep.
        //HWREGH(USB_BASE + MUSB_TX_DYNA_CONG_OFFSET) = (fifo_size << 13)
        //                                              + (g_musb_udc.fifo_size_offset >> 3);
        //USB_LOG_INFO("TXDYNA: %x, fifo_size %d, used %d, orig %x\n", HWREGH(USB_BASE + MUSB_TX_DYNA_CONG_OFFSET), fifo_size, used,
        //    (fifo_size << 13) + (g_musb_udc.fifo_size_offset >> 3));

        HWREGB(USB_BASE + MUSB_TX_FIFO1) = g_musb_udc.fifo_size_offset >> 3;
        HWREGB(USB_BASE + MUSB_TX_FIFO2) = fifo_size << 5;
        USB_LOG_DBG("RXDYNA: %x/%x, orig %x %x\n", HWREGB(USB_BASE + MUSB_TX_FIFO1), HWREGB(USB_BASE + MUSB_TX_FIFO2),
            g_musb_udc.fifo_size_offset >> 3, fifo_size << 5);
#endif

        g_musb_udc.fifo_size_offset += used;
    }

    musb_set_active_ep(old_ep_idx);

    return 0;
}

int usbd_ep_close(const uint8_t ep)
{
    return 0;
}

int usbd_ep_set_stall(const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    uint8_t old_ep_idx;

    old_ep_idx = musb_get_active_ep();
    musb_set_active_ep(ep_idx);

    if (USB_EP_DIR_IS_OUT(ep)) {
        if (ep_idx == 0x00) {
            usb_ep0_state = USB_EP0_STATE_STALL;
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) |= (USB_CSRL0_STALL | USB_CSRL0_RXRDYC);
        } else {
            HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) |= USB_RXCSRL1_STALL;
        }
    } else {
        if (ep_idx == 0x00) {
            usb_ep0_state = USB_EP0_STATE_STALL;
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) |= (USB_CSRL0_STALL | USB_CSRL0_RXRDYC);
        } else {
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) |= USB_TXCSRL1_STALL;
        }
    }

    musb_set_active_ep(old_ep_idx);
    return 0;
}

int usbd_ep_clear_stall(const uint8_t ep)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    uint8_t old_ep_idx;

    old_ep_idx = musb_get_active_ep();
    musb_set_active_ep(ep_idx);

    if (USB_EP_DIR_IS_OUT(ep)) {
        if (ep_idx == 0x00) {
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) &= ~USB_CSRL0_STALLED;
        } else {
            // Clear the stall on an OUT endpoint.
            HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) &= ~(USB_RXCSRL1_STALL | USB_RXCSRL1_STALLED);
            // Reset the data toggle.
            HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) |= USB_RXCSRL1_CLRDT;
        }
    } else {
        if (ep_idx == 0x00) {
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) &= ~USB_CSRL0_STALLED;
        } else {
            // Clear the stall on an IN endpoint.
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) &= ~(USB_TXCSRL1_STALL | USB_TXCSRL1_STALLED);
            // Reset the data toggle.
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) |= USB_TXCSRL1_CLRDT;
        }
    }

    musb_set_active_ep(old_ep_idx);
    return 0;
}

int usbd_ep_is_stalled(const uint8_t ep, uint8_t *stalled)
{
    return 0;
}

int usbd_ep_start_write(const uint8_t ep, const uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    uint8_t old_ep_idx;

    if (!data && data_len) {
        return -1;
    }
    if (!g_musb_udc.in_ep[ep_idx].ep_enable) {
        return -2;
    }

    old_ep_idx = musb_get_active_ep();
    musb_set_active_ep(ep_idx);

    if (HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) & USB_TXCSRL1_TXRDY) {
        musb_set_active_ep(old_ep_idx);
        return -3;
    }

    g_musb_udc.in_ep[ep_idx].xfer_buf = (uint8_t *)data;
    g_musb_udc.in_ep[ep_idx].xfer_len = data_len;
    g_musb_udc.in_ep[ep_idx].actual_xfer_len = 0;

    if (data_len == 0) {
        if (ep_idx == 0x00) {
            if (g_musb_udc.setup.wLength == 0) {
                usb_ep0_state = USB_EP0_STATE_IN_STATUS;
            } else {
                usb_ep0_state = USB_EP0_STATE_IN_ZLP;
            }
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = (USB_CSRL0_TXRDY | USB_CSRL0_DATAEND);
        } else {
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_TXCSRL1_TXRDY;
            HWREGB(USB_BASE + MUSB_TXIEL_OFFSET) |= (1 << ep_idx);
        }
        musb_set_active_ep(old_ep_idx);
        return 0;
    }
    data_len = MIN(data_len, g_musb_udc.in_ep[ep_idx].ep_mps);

    musb_write_packet(ep_idx, (uint8_t *)data, data_len);
    HWREGB(USB_BASE + MUSB_TXIEL_OFFSET) |= (1 << ep_idx);

    if (ep_idx == 0x00) {
        usb_ep0_state = USB_EP0_STATE_IN_DATA;
        if (data_len < g_musb_udc.in_ep[ep_idx].ep_mps) {
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = (USB_CSRL0_TXRDY | USB_CSRL0_DATAEND);
        } else {
            HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_CSRL0_TXRDY;
        }
    } else {
        HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_TXCSRL1_TXRDY;
    }

    musb_set_active_ep(old_ep_idx);
    return 0;
}

#define USB_DEBUG_GPIO                 (0x0802800 +(15*4))
// MGC_FdrcStartRx
int usbd_ep_start_read(const uint8_t ep, uint8_t *data, uint32_t data_len)
{
    uint8_t ep_idx = USB_EP_GET_IDX(ep);
    uint8_t old_ep_idx;
#if 0
    uint8_t bCsr1, bCsr2;
    uint16_t read_count;
#endif

    if (!data && data_len) {
        return -1;
    }
    if (!g_musb_udc.out_ep[ep_idx].ep_enable) {
        return -2;
    }

    old_ep_idx = musb_get_active_ep();
    musb_set_active_ep(ep_idx);

    g_musb_udc.out_ep[ep_idx].xfer_buf = data;
    g_musb_udc.out_ep[ep_idx].xfer_len = data_len;
    g_musb_udc.out_ep[ep_idx].actual_xfer_len = 0;

    if (data_len == 0) {
        if (ep_idx == 0) {
            usb_ep0_state = USB_EP0_STATE_SETUP;
        }
        musb_set_active_ep(old_ep_idx);
        return 0;
    }
    if (ep_idx == 0) {
        usb_ep0_state = USB_EP0_STATE_OUT_DATA;
    } else {
#if 0
        bCsr1 = HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET);
        bCsr2 = HWREGB(USB_BASE + MUSB_IND_RXCSRH_OFFSET);

        //bCsr2 &= ~(MGC_M_RXCSR2_DMAENAB | MGC_M_RXCSR2_DMAMODE);  // USB_RXCSRL1_REQPKT
        bCsr2 &= ~(USB_RXCSRH1_DMAEN | USB_RXCSRH1_DMAMODE);

        //if (bCsr1 & (~USB_RXCSRL1_OVER))
        //{
        //    bCsr1 &= (~USB_RXCSRL1_OVER);
        //    os_printf("usb overflow\n");
        //}

        while (bCsr1 & USB_RXCSRL1_RXRDY)
        {
#ifdef CFG_CHERRYUSB_DEBUG_OUT_EP
            REG_WRITE(USB_DEBUG_GPIO, 2);
#endif
            // bCsr1 |= USB_RXCSRL1_FLUSH;
            read_count = HWREGH(USB_BASE + MUSB_IND_RXCOUNT_OFFSET);
            //os_printf("fifo has data, remain %d, xfer_len %d, actual_xfer_len %d\n", read_count,
            //    g_musb_udc.out_ep[ep_idx].xfer_len, g_musb_udc.out_ep[ep_idx].actual_xfer_len);

            // read_count = HWREGH(USB_BASE + MUSB_IND_RXCOUNT_OFFSET);

            musb_read_packet(ep_idx, g_musb_udc.out_ep[ep_idx].xfer_buf, read_count);
            HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) &= ~(USB_RXCSRL1_RXRDY);

            g_musb_udc.out_ep[ep_idx].xfer_buf += read_count;
            g_musb_udc.out_ep[ep_idx].actual_xfer_len += read_count;
            g_musb_udc.out_ep[ep_idx].xfer_len -= read_count;

            if ((read_count < g_musb_udc.out_ep[ep_idx].ep_mps) || (g_musb_udc.out_ep[ep_idx].xfer_len == 0)) {
                // HWREGB(USB_BASE + MUSB_RXIEL_OFFSET) &= ~(1 << ep_idx);
                // FIXME: maybe infinite loop here !!!
                usbd_event_ep_out_complete_handler(ep_idx, g_musb_udc.out_ep[ep_idx].actual_xfer_len);
            } else {
            }
            bCsr1 = HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET);
#ifdef CFG_CHERRYUSB_DEBUG_OUT_EP
            REG_WRITE(USB_DEBUG_GPIO, 0);
#endif
        }
        // HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) = bCsr1;
#endif

        HWREGB(USB_BASE + MUSB_RXIEL_OFFSET) |= (1 << ep_idx);
    }
    musb_set_active_ep(old_ep_idx);
    return 0;
}

static void handle_ep0(void)
{
    uint8_t ep0_status = HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET);
    uint16_t read_count;

    // os_printf("\n%s, state %d, ep0_status 0x%x\n", __func__, usb_ep0_state, ep0_status);
    /* SentStall */
    if (ep0_status & USB_CSRL0_STALLED) {
        HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) &= ~USB_CSRL0_STALLED;
        usb_ep0_state = USB_EP0_STATE_SETUP;
        // os_printf("%s %d\n", __func__, __LINE__);
        return;
    }

    /* SetupEnd */
    if (ep0_status & USB_CSRL0_SETEND) {
        HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_CSRL0_SETENDC;
    }

    /* Function Address */
    if (g_musb_udc.dev_addr > 0) {
        HWREGB(USB_BASE + MUSB_FADDR_OFFSET) = g_musb_udc.dev_addr;
        g_musb_udc.dev_addr = 0;
    }

    switch (usb_ep0_state) {
        case USB_EP0_STATE_SETUP:
            if (ep0_status & USB_CSRL0_RXRDY) {
                read_count = HWREGH(USB_BASE + MUSB_IND_RXCOUNT_OFFSET);

                if (read_count != 8) {
                    return;
                }

                musb_read_packet(0, (uint8_t *)&g_musb_udc.setup, 8);
                if (g_musb_udc.setup.wLength) {
                    HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_CSRL0_RXRDYC;
                } else {
                    HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = (USB_CSRL0_RXRDYC | USB_CSRL0_DATAEND);
                }

                // os_printf("%s %d\n", __func__, __LINE__);
                usbd_event_ep0_setup_complete_handler((uint8_t *)&g_musb_udc.setup);
            }
            break;

        case USB_EP0_STATE_IN_DATA:
            if (g_musb_udc.in_ep[0].xfer_len > g_musb_udc.in_ep[0].ep_mps) {
                g_musb_udc.in_ep[0].actual_xfer_len += g_musb_udc.in_ep[0].ep_mps;
                g_musb_udc.in_ep[0].xfer_len -= g_musb_udc.in_ep[0].ep_mps;
            } else {
                g_musb_udc.in_ep[0].actual_xfer_len += g_musb_udc.in_ep[0].xfer_len;
                g_musb_udc.in_ep[0].xfer_len = 0;
            }

            usbd_event_ep_in_complete_handler(0x80, g_musb_udc.in_ep[0].actual_xfer_len);

            break;
        case USB_EP0_STATE_OUT_DATA:
            if (ep0_status & USB_CSRL0_RXRDY) {
                read_count = HWREGH(USB_BASE + MUSB_IND_RXCOUNT_OFFSET);

                // os_printf("XXX: read_count %d, active ep %d\n", read_count, musb_get_active_ep());
                //if (!read_count)
                //    break;

                musb_read_packet(0, g_musb_udc.out_ep[0].xfer_buf, read_count);
                g_musb_udc.out_ep[0].xfer_buf += read_count;
                g_musb_udc.out_ep[0].actual_xfer_len += read_count;

                if (read_count < g_musb_udc.out_ep[0].ep_mps) {
                    usbd_event_ep_out_complete_handler(0x00, g_musb_udc.out_ep[0].actual_xfer_len);
                    HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = (USB_CSRL0_RXRDYC | USB_CSRL0_DATAEND);
                    usb_ep0_state = USB_EP0_STATE_IN_STATUS;
                } else {
                    HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_CSRL0_RXRDYC;
                }
            }
            break;
        case USB_EP0_STATE_IN_STATUS:
        case USB_EP0_STATE_IN_ZLP:
            usb_ep0_state = USB_EP0_STATE_SETUP;
            usbd_event_ep_in_complete_handler(0x80, 0);
            break;
    }
}

void USBD_IRQHandler(void)
{
    uint32_t is;
    uint32_t txis;
    uint32_t rxis;
    uint8_t old_ep_idx;
    uint8_t ep_idx;
    uint16_t write_count, read_count;

    is = HWREGB(USB_BASE + MUSB_IS_OFFSET);
    txis = HWREGH(USB_BASE + MUSB_TXIS_OFFSET);
    rxis = HWREGH(USB_BASE + MUSB_RXIS_OFFSET);

    HWREGB(USB_BASE + MUSB_IS_OFFSET) = is;

    old_ep_idx = musb_get_active_ep();

    /* Receive a reset signal from the USB bus */
    if (is & USB_IS_RESET) {
        memset(&g_musb_udc, 0, sizeof(struct musb_udc));
        g_musb_udc.fifo_size_offset = USB_CTRL_EP_MPS;
        usbd_event_reset_handler();
        HWREGB(USB_BASE + MUSB_TXIEL_OFFSET) = USB_TXIE_EP0;
        HWREGB(USB_BASE + MUSB_RXIEL_OFFSET) = 0;

        for (uint8_t i = 1; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
            musb_set_active_ep(i);
            HWREGH(USB_BASE + MUSB_RX_DYNA_CONG_OFFSET) = 0;
            HWREGH(USB_BASE + MUSB_TX_DYNA_CONG_OFFSET) = 0;
        }
        usb_ep0_state = USB_EP0_STATE_SETUP;
    }

    if (is & USB_IS_SOF) {
    }

    if (is & USB_IS_RESUME) {
    }

    if (is & USB_IS_SUSPEND) {
    }

    txis &= HWREGB(USB_BASE + MUSB_TXIEL_OFFSET);
    /* Handle EP0 interrupt */
    if (txis & USB_TXIE_EP0) {
        HWREGH(USB_BASE + MUSB_TXIS_OFFSET) = USB_TXIE_EP0;
        musb_set_active_ep(0);
        handle_ep0();
        txis &= ~USB_TXIE_EP0;
    }

    ep_idx = 1;
    while (txis) {
        if (txis & (1 << ep_idx)) {
            musb_set_active_ep(ep_idx);
            HWREGH(USB_BASE + MUSB_TXIS_OFFSET) = (1 << ep_idx);
            if (HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) & USB_TXCSRL1_UNDRN) {
                HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) &= ~USB_TXCSRL1_UNDRN;
            }

            if (g_musb_udc.in_ep[ep_idx].xfer_len > g_musb_udc.in_ep[ep_idx].ep_mps) {
                g_musb_udc.in_ep[ep_idx].xfer_buf += g_musb_udc.in_ep[ep_idx].ep_mps;
                g_musb_udc.in_ep[ep_idx].actual_xfer_len += g_musb_udc.in_ep[ep_idx].ep_mps;
                g_musb_udc.in_ep[ep_idx].xfer_len -= g_musb_udc.in_ep[ep_idx].ep_mps;
            } else {
                g_musb_udc.in_ep[ep_idx].xfer_buf += g_musb_udc.in_ep[ep_idx].xfer_len;
                g_musb_udc.in_ep[ep_idx].actual_xfer_len += g_musb_udc.in_ep[ep_idx].xfer_len;
                g_musb_udc.in_ep[ep_idx].xfer_len = 0;
            }

            if (g_musb_udc.in_ep[ep_idx].xfer_len == 0) {
                HWREGB(USB_BASE + MUSB_TXIEL_OFFSET) &= ~(1 << ep_idx);
                usbd_event_ep_in_complete_handler(ep_idx | 0x80, g_musb_udc.in_ep[ep_idx].actual_xfer_len);
            } else {
                write_count = MIN(g_musb_udc.in_ep[ep_idx].xfer_len, g_musb_udc.in_ep[ep_idx].ep_mps);

                musb_write_packet(ep_idx, g_musb_udc.in_ep[ep_idx].xfer_buf, write_count);
                HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET) = USB_TXCSRL1_TXRDY;
            }

            txis &= ~(1 << ep_idx);
        }
        ep_idx++;
    }

    rxis &= HWREGB(USB_BASE + MUSB_RXIEL_OFFSET);
    ep_idx = 1;
    while (rxis) {
        if (rxis & (1 << ep_idx)) {
            musb_set_active_ep(ep_idx);
            HWREGH(USB_BASE + MUSB_RXIS_OFFSET) = (1 << ep_idx);
            if (HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) & USB_RXCSRL1_RXRDY) {
                read_count = HWREGH(USB_BASE + MUSB_IND_RXCOUNT_OFFSET);

                musb_read_packet(ep_idx, g_musb_udc.out_ep[ep_idx].xfer_buf, read_count);

                g_musb_udc.out_ep[ep_idx].xfer_buf += read_count;
                g_musb_udc.out_ep[ep_idx].actual_xfer_len += read_count;
                g_musb_udc.out_ep[ep_idx].xfer_len -= read_count;

                if ((read_count < g_musb_udc.out_ep[ep_idx].ep_mps) || (g_musb_udc.out_ep[ep_idx].xfer_len == 0)) {
                    HWREGB(USB_BASE + MUSB_RXIEL_OFFSET) &= ~(1 << ep_idx);
                    usbd_event_ep_out_complete_handler(ep_idx, g_musb_udc.out_ep[ep_idx].actual_xfer_len);
                } else {
                }

                HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET) &= ~(USB_RXCSRL1_RXRDY);
            }

            rxis &= ~(1 << ep_idx);
        }
        ep_idx++;
    }

    musb_set_active_ep(old_ep_idx);
}

void usbd_dump_reg()
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    uint8_t old_ep_idx;
    old_ep_idx = musb_get_active_ep();


    os_printf("FADDR: %x\n", HWREGB(USB_BASE + MUSB_FADDR_OFFSET));
    os_printf("POWER: %x\n", HWREGB(USB_BASE + MUSB_POWER_OFFSET));
    os_printf("TXIEL: %x\n", HWREGB(USB_BASE + MUSB_TXIEL_OFFSET));
    os_printf("TXIEH: %x\n", HWREGB(USB_BASE + MUSB_TXIEH_OFFSET));
    os_printf("RXIEL: %x\n", HWREGB(USB_BASE + MUSB_RXIEL_OFFSET));
    os_printf("RXIEH: %x\n", HWREGB(USB_BASE + MUSB_RXIEH_OFFSET));
    os_printf("IE: %x\n", HWREGB(USB_BASE + MUSB_IE_OFFSET));
    os_printf("DEVCTL: %x\n", HWREGB(USB_BASE + MUSB_DEVCTL_OFFSET));

    musb_set_active_ep(0);
    os_printf("--0--\n");
    //os_printf("FADDR: %x\n", HWREGB(USB_BASE + MUSB_IND_TXMAP_OFFSET));
    os_printf("TXCSL: %x\n", HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET));
    os_printf("TXCSH: %x\n", HWREGB(USB_BASE + MUSB_IND_TXCSRH_OFFSET));
    //os_printf("TXIEH: %x\n", HWREGB(USB_BASE + MUSB_IND_RXMAP_OFFSET));
    //os_printf("RXIEL: %x\n", HWREGH(USB_BASE + MUSB_TX_DYNA_CONG_OFFSET));
    //os_printf("RXIEH: %x\n", HWREGB(USB_BASE + MUSB_RX_DYNA_CONG_OFFSET));


    musb_set_active_ep(1);
    os_printf("--1--\n");
    os_printf("TXMAP: %x\n", HWREGB(USB_BASE + MUSB_IND_TXMAP_OFFSET));
    os_printf("TXCSL: %x\n", HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET));
    os_printf("TXCSH: %x\n", HWREGB(USB_BASE + MUSB_IND_TXCSRH_OFFSET));
    os_printf("RXMAP: %x\n", HWREGB(USB_BASE + MUSB_IND_RXMAP_OFFSET));
    os_printf("RXCSL: %x\n", HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET));
    os_printf("RXCSH: %x\n", HWREGB(USB_BASE + MUSB_IND_RXCSRH_OFFSET));
    os_printf("TXFIFO: %x\n", HWREGH(USB_BASE + MUSB_TX_DYNA_CONG_OFFSET));
    os_printf("RXFIFO: %x\n", HWREGB(USB_BASE + MUSB_RX_DYNA_CONG_OFFSET));


    musb_set_active_ep(2);
    os_printf("--2--\n");
    os_printf("TXMAP: %x\n", HWREGB(USB_BASE + MUSB_IND_TXMAP_OFFSET));
    os_printf("TXCSL: %x\n", HWREGB(USB_BASE + MUSB_IND_TXCSRL_OFFSET));
    os_printf("TXCSH: %x\n", HWREGB(USB_BASE + MUSB_IND_TXCSRH_OFFSET));
    os_printf("RXMAP: %x\n", HWREGB(USB_BASE + MUSB_IND_RXMAP_OFFSET));
    os_printf("RXCSL: %x\n", HWREGB(USB_BASE + MUSB_IND_RXCSRL_OFFSET));
    os_printf("RXCSH: %x\n", HWREGB(USB_BASE + MUSB_IND_RXCSRH_OFFSET));
    os_printf("TXFIFO: %x\n", HWREGH(USB_BASE + MUSB_TX_DYNA_CONG_OFFSET));
    os_printf("RXFIFO: %x\n", HWREGB(USB_BASE + MUSB_RX_DYNA_CONG_OFFSET));



    musb_set_active_ep(old_ep_idx);

    GLOBAL_INT_RESTORE();
}
