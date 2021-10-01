/*
 * Copyright (c) 2019 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_usdhc

#include <sys/__assert.h>
#include <drivers/disk.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <fsl_usdhc.h>

#include "sdmmc_sdhc.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(usdhc, CONFIG_SDMMC_LOG_LEVEL);

enum usdhc_cmd_type {
	USDHC_CMD_TYPE_NORMAL = 0U,
	/*!< Normal command */
	USDHC_CMD_TYPE_SUSPEND = 1U,
	/*!< Suspend command */
	USDHC_CMD_TYPE_RESUME = 2U,
	/*!< Resume command */
	USDHC_CMD_TYPE_ABORT = 3U,
	/*!< Abort command */
	USDHC_CMD_TYPE_EMPTY = 4U,
	/*!< Empty command */
};

enum usdhc_status_flag {
	USDHC_CMD_INHIBIT_FLAG =
		USDHC_PRES_STATE_CIHB_MASK,
	/*!< Command inhibit */
	USDHC_DATA_INHIBIT_FLAG =
		USDHC_PRES_STATE_CDIHB_MASK,
	/*!< Data inhibit */
	USDHC_DATA_LINE_ACTIVE_FLAG =
		USDHC_PRES_STATE_DLA_MASK,
	/*!< Data line active */
	USDHC_SD_CLK_STATUS_FLAG =
		USDHC_PRES_STATE_SDSTB_MASK,
	/*!< SD bus clock stable */
	USDHC_WRITE_ACTIVE_FLAG =
		USDHC_PRES_STATE_WTA_MASK,
	/*!< Write transfer active */
	USDHC_READ_ACTIVE_FLAG =
		USDHC_PRES_STATE_RTA_MASK,
	/*!< Read transfer active */
	USDHC_BUF_WRITE_ENABLE_FLAG =
		USDHC_PRES_STATE_BWEN_MASK,
	/*!< Buffer write enable */
	USDHC_BUF_READ_ENABLE_FLAG =
		USDHC_PRES_STATE_BREN_MASK,
	/*!< Buffer read enable */
	USDHC_RETUNING_REQ_FLAG =
		USDHC_PRES_STATE_RTR_MASK,
	/*!< re-tuning request flag ,only used for SDR104 mode */
	USDHC_DELAY_SETTING_DONE_FLAG =
		USDHC_PRES_STATE_TSCD_MASK,
	/*!< delay setting finished flag */
	USDHC_CARD_INSERTED_FLAG =
		USDHC_PRES_STATE_CINST_MASK,
	/*!< Card inserted */
	USDHC_CMD_LINE_LEVEL_FLAG =
		USDHC_PRES_STATE_CLSL_MASK,
	/*!< Command line signal level */
	USDHC_DATA0_LINE_LEVEL_FLAG =
		1U << USDHC_PRES_STATE_DLSL_SHIFT,
	/*!< Data0 line signal level */
	USDHC_DATA1_LINE_LEVEL_FLAG =
		1U << (USDHC_PRES_STATE_DLSL_SHIFT + 1U),
	/*!< Data1 line signal level */
	USDHC_DATA2_LINE_LEVEL_FLAG =
		1U << (USDHC_PRES_STATE_DLSL_SHIFT + 2U),
	/*!< Data2 line signal level */
	USDHC_DATA3_LINE_LEVEL_FLAG =
		1U << (USDHC_PRES_STATE_DLSL_SHIFT + 3U),
	/*!< Data3 line signal level */
	USDHC_DATA4_LINE_LEVEL_FLAG =
		1U << (USDHC_PRES_STATE_DLSL_SHIFT + 4U),
	/*!< Data4 line signal level */
	USDHC_DATA5_LINE_LEVEL_FLAG =
		1U << (USDHC_PRES_STATE_DLSL_SHIFT + 5U),
	/*!< Data5 line signal level */
	USDHC_DATA6_LINE_LEVEL_FLAG =
		1U << (USDHC_PRES_STATE_DLSL_SHIFT + 6U),
	/*!< Data6 line signal level */
	USDHC_DATA7_LINE_LEVEL_FLAG =
		(int)(1U << (USDHC_PRES_STATE_DLSL_SHIFT + 7U)),
	/*!< Data7 line signal level */
};

enum usdhc_transfer_flag {
	USDHC_ENABLE_DMA_FLAG =
		USDHC_MIX_CTRL_DMAEN_MASK,
	/*!< Enable DMA */

	USDHC_CMD_TYPE_SUSPEND_FLAG =
		(USDHC_CMD_XFR_TYP_CMDTYP(1U)),
	/*!< Suspend command */
	USDHC_CMD_TYPE_RESUME_FLAG =
		(USDHC_CMD_XFR_TYP_CMDTYP(2U)),
	/*!< Resume command */
	USDHC_CMD_TYPE_ABORT_FLAG =
		(USDHC_CMD_XFR_TYP_CMDTYP(3U)),
	/*!< Abort command */

	USDHC_BLOCK_COUNT_FLAG =
		USDHC_MIX_CTRL_BCEN_MASK,
	/*!< Enable block count */
	USDHC_AUTO_CMD12_FLAG =
		USDHC_MIX_CTRL_AC12EN_MASK,
	/*!< Enable auto CMD12 */
	USDHC_DATA_READ_FLAG =
		USDHC_MIX_CTRL_DTDSEL_MASK,
	/*!< Enable data read */
	USDHC_MULTIPLE_BLOCK_FLAG =
		USDHC_MIX_CTRL_MSBSEL_MASK,
	/*!< Multiple block data read/write */
	USDHC_AUTO_CMD23FLAG =
		USDHC_MIX_CTRL_AC23EN_MASK,
	/*!< Enable auto CMD23 */
	USDHC_RSP_LEN_136_FLAG =
		USDHC_CMD_XFR_TYP_RSPTYP(1U),
	/*!< 136 bit response length */
	USDHC_RSP_LEN_48_FLAG =
		USDHC_CMD_XFR_TYP_RSPTYP(2U),
	/*!< 48 bit response length */
	USDHC_RSP_LEN_48_BUSY_FLAG =
		USDHC_CMD_XFR_TYP_RSPTYP(3U),
	/*!< 48 bit response length with busy status */

	USDHC_CRC_CHECK_FLAG =
		USDHC_CMD_XFR_TYP_CCCEN_MASK,
	/*!< Enable CRC check */
	USDHC_IDX_CHECK_FLAG =
		USDHC_CMD_XFR_TYP_CICEN_MASK,
	/*!< Enable index check */
	USDHC_DATA_PRESENT_FLAG =
		USDHC_CMD_XFR_TYP_DPSEL_MASK,
	/*!< Data present flag */
};

enum usdhc_int_status_flag {
	USDHC_INT_CMD_DONE_FLAG =
		USDHC_INT_STATUS_CC_MASK,
	/*!< Command complete */
	USDHC_INT_DATA_DONE_FLAG =
		USDHC_INT_STATUS_TC_MASK,
	/*!< Data complete */
	USDHC_INT_BLK_GAP_EVENT_FLAG =
		USDHC_INT_STATUS_BGE_MASK,
	/*!< Block gap event */
	USDHC_INT_DMA_DONE_FLAG =
		USDHC_INT_STATUS_DINT_MASK,
	/*!< DMA interrupt */
	USDHC_INT_BUF_WRITE_READY_FLAG =
		USDHC_INT_STATUS_BWR_MASK,
	/*!< Buffer write ready */
	USDHC_INT_BUF_READ_READY_FLAG =
		USDHC_INT_STATUS_BRR_MASK,
	/*!< Buffer read ready */
	USDHC_INT_CARD_INSERTED_FLAG =
		USDHC_INT_STATUS_CINS_MASK,
	/*!< Card inserted */
	USDHC_INT_CARD_REMOVED_FLAG =
		USDHC_INT_STATUS_CRM_MASK,
	/*!< Card removed */
	USDHC_INT_CARD_INTERRUPT_FLAG =
		USDHC_INT_STATUS_CINT_MASK,
	/*!< Card interrupt */

	USDHC_INT_RE_TUNING_EVENT_FLAG =
		USDHC_INT_STATUS_RTE_MASK,
	/*!< Re-Tuning event,only for SD3.0 SDR104 mode */
	USDHC_INT_TUNING_PASS_FLAG =
		USDHC_INT_STATUS_TP_MASK,
	/*!< SDR104 mode tuning pass flag */
	USDHC_INT_TUNING_ERR_FLAG =
		USDHC_INT_STATUS_TNE_MASK,
	/*!< SDR104 tuning error flag */

	USDHC_INT_CMD_TIMEOUT_FLAG =
		USDHC_INT_STATUS_CTOE_MASK,
	/*!< Command timeout error */
	USDHC_INT_CMD_CRC_ERR_FLAG =
		USDHC_INT_STATUS_CCE_MASK,
	/*!< Command CRC error */
	USDHC_INT_CMD_ENDBIT_ERR_FLAG =
		USDHC_INT_STATUS_CEBE_MASK,
	/*!< Command end bit error */
	USDHC_INT_CMD_IDX_ERR_FLAG =
		USDHC_INT_STATUS_CIE_MASK,
	/*!< Command index error */
	USDHC_INT_DATA_TIMEOUT_FLAG =
		USDHC_INT_STATUS_DTOE_MASK,
	/*!< Data timeout error */
	USDHC_INT_DATA_CRC_ERR_FLAG =
		USDHC_INT_STATUS_DCE_MASK,
	/*!< Data CRC error */
	USDHC_INT_DATA_ENDBIT_ERR_FLAG =
		USDHC_INT_STATUS_DEBE_MASK,
	/*!< Data end bit error */
	USDHC_INT_AUTO_CMD12_ERR_FLAG =
		USDHC_INT_STATUS_AC12E_MASK,
	/*!< Auto CMD12 error */
	USDHC_INT_DMA_ERR_FLAG =
		USDHC_INT_STATUS_DMAE_MASK,
	/*!< DMA error */

	USDHC_INT_CMD_ERR_FLAG =
		(USDHC_INT_CMD_TIMEOUT_FLAG |
		USDHC_INT_CMD_CRC_ERR_FLAG |
		USDHC_INT_CMD_ENDBIT_ERR_FLAG |
		USDHC_INT_CMD_IDX_ERR_FLAG),
	/*!< Command error */
	USDHC_INT_DATA_ERR_FLAG =
		(USDHC_INT_DATA_TIMEOUT_FLAG |
		USDHC_INT_DATA_CRC_ERR_FLAG |
		USDHC_INT_DATA_ENDBIT_ERR_FLAG |
		USDHC_INT_AUTO_CMD12_ERR_FLAG),
	/*!< Data error */
	USDHC_INT_ERR_FLAG =
		(USDHC_INT_CMD_ERR_FLAG |
		USDHC_INT_DATA_ERR_FLAG |
		USDHC_INT_DMA_ERR_FLAG),
	/*!< All error */
	USDHC_INT_DATA_FLAG =
		(USDHC_INT_DATA_DONE_FLAG |
		USDHC_INT_DMA_DONE_FLAG |
		USDHC_INT_BUF_WRITE_READY_FLAG |
		USDHC_INT_BUF_READ_READY_FLAG |
		USDHC_INT_DATA_ERR_FLAG |
		USDHC_INT_DMA_ERR_FLAG),
	/*!< Data interrupts */
	USDHC_INT_CMD_FLAG =
		(USDHC_INT_CMD_DONE_FLAG |
		USDHC_INT_CMD_ERR_FLAG),
	/*!< Command interrupts */
	USDHC_INT_CARD_DETECT_FLAG =
		(USDHC_INT_CARD_INSERTED_FLAG |
		USDHC_INT_CARD_REMOVED_FLAG),
	/*!< Card detection interrupts */
	USDHC_INT_SDR104_TUNING_FLAG =
		(USDHC_INT_RE_TUNING_EVENT_FLAG |
		USDHC_INT_TUNING_PASS_FLAG |
		USDHC_INT_TUNING_ERR_FLAG),

	USDHC_INT_ALL_FLAGS =
		(USDHC_INT_BLK_GAP_EVENT_FLAG |
		USDHC_INT_CARD_INTERRUPT_FLAG |
		USDHC_INT_CMD_FLAG |
		USDHC_INT_DATA_FLAG |
		USDHC_INT_ERR_FLAG |
		USDHC_INT_SDR104_TUNING_FLAG),
	/*!< All flags mask */
};

enum usdhc_data_bus_width {
	USDHC_DATA_BUS_WIDTH_1BIT = 0U,
	/*!< 1-bit mode */
	USDHC_DATA_BUS_WIDTH_4BIT = 1U,
	/*!< 4-bit mode */
	USDHC_DATA_BUS_WIDTH_8BIT = 2U,
	/*!< 8-bit mode */
};

#define USDHC_MAX_BLOCK_COUNT	\
	(USDHC_BLK_ATT_BLKCNT_MASK >>	\
	USDHC_BLK_ATT_BLKCNT_SHIFT)

struct usdhc_cmd {
	uint32_t index;	/*cmd idx*/
	uint32_t argument;	/*cmd arg*/
	enum usdhc_cmd_type cmd_type;
	enum sdhc_rsp_type rsp_type;
	uint32_t response[4U];
	uint32_t rsp_err_flags;
	uint32_t flags;
};

struct usdhc_data {
	bool cmd12;
	/* Enable auto CMD12 */
	bool cmd23;
	/* Enable auto CMD23 */
	bool ignore_err;
	/* Enable to ignore error event
	 * to read/write all the data
	 */
	bool data_enable;
	uint8_t data_type;
	/* this is used to distinguish
	 * the normal/tuning/boot data
	 */
	uint32_t block_size;
	/* Block size
	 */
	uint32_t block_count;
	/* Block count
	 */
	uint32_t *rx_data;
	/* Buffer to save data read
	 */
	const uint32_t *tx_data;
	/* Data buffer to write
	 */
};

enum usdhc_dma_mode {
	USDHC_DMA_SIMPLE = 0U,
	/* external DMA
	 */
	USDHC_DMA_ADMA1 = 1U,
	/* ADMA1 is selected
	 */
	USDHC_DMA_ADMA2 = 2U,
	/* ADMA2 is selected
	 */
	USDHC_EXT_DMA = 3U,
	/* external dma mode select
	 */
};

enum usdhc_burst_len {
	USDHC_INCR_BURST_LEN = 0x01U,
	/* enable burst len for INCR
	 */
	USDHC_INCR4816_BURST_LEN = 0x02U,
	/* enable burst len for INCR4/INCR8/INCR16
	 */
	USDHC_INCR4816_BURST_LEN_WRAP = 0x04U,
	/* enable burst len for INCR4/8/16 WRAP
	 */
};

struct usdhc_adma_config {
	enum usdhc_dma_mode dma_mode;
	/* DMA mode
	 */
	enum usdhc_burst_len burst_len;
	/* burst len config
	 */
	uint32_t *adma_table;
	/* ADMA table address,
	 * can't be null if transfer way is ADMA1/ADMA2
	 */
	uint32_t adma_table_words;
	/* ADMA table length united as words,
	 * can't be 0 if transfer way is ADMA1/ADMA2
	 */
};

struct usdhc_context {
	bool cmd_only;
	struct usdhc_cmd cmd;
	struct usdhc_data data;
	struct usdhc_adma_config dma_cfg;
};

enum usdhc_endian_mode {
	USDHC_BIG_ENDIAN = 0U,
	/* Big endian mode
	 */
	USDHC_HALF_WORD_BIG_ENDIAN = 1U,
	/* Half word big endian mode
	 */
	USDHC_LITTLE_ENDIAN = 2U,
	/* Little endian mode
	 */
};

struct usdhc_config {
	USDHC_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	uint8_t nusdhc;

	char *pwr_name;
	uint8_t pwr_pin;
	gpio_dt_flags_t pwr_flags;

	char *detect_name;
	uint8_t detect_pin;
	gpio_dt_flags_t detect_flags;

	void (*irq_config_func)(const struct device *dev);

	bool no_1_8_v;

	uint32_t data_timeout;
	/* Data timeout value
	 */
	enum usdhc_endian_mode endian;
	/* Endian mode
	 */
	uint8_t read_watermark;
	/* Watermark level for DMA read operation.
	 * Available range is 1 ~ 128.
	 */
	uint8_t write_watermark;
	/* Watermark level for DMA write operation.
	 * Available range is 1 ~ 128.
	 */
	uint8_t read_burst_len;
	/* Read burst len
	 */
	uint8_t write_burst_len;
	/* Write burst len
	 */
};

struct usdhc_capability {
	uint32_t max_blk_len;
	uint32_t max_blk_cnt;
	uint32_t host_flags;
};

enum host_detect_type {
	SD_DETECT_GPIO_CD,
	/* sd card detect by CD pin through GPIO
	 */
	SD_DETECT_HOST_CD,
	/* sd card detect by CD pin through host
	 */
	SD_DETECT_HOST_DATA3,
	/* sd card detect by DAT3 pin through host
	 */
};

struct usdhc_client_info {
	uint32_t busclk_hz;
	uint32_t relative_addr;
	uint32_t version;
	uint32_t card_flags;
	uint32_t raw_cid[4U];
	uint32_t raw_csd[4U];
	uint32_t raw_scr[2U];
	uint32_t raw_ocr;
	struct sd_cid cid;
	struct sd_csd csd;
	struct sd_scr scr;
	uint32_t sd_block_count;
	uint32_t sd_block_size;
	enum sd_timing_mode sd_timing;
	enum sd_driver_strength driver_strength;
	enum sd_max_current max_current;
	enum sd_voltage voltage;
};

struct usdhc_priv {
	bool host_ready;
	uint8_t status;

	const struct device *pwr_gpio;
	const struct device *detect_gpio;
	struct gpio_callback detect_cb;

	enum host_detect_type detect_type;
	bool inserted;

	uint32_t src_clk_hz;

	const struct usdhc_config *config;
	struct usdhc_capability host_capability;

	struct usdhc_client_info card_info;

	struct usdhc_context op_context;

	usdhc_handle_t host_handle;
	struct k_sem transfer_complete_sem;
	int tx_status;
};

enum usdhc_xfer_data_type {
	USDHC_XFER_NORMAL = 0U,
	/* transfer normal read/write data
	 */
	USDHC_XFER_TUNING = 1U,
	/* transfer tuning data
	 */
	USDHC_XFER_BOOT = 2U,
	/* transfer boot data
	 */
	USDHC_XFER_BOOT_CONTINUOUS = 3U,
	/* transfer boot data continuous
	 */
};

#define USDHC_ADMA1_ADDRESS_ALIGN (4096U)
#define USDHC_ADMA1_LENGTH_ALIGN (4096U)
#define USDHC_ADMA2_ADDRESS_ALIGN (4U)
#define USDHC_ADMA2_LENGTH_ALIGN (4U)

#define USDHC_ADMA2_DESCRIPTOR_LENGTH_SHIFT (16U)
#define USDHC_ADMA2_DESCRIPTOR_LENGTH_MASK (0xFFFFU)
#define USDHC_ADMA2_DESCRIPTOR_MAX_LENGTH_PER_ENTRY	\
	(USDHC_ADMA2_DESCRIPTOR_LENGTH_MASK - 3U)

#define SWAP_WORD_BYTE_SEQUENCE(x) (__REV(x))
#define SWAP_HALF_WROD_BYTE_SEQUENCE(x) (__REV16(x))

#define SDMMCHOST_NOT_SUPPORT 0U

#define CARD_BUS_FREQ_50MHZ (0U)
#define CARD_BUS_FREQ_100MHZ0 (1U)
#define CARD_BUS_FREQ_100MHZ1 (2U)
#define CARD_BUS_FREQ_200MHZ (3U)

#define CARD_BUS_STRENGTH_0 (0U)
#define CARD_BUS_STRENGTH_1 (1U)
#define CARD_BUS_STRENGTH_2 (2U)
#define CARD_BUS_STRENGTH_3 (3U)
#define CARD_BUS_STRENGTH_4 (4U)
#define CARD_BUS_STRENGTH_5 (5U)
#define CARD_BUS_STRENGTH_6 (6U)
#define CARD_BUS_STRENGTH_7 (7U)

enum usdhc_adma_flag {
	USDHC_ADMA_SINGLE_FLAG = 0U,
	USDHC_ADMA_MUTI_FLAG = 1U,
};

enum usdhc_adma2_descriptor_flag {
	USDHC_ADMA2_VALID_FLAG = (1U << 0U),
	/* Valid flag
	 */
	USDHC_ADMA2_END_FLAG = (1U << 1U),
	/* End flag
	 */
	USDHC_ADMA2_INT_FLAG = (1U << 2U),
	/* Interrupt flag
	 */
	USDHC_ADMA2_ACTIVITY1_FLAG = (1U << 4U),
	/* Activity 1 mask
	 */
	USDHC_ADMA2_ACTIVITY2_FLAG = (1U << 5U),
	/* Activity 2 mask
	 */

	USDHC_ADMA2_NOP_FLAG =
		(USDHC_ADMA2_VALID_FLAG),
	/* No operation
	 */
	USDHC_ADMA2_RESERVED_FLAG =
		(USDHC_ADMA2_ACTIVITY1_FLAG |
		USDHC_ADMA2_VALID_FLAG),
	/* Reserved
	 */
	USDHC_ADMA2_XFER_FLAG =
		(USDHC_ADMA2_ACTIVITY2_FLAG |
		USDHC_ADMA2_VALID_FLAG),
	/* Transfer type
	 */
	USDHC_ADMA2_LINK_FLAG =
		(USDHC_ADMA2_ACTIVITY1_FLAG |
		USDHC_ADMA2_ACTIVITY2_FLAG |
		USDHC_ADMA2_VALID_FLAG),
	/* Link type
	 */
};

struct usdhc_adma2_descriptor {
	uint32_t attribute;
	/*!< The control and status field */
	const uint32_t *address;
	/*!< The address field */
};

enum usdhc_card_flag {
	USDHC_HIGH_CAPACITY_FLAG =
		(1U << 1U),
	/* Support high capacity
	 */
	USDHC_4BIT_WIDTH_FLAG =
		(1U << 2U),
	/* Support 4-bit data width
	 */
	USDHC_SDHC_FLAG =
		(1U << 3U),
	/* Card is SDHC
	 */
	USDHC_SDXC_FLAG =
		(1U << 4U),
	/* Card is SDXC
	 */
	USDHC_VOL_1_8V_FLAG =
		(1U << 5U),
	/* card support 1.8v voltage
	 */
	USDHC_SET_BLK_CNT_CMD23_FLAG =
		(1U << 6U),
	/* card support cmd23 flag
	 */
	USDHC_SPEED_CLASS_CONTROL_CMD_FLAG =
		(1U << 7U),
	/* card support speed class control flag
	 */
};

enum usdhc_capability_flag {
	USDHC_SUPPORT_ADMA_FLAG =
		USDHC_HOST_CTRL_CAP_ADMAS_MASK,
	/*!< Support ADMA */
	USDHC_SUPPORT_HIGHSPEED_FLAG =
		USDHC_HOST_CTRL_CAP_HSS_MASK,
	/*!< Support high-speed */
	USDHC_SUPPORT_DMA_FLAG =
		USDHC_HOST_CTRL_CAP_DMAS_MASK,
	/*!< Support DMA */
	USDHC_SUPPORT_SUSPEND_RESUME_FLAG =
		USDHC_HOST_CTRL_CAP_SRS_MASK,
	/*!< Support suspend/resume */
	USDHC_SUPPORT_V330_FLAG =
		USDHC_HOST_CTRL_CAP_VS33_MASK,
	/*!< Support voltage 3.3V */
	USDHC_SUPPORT_V300_FLAG =
		USDHC_HOST_CTRL_CAP_VS30_MASK,
	/*!< Support voltage 3.0V */
	USDHC_SUPPORT_V180_FLAG =
		USDHC_HOST_CTRL_CAP_VS18_MASK,
	/*!< Support voltage 1.8V */
	/* Put additional two flags in
	 * HTCAPBLT_MBL's position.
	 */
	USDHC_SUPPORT_4BIT_FLAG =
		(USDHC_HOST_CTRL_CAP_MBL_SHIFT << 0U),
	/*!< Support 4 bit mode */
	USDHC_SUPPORT_8BIT_FLAG =
		(USDHC_HOST_CTRL_CAP_MBL_SHIFT << 1U),
	/*!< Support 8 bit mode */
	/* sd version 3.0 new feature */
	USDHC_SUPPORT_DDR50_FLAG =
		USDHC_HOST_CTRL_CAP_DDR50_SUPPORT_MASK,
	/*!< support DDR50 mode */

#if defined(FSL_FEATURE_USDHC_HAS_SDR104_MODE) &&\
	(!FSL_FEATURE_USDHC_HAS_SDR104_MODE)
	USDHC_SUPPORT_SDR104_FLAG = 0,
	/*!< not support SDR104 mode */
#else
	USDHC_SUPPORT_SDR104_FLAG =
		USDHC_HOST_CTRL_CAP_SDR104_SUPPORT_MASK,
	/*!< support SDR104 mode */
#endif
#if defined(FSL_FEATURE_USDHC_HAS_SDR50_MODE) &&\
	(!FSL_FEATURE_USDHC_HAS_SDR50_MODE)
	USDHC_SUPPORT_SDR50_FLAG = 0U,
	/*!< not support SDR50 mode */
#else
	USDHC_SUPPORT_SDR50_FLAG =
		USDHC_HOST_CTRL_CAP_SDR50_SUPPORT_MASK,
	/*!< support SDR50 mode */
#endif
};

#define NXP_SDMMC_MAX_VOLTAGE_RETRIES (1000U)

#define CARD_DATA0_STATUS_MASK USDHC_DATA0_LINE_LEVEL_FLAG
#define CARD_DATA1_STATUS_MASK USDHC_DATA1_LINE_LEVEL_FLAG
#define CARD_DATA2_STATUS_MASK USDHC_DATA2_LINE_LEVEL_FLAG
#define CARD_DATA3_STATUS_MASK USDHC_DATA3_LINE_LEVEL_FLAG
#define CARD_DATA0_NOT_BUSY USDHC_DATA0_LINE_LEVEL_FLAG

#define SDHC_STANDARD_TUNING_START (10U)
/*!< standard tuning start point */
#define SDHC_TUINIG_STEP (2U)
/*!< standard tuning step */
#define SDHC_RETUNING_TIMER_COUNT (0U)
/*!< Re-tuning timer */

#define USDHC_MAX_DVS	\
	((USDHC_SYS_CTRL_DVS_MASK >>	\
	USDHC_SYS_CTRL_DVS_SHIFT) + 1U)
#define USDHC_MAX_CLKFS	\
	((USDHC_SYS_CTRL_SDCLKFS_MASK >>	\
	USDHC_SYS_CTRL_SDCLKFS_SHIFT) + 1U)
#define USDHC_PREV_DVS(x) ((x) -= 1U)
#define USDHC_PREV_CLKFS(x, y) ((x) >>= (y))

#define SDMMCHOST_SUPPORT_SDR104_FREQ SD_CLOCK_208MHZ

#define USDHC_ADMA_TABLE_WORDS (8U)
#define USDHC_ADMA2_ADDR_ALIGN (4U)
#define USDHC_READ_BURST_LEN (8U)
#define USDHC_WRITE_BURST_LEN (8U)
#define USDHC_DATA_TIMEOUT (0xFU)

#define USDHC_READ_WATERMARK_LEVEL (0x80U)
#define USDHC_WRITE_WATERMARK_LEVEL (0x80U)

enum usdhc_reset {
	USDHC_RESET_ALL =
		USDHC_SYS_CTRL_RSTA_MASK,
	/*!< Reset all except card detection */
	USDHC_RESET_CMD =
		USDHC_SYS_CTRL_RSTC_MASK,
	/*!< Reset command line */
	USDHC_RESET_DATA =
		USDHC_SYS_CTRL_RSTD_MASK,
	/*!< Reset data line */

#if defined(FSL_FEATURE_USDHC_HAS_SDR50_MODE) &&\
	(!FSL_FEATURE_USDHC_HAS_SDR50_MODE)
	USDHC_RESET_TUNING = 0U,
	/*!< no reset tuning circuit bit */
#else
	USDHC_RESET_TUNING = USDHC_SYS_CTRL_RSTT_MASK,
	/*!< reset tuning circuit */
#endif

	USDHC_RESETS_All =
		(USDHC_RESET_ALL |
		USDHC_RESET_CMD |
		USDHC_RESET_DATA |
		USDHC_RESET_TUNING),
	/*!< All reset types */
};

static void usdhc_millsec_delay(unsigned int cycles_to_wait)
{
	unsigned int start = sys_clock_cycle_get_32();

	while (sys_clock_cycle_get_32() - start < (cycles_to_wait * 1000))
		;
}

uint32_t g_usdhc_boot_dummy __aligned(64);
uint32_t g_usdhc_rx_dummy[2048] __aligned(64);

static inline void usdhc_write_data(USDHC_Type *base, uint32_t data)
{
	base->DATA_BUFF_ACC_PORT = data;
}

static inline uint32_t usdhc_read_data(USDHC_Type *base)
{
	return base->DATA_BUFF_ACC_PORT;
}

static bool usdhc_hw_reset(USDHC_Type *base, uint32_t mask, uint32_t timeout)
{
	base->SYS_CTRL |= (mask & (USDHC_SYS_CTRL_RSTA_MASK |
		USDHC_SYS_CTRL_RSTC_MASK | USDHC_SYS_CTRL_RSTD_MASK));
	/* Delay some time to wait reset success. */
	while ((base->SYS_CTRL & mask)) {
		if (!timeout) {
			break;
		}
		timeout--;
	}

	return ((!timeout) ? false : true);
}

static inline void usdhc_error_recovery(USDHC_Type *base)
{
	uint32_t status = base->PRES_STATE;

	if (status & USDHC_CMD_INHIBIT_FLAG) {
		/* reset command line */
		usdhc_hw_reset(base, USDHC_RESET_CMD, 100U);
	}
	if (status & USDHC_DATA_INHIBIT_FLAG || (base->ADMA_ERR_STATUS & 0xFUL)) {
		/* reset data line */
		usdhc_hw_reset(base, USDHC_RESET_DATA, 100U);
	}
}

static int usdhc_xfer(struct usdhc_priv *priv)
{
	int error = -EIO;
	USDHC_Type *base = priv->config->base;
	usdhc_transfer_t transfer = {0};
	usdhc_data_t data;
	usdhc_command_t cmd;

	cmd.index = priv->op_context.cmd.index;
	cmd.argument = priv->op_context.cmd.argument;
	cmd.type = (usdhc_card_command_type_t)priv->op_context.cmd.cmd_type;
	cmd.responseType = (usdhc_card_response_type_t)priv->op_context.cmd.rsp_type;
	memset(&cmd.response[0], 0, sizeof(cmd.response));
	cmd.responseErrorFlags = 0U;
	cmd.flags = priv->op_context.cmd.flags;
	transfer.command = &cmd;

	if (!priv->op_context.cmd_only) {
		data.enableAutoCommand12 = priv->op_context.data.cmd12;
		data.enableAutoCommand23 = priv->op_context.data.cmd23;
		data.enableIgnoreError = priv->op_context.data.ignore_err;
		data.dataType = priv->op_context.data.data_type;
		data.blockSize = priv->op_context.data.block_size;
		data.blockCount = priv->op_context.data.block_count;
		data.rxData = priv->op_context.data.rx_data;
		data.txData = priv->op_context.data.tx_data;
		transfer.data = &data;
	} else {
		transfer.data = NULL;
	}
	priv->tx_status = 0;
	error = USDHC_TransferNonBlocking(base, &priv->host_handle, NULL,
				&transfer)  == kStatus_Success ? 0 : -EIO;
	if (error || k_sem_take(&priv->transfer_complete_sem, K_FOREVER)) {
		LOG_WRN("Transfer to SD card failed");
		error = -ETIMEDOUT;
	} else {
		if (priv->tx_status) {
			LOG_ERR("Transfer to SD card failed with error %d", priv->tx_status);
			error = priv->tx_status;
		}
		/* Populate the transfer data */
		memcpy(&priv->op_context.cmd.response[0], &cmd.response[0],
				sizeof(cmd.response));
		priv->op_context.cmd.rsp_err_flags = cmd.responseErrorFlags;
	}
	if (error) {
		usdhc_error_recovery(base);
	}
	return error;
}

static inline void usdhc_select_1_8_vol(USDHC_Type *base, bool en_1_8_v)
{
	if (en_1_8_v)
		base->VEND_SPEC |= USDHC_VEND_SPEC_VSELECT_MASK;
	else
		base->VEND_SPEC &= ~USDHC_VEND_SPEC_VSELECT_MASK;
}

static inline void usdhc_force_clk_on(USDHC_Type *base, bool on)
{
	if (on)
		base->VEND_SPEC |= USDHC_VEND_SPEC_FRC_SDCLK_ON_MASK;
	else
		base->VEND_SPEC &= ~USDHC_VEND_SPEC_FRC_SDCLK_ON_MASK;
}

static void usdhc_tuning(USDHC_Type *base, uint32_t start, uint32_t step, bool enable)
{
	uint32_t tuning_ctrl = 0U;

	if (enable) {
		/* feedback clock */
		base->MIX_CTRL |= USDHC_MIX_CTRL_FBCLK_SEL_MASK;
		/* config tuning start and step */
		tuning_ctrl = base->TUNING_CTRL;
		tuning_ctrl &= ~(USDHC_TUNING_CTRL_TUNING_START_TAP_MASK |
			USDHC_TUNING_CTRL_TUNING_STEP_MASK);
		tuning_ctrl |= (USDHC_TUNING_CTRL_TUNING_START_TAP(start) |
			USDHC_TUNING_CTRL_TUNING_STEP(step) |
			USDHC_TUNING_CTRL_STD_TUNING_EN_MASK);
		base->TUNING_CTRL = tuning_ctrl;

		/* excute tuning */
		base->AUTOCMD12_ERR_STATUS |=
			(USDHC_AUTOCMD12_ERR_STATUS_EXECUTE_TUNING_MASK |
			USDHC_AUTOCMD12_ERR_STATUS_SMP_CLK_SEL_MASK);
	} else {
		/* disable the standard tuning */
		base->TUNING_CTRL &= ~USDHC_TUNING_CTRL_STD_TUNING_EN_MASK;
		/* clear excute tuning */
		base->AUTOCMD12_ERR_STATUS &=
			~(USDHC_AUTOCMD12_ERR_STATUS_EXECUTE_TUNING_MASK |
			USDHC_AUTOCMD12_ERR_STATUS_SMP_CLK_SEL_MASK);
	}
}

int usdhc_adjust_tuning_timing(USDHC_Type *base, uint32_t delay)
{
	uint32_t clk_tune_ctrl = 0U;

	clk_tune_ctrl = base->CLK_TUNE_CTRL_STATUS;

	clk_tune_ctrl &= ~USDHC_CLK_TUNE_CTRL_STATUS_DLY_CELL_SET_PRE_MASK;

	clk_tune_ctrl |= USDHC_CLK_TUNE_CTRL_STATUS_DLY_CELL_SET_PRE(delay);

	/* load the delay setting */
	base->CLK_TUNE_CTRL_STATUS = clk_tune_ctrl;
	/* check delat setting error */
	if (base->CLK_TUNE_CTRL_STATUS &
		(USDHC_CLK_TUNE_CTRL_STATUS_PRE_ERR_MASK |
		USDHC_CLK_TUNE_CTRL_STATUS_NXT_ERR_MASK))
		return -EIO;

	return 0;
}

static inline void usdhc_set_retuning_timer(USDHC_Type *base, uint32_t counter)
{
	base->HOST_CTRL_CAP &= ~USDHC_HOST_CTRL_CAP_TIME_COUNT_RETUNING_MASK;
	base->HOST_CTRL_CAP |= USDHC_HOST_CTRL_CAP_TIME_COUNT_RETUNING(counter);
}

static inline void usdhc_set_bus_width(USDHC_Type *base,
	enum usdhc_data_bus_width width)
{
	base->PROT_CTRL = ((base->PROT_CTRL & ~USDHC_PROT_CTRL_DTW_MASK) |
		USDHC_PROT_CTRL_DTW(width));
}

static int usdhc_execute_tuning(struct usdhc_priv *priv)
{
	bool tuning_err = true;
	int ret;
	USDHC_Type *base = priv->config->base;

	/* enable the standard tuning */
	usdhc_tuning(base, SDHC_STANDARD_TUNING_START, SDHC_TUINIG_STEP, true);

	while (true) {
		/* send tuning block */
		ret = usdhc_xfer(priv);
		if (ret) {
			return ret;
		}
		usdhc_millsec_delay(10);

		/*wait excute tuning bit clear*/
		if ((base->AUTOCMD12_ERR_STATUS &
			USDHC_AUTOCMD12_ERR_STATUS_EXECUTE_TUNING_MASK)) {
			continue;
		}

		/* if tuning error , re-tuning again */
		if ((base->CLK_TUNE_CTRL_STATUS &
			(USDHC_CLK_TUNE_CTRL_STATUS_NXT_ERR_MASK |
			USDHC_CLK_TUNE_CTRL_STATUS_PRE_ERR_MASK)) &&
			tuning_err) {
			tuning_err = false;
			/* enable the standard tuning */
			usdhc_tuning(base, SDHC_STANDARD_TUNING_START,
				SDHC_TUINIG_STEP, true);
			usdhc_adjust_tuning_timing(base,
				SDHC_STANDARD_TUNING_START);
		} else {
			break;
		}
	}

	/* delay to wait the host controller stable */
	usdhc_millsec_delay(1000);

	/* check tuning result*/
	if (!(base->AUTOCMD12_ERR_STATUS &
		USDHC_AUTOCMD12_ERR_STATUS_SMP_CLK_SEL_MASK)) {
		return -EIO;
	}

	usdhc_set_retuning_timer(base, SDHC_RETUNING_TIMER_COUNT);

	return 0;
}

static int usdhc_vol_switch(struct usdhc_priv *priv)
{
	USDHC_Type *base = priv->config->base;
	int retry = 0xffff;

	while (base->PRES_STATE &
		(CARD_DATA1_STATUS_MASK | CARD_DATA2_STATUS_MASK |
		CARD_DATA3_STATUS_MASK | CARD_DATA0_NOT_BUSY)) {
		retry--;
		if (retry <= 0) {
			return -EACCES;
		}
	}

	/* host switch to 1.8V */
	usdhc_select_1_8_vol(base, true);

	usdhc_millsec_delay(20000U);

	/*enable force clock on*/
	usdhc_force_clk_on(base, true);
	/* dealy 1ms,not exactly correct when use while */
	usdhc_millsec_delay(20000U);
	/*disable force clock on*/
	usdhc_force_clk_on(base, false);

	/* check data line and cmd line status */
	retry = 0xffff;
	while (!(base->PRES_STATE &
		(CARD_DATA1_STATUS_MASK | CARD_DATA2_STATUS_MASK |
		CARD_DATA3_STATUS_MASK | CARD_DATA0_NOT_BUSY))) {
		retry--;
		if (retry <= 0) {
			return -EBUSY;
		}
	}

	return 0;
}

static inline void usdhc_op_ctx_init(struct usdhc_priv *priv,
	bool cmd_only, uint8_t cmd_idx, uint32_t arg, enum sdhc_rsp_type rsp_type)
{
	struct usdhc_cmd *cmd = &priv->op_context.cmd;
	struct usdhc_data *data = &priv->op_context.data;

	priv->op_context.cmd_only = cmd_only;

	memset((char *)cmd, 0, sizeof(struct usdhc_cmd));
	memset((char *)data, 0, sizeof(struct usdhc_data));

	cmd->index = cmd_idx;
	cmd->argument = arg;
	cmd->rsp_type = rsp_type;
}

static int usdhc_select_fun(struct usdhc_priv *priv,
	uint32_t group, uint32_t function)
{
	const struct usdhc_config *config = priv->config;
	uint32_t *fun_status;
	uint16_t fun_grp_info[6U] = {0};
	uint32_t current_fun_status = 0U, arg;
	struct usdhc_cmd *cmd = &priv->op_context.cmd;
	struct usdhc_data *data = &priv->op_context.data;
	int ret;

	/* check if card support CMD6 */
	if ((priv->card_info.version <= SD_SPEC_VER1_0) ||
		(!(priv->card_info.csd.cmd_class & SD_CMD_CLASS_SWITCH))) {
		return -EINVAL;
	}

	/* Check if card support high speed mode. */
	arg = (SD_SWITCH_CHECK << 31U | 0x00FFFFFFU);
	arg &= ~((uint32_t)(0xFU) << (group * 4U));
	arg |= (function << (group * 4U));
	usdhc_op_ctx_init(priv, 0, SDHC_SWITCH, arg, SDHC_RSP_TYPE_R1);

	data->block_size = 64U;
	data->block_count = 1U;
	data->rx_data = &g_usdhc_rx_dummy[0];
	ret = usdhc_xfer(priv);
	if (ret || (cmd->response[0U] & SDHC_R1ERR_All_FLAG))
		return -EIO;

	fun_status = data->rx_data;

	/* Switch function status byte sequence
	 * from card is big endian(MSB first).
	 */
	switch (config->endian) {
	case USDHC_LITTLE_ENDIAN:
		fun_status[0U] = SWAP_WORD_BYTE_SEQUENCE(fun_status[0U]);
		fun_status[1U] = SWAP_WORD_BYTE_SEQUENCE(fun_status[1U]);
		fun_status[2U] = SWAP_WORD_BYTE_SEQUENCE(fun_status[2U]);
		fun_status[3U] = SWAP_WORD_BYTE_SEQUENCE(fun_status[3U]);
		fun_status[4U] = SWAP_WORD_BYTE_SEQUENCE(fun_status[4U]);
		break;
	case USDHC_BIG_ENDIAN:
		break;
	case USDHC_HALF_WORD_BIG_ENDIAN:
		fun_status[0U] = SWAP_HALF_WROD_BYTE_SEQUENCE(fun_status[0U]);
		fun_status[1U] = SWAP_HALF_WROD_BYTE_SEQUENCE(fun_status[1U]);
		fun_status[2U] = SWAP_HALF_WROD_BYTE_SEQUENCE(fun_status[2U]);
		fun_status[3U] = SWAP_HALF_WROD_BYTE_SEQUENCE(fun_status[3U]);
		fun_status[4U] = SWAP_HALF_WROD_BYTE_SEQUENCE(fun_status[4U]);
		break;
	default:
		return -ENOTSUP;
	}

	fun_grp_info[5U] = (uint16_t)fun_status[0U];
	fun_grp_info[4U] = (uint16_t)(fun_status[1U] >> 16U);
	fun_grp_info[3U] = (uint16_t)(fun_status[1U]);
	fun_grp_info[2U] = (uint16_t)(fun_status[2U] >> 16U);
	fun_grp_info[1U] = (uint16_t)(fun_status[2U]);
	fun_grp_info[0U] = (uint16_t)(fun_status[3U] >> 16U);
	current_fun_status = ((fun_status[3U] & 0xFFU) << 8U) |
		(fun_status[4U] >> 24U);

	/* check if function is support */
	if (((fun_grp_info[group] & (1 << function)) == 0U) ||
		((current_fun_status >>
		(group * 4U)) & 0xFU) != function) {
		return -ENOTSUP;
	}

	/* Switch to high speed mode. */
	usdhc_op_ctx_init(priv, 0, SDHC_SWITCH, arg, SDHC_RSP_TYPE_R1);

	data->block_size = 64U;
	data->block_count = 1U;
	data->rx_data = &g_usdhc_rx_dummy[0];

	cmd->argument = (SD_SWITCH_SET << 31U | 0x00FFFFFFU);
	cmd->argument &= ~((uint32_t)(0xFU) << (group * 4U));
	cmd->argument |= (function << (group * 4U));

	ret = usdhc_xfer(priv);
	if (ret || (cmd->response[0U] & SDHC_R1ERR_All_FLAG))
		return -EIO;
	/* Switch function status byte sequence
	 * from card is big endian(MSB first).
	 */
	switch (config->endian) {
	case USDHC_LITTLE_ENDIAN:
		fun_status[3U] = SWAP_WORD_BYTE_SEQUENCE(fun_status[3U]);
		fun_status[4U] = SWAP_WORD_BYTE_SEQUENCE(fun_status[4U]);
		break;
	case USDHC_BIG_ENDIAN:
		break;
	case USDHC_HALF_WORD_BIG_ENDIAN:
		fun_status[3U] = SWAP_HALF_WROD_BYTE_SEQUENCE(fun_status[3U]);
		fun_status[4U] = SWAP_HALF_WROD_BYTE_SEQUENCE(fun_status[4U]);
		break;
	default:
		return -ENOTSUP;
	}
	/* According to the "switch function status[bits 511~0]" return
	 * by switch command in mode "set function":
	 * -check if group 1 is successfully changed to function 1 by checking
	 * if bits 379~376 equal value 1;
	 */
	current_fun_status = ((fun_status[3U] & 0xFFU) << 8U) |
		(fun_status[4U] >> 24U);

	if (((current_fun_status >>
		(group * 4U)) & 0xFU) != function) {
		return -EINVAL;
	}

	return 0;
}

uint32_t usdhc_set_sd_clk(USDHC_Type *base, uint32_t src_clk_hz, uint32_t sd_clk_hz)
{
	uint32_t total_div = 0U;
	uint32_t divisor = 0U;
	uint32_t prescaler = 0U;
	uint32_t sysctl = 0U;
	uint32_t nearest_freq = 0U;

	__ASSERT_NO_MSG(src_clk_hz != 0U);
	__ASSERT_NO_MSG((sd_clk_hz != 0U) && (sd_clk_hz <= src_clk_hz));

	/* calculate total divisor first */
	total_div = src_clk_hz / sd_clk_hz;
	if (total_div > (USDHC_MAX_CLKFS * USDHC_MAX_DVS)) {
		return 0U;
	}

	if (total_div) {
		/* calculate the divisor (src_clk_hz / divisor) <= sd_clk_hz */
		if ((src_clk_hz / total_div) > sd_clk_hz)
			total_div++;

		/* divide the total divisor to div and prescaler */
		if (total_div > USDHC_MAX_DVS) {
			prescaler = total_div / USDHC_MAX_DVS;
			/* prescaler must be a value which equal 2^n and
			 * smaller than SDHC_MAX_CLKFS
			 */
			while (((USDHC_MAX_CLKFS % prescaler) != 0U) ||
				(prescaler == 1U))
				prescaler++;
			/* calculate the divisor */
			divisor = total_div / prescaler;
			/* fine tuning the divisor until
			 * divisor * prescaler >= total_div
			 */
			while ((divisor * prescaler) < total_div) {
				divisor++;
				if (divisor > USDHC_MAX_DVS) {
					if ((prescaler <<= 1U) >
						USDHC_MAX_CLKFS) {
						return 0;
					}
				divisor = total_div / prescaler;
				}
			}
		} else {
			/* in this situation , divsior and SDCLKFS
			 * can generate same clock
			 * use SDCLKFS
			 */
			if (((total_div % 2U) != 0U) & (total_div != 1U)) {
				divisor = total_div;
				prescaler = 1U;
			} else {
				divisor = 1U;
				prescaler = total_div;
			}
		}
		nearest_freq = src_clk_hz / (divisor == 0U ? 1U : divisor) /
			prescaler;
	} else {
		/* in this condition , src_clk_hz = busClock_Hz, */
		/* in DDR mode , set SDCLKFS to 0, divisor = 0, actually the
		 * totoal divider = 2U
		 */
		divisor = 0U;
		prescaler = 0U;
		nearest_freq = src_clk_hz;
	}

	/* calculate the value write to register */
	if (divisor != 0U) {
		USDHC_PREV_DVS(divisor);
	}
	/* calculate the value write to register */
	if (prescaler != 0U) {
		USDHC_PREV_CLKFS(prescaler, 1U);
	}

	/* Set the SD clock frequency divisor, SD clock frequency select,
	 * data timeout counter value.
	 */
	sysctl = base->SYS_CTRL;
	sysctl &= ~(USDHC_SYS_CTRL_DVS_MASK |
		USDHC_SYS_CTRL_SDCLKFS_MASK);
	sysctl |= (USDHC_SYS_CTRL_DVS(divisor) |
		USDHC_SYS_CTRL_SDCLKFS(prescaler));
	base->SYS_CTRL = sysctl;

	/* Wait until the SD clock is stable. */
	while (!(base->PRES_STATE & USDHC_PRES_STATE_SDSTB_MASK)) {
		;
	}

	return nearest_freq;
}

static void usdhc_enable_ddr_mode(USDHC_Type *base,
	bool enable, uint32_t nibble_pos)
{
	uint32_t prescaler = (base->SYS_CTRL & USDHC_SYS_CTRL_SDCLKFS_MASK) >>
		USDHC_SYS_CTRL_SDCLKFS_SHIFT;

	if (enable) {
		base->MIX_CTRL &= ~USDHC_MIX_CTRL_NIBBLE_POS_MASK;
		base->MIX_CTRL |= (USDHC_MIX_CTRL_DDR_EN_MASK |
			USDHC_MIX_CTRL_NIBBLE_POS(nibble_pos));
		prescaler >>= 1U;
	} else {
		base->MIX_CTRL &= ~USDHC_MIX_CTRL_DDR_EN_MASK;

		if (prescaler == 0U) {
			prescaler += 1U;
		} else {
			prescaler <<= 1U;
		}
	}

	base->SYS_CTRL = (base->SYS_CTRL & (~USDHC_SYS_CTRL_SDCLKFS_MASK)) |
		USDHC_SYS_CTRL_SDCLKFS(prescaler);
}

static int usdhc_select_bus_timing(struct usdhc_priv *priv)
{
	const struct usdhc_config *config = priv->config;
	int error = -EIO;

	if (priv->card_info.voltage != SD_VOL_1_8_V) {
		/* Switch the card to high speed mode */
		if (priv->host_capability.host_flags &
			USDHC_SUPPORT_HIGHSPEED_FLAG) {
			/* group 1, function 1 ->high speed mode*/
			error = usdhc_select_fun(priv, SD_GRP_TIMING_MODE,
				SD_TIMING_SDR25_HIGH_SPEED_MODE);
			/* If the result isn't "switching to
			 * high speed mode(50MHZ)
			 * successfully or card doesn't support
			 * high speed
			 * mode". Return failed status.
			 */
			if (!error) {
				priv->card_info.sd_timing =
					SD_TIMING_SDR25_HIGH_SPEED_MODE;
				priv->card_info.busclk_hz =
					usdhc_set_sd_clk(config->base,
						priv->src_clk_hz,
						SD_CLOCK_50MHZ);
			} else if (error == -ENOTSUP) {
				/* if not support high speed,
				 * keep the card work at default mode
				 */
				return 0;
			}
		} else {
			/* if not support high speed,
			 * keep the card work at default mode
			 */
			return 0;
		}
	} else if ((USDHC_SUPPORT_SDR104_FLAG !=
		SDMMCHOST_NOT_SUPPORT) ||
		(USDHC_SUPPORT_SDR50_FLAG != SDMMCHOST_NOT_SUPPORT) ||
		(USDHC_SUPPORT_DDR50_FLAG != SDMMCHOST_NOT_SUPPORT)) {
		/* card is in UHS_I mode */
		switch (priv->card_info.sd_timing) {
			/* if not select timing mode,
			 * sdmmc will handle it automatically
			 */
		case SD_TIMING_SDR12_DFT_MODE:
		case SD_TIMING_SDR104_MODE:
			error = usdhc_select_fun(priv, SD_GRP_TIMING_MODE,
				SD_TIMING_SDR104_MODE);
			if (!error) {
				priv->card_info.sd_timing =
					SD_TIMING_SDR104_MODE;
				priv->card_info.busclk_hz =
					usdhc_set_sd_clk(config->base,
						priv->src_clk_hz,
						SDMMCHOST_SUPPORT_SDR104_FREQ);
				break;
			}
		case SD_TIMING_DDR50_MODE:
			error = usdhc_select_fun(priv, SD_GRP_TIMING_MODE,
				SD_TIMING_DDR50_MODE);
			if (!error) {
				priv->card_info.sd_timing =
					SD_TIMING_DDR50_MODE;
				priv->card_info.busclk_hz =
					usdhc_set_sd_clk(
						config->base,
						priv->src_clk_hz,
						SD_CLOCK_50MHZ);
				usdhc_enable_ddr_mode(config->base, true, 0U);
			}
			break;
		case SD_TIMING_SDR50_MODE:
			error = usdhc_select_fun(priv,
				SD_GRP_TIMING_MODE,
				SD_TIMING_SDR50_MODE);
			if (!error) {
				priv->card_info.sd_timing =
					SD_TIMING_SDR50_MODE;
				priv->card_info.busclk_hz =
					usdhc_set_sd_clk(
						config->base,
						priv->src_clk_hz,
						SD_CLOCK_100MHZ);
			}
			break;
		case SD_TIMING_SDR25_HIGH_SPEED_MODE:
			error = usdhc_select_fun(priv, SD_GRP_TIMING_MODE,
				SD_TIMING_SDR25_HIGH_SPEED_MODE);
			if (!error) {
				priv->card_info.sd_timing =
					SD_TIMING_SDR25_HIGH_SPEED_MODE;
				priv->card_info.busclk_hz =
					usdhc_set_sd_clk(
						config->base,
						priv->src_clk_hz,
						SD_CLOCK_50MHZ);
			}
			break;

		default:
			break;
		}
	}

	/* SDR50 and SDR104 mode need tuning */
	if ((priv->card_info.sd_timing == SD_TIMING_SDR50_MODE) ||
		(priv->card_info.sd_timing == SD_TIMING_SDR104_MODE)) {
		struct usdhc_cmd *cmd = &priv->op_context.cmd;
		struct usdhc_data *data = &priv->op_context.data;

		/* config IO strength in IOMUX*/
		if (priv->card_info.sd_timing == SD_TIMING_SDR50_MODE) {
			imxrt_usdhc_pinmux(config->nusdhc, false,
				CARD_BUS_FREQ_100MHZ1,
				CARD_BUS_STRENGTH_7);
		} else {
			imxrt_usdhc_pinmux(config->nusdhc, false,
				CARD_BUS_FREQ_200MHZ,
				CARD_BUS_STRENGTH_7);
		}
		/* execute tuning */
		priv->op_context.cmd_only = 0;

		memset((char *)cmd, 0, sizeof(struct usdhc_cmd));
		memset((char *)data, 0, sizeof(struct usdhc_data));

		cmd->index = SDHC_SEND_TUNING_BLOCK;
		cmd->rsp_type = SDHC_RSP_TYPE_R1;

		data->block_size = 64;
		data->block_count = 1U;
		data->rx_data = &g_usdhc_rx_dummy[0];
		data->data_type = USDHC_XFER_TUNING;
		error = usdhc_execute_tuning(priv);
		if (error)
			return error;
	} else {
		/* set default IO strength to 4 to cover card adapter driver
		 * strength difference
		 */
		imxrt_usdhc_pinmux(config->nusdhc, false,
			CARD_BUS_FREQ_100MHZ1,
			CARD_BUS_STRENGTH_4);
	}

	return error;
}

static int usdhc_write_sector(void *bus_data, const uint8_t *buf, uint32_t sector,
		     uint32_t count)
{
	struct usdhc_priv *priv = bus_data;
	struct usdhc_cmd *cmd = &priv->op_context.cmd;
	struct usdhc_data *data = &priv->op_context.data;

	memset((char *)cmd, 0, sizeof(struct usdhc_cmd));
	memset((char *)data, 0, sizeof(struct usdhc_data));

	priv->op_context.cmd_only = 0;
	cmd->index = SDHC_WRITE_MULTIPLE_BLOCK;
	data->block_size = priv->card_info.sd_block_size;
	data->block_count = count;
	data->tx_data = (const uint32_t *)buf;
	data->cmd12 = true;
	if (data->block_count == 1U) {
		cmd->index = SDHC_WRITE_BLOCK;
	}
	cmd->argument = sector;
	if (!(priv->card_info.card_flags & SDHC_HIGH_CAPACITY_FLAG)) {
		cmd->argument *= priv->card_info.sd_block_size;
	}
	cmd->rsp_type = SDHC_RSP_TYPE_R1;
	cmd->rsp_err_flags = SDHC_R1ERR_All_FLAG;

	return usdhc_xfer(priv);
}

static int usdhc_read_sector(void *bus_data, uint8_t *buf, uint32_t sector,
		     uint32_t count)
{
	struct usdhc_priv *priv = bus_data;
	struct usdhc_cmd *cmd = &priv->op_context.cmd;
	struct usdhc_data *data = &priv->op_context.data;

	memset((char *)cmd, 0, sizeof(struct usdhc_cmd));
	memset((char *)data, 0, sizeof(struct usdhc_data));

	priv->op_context.cmd_only = 0;
	cmd->index = SDHC_READ_MULTIPLE_BLOCK;
	data->block_size = priv->card_info.sd_block_size;
	data->block_count = count;
	data->rx_data = (uint32_t *)buf;
	data->cmd12 = true;

	if (data->block_count == 1U) {
		cmd->index = SDHC_READ_SINGLE_BLOCK;
	}

	cmd->argument = sector;
	if (!(priv->card_info.card_flags & SDHC_HIGH_CAPACITY_FLAG)) {
		cmd->argument *= priv->card_info.sd_block_size;
	}

	cmd->rsp_type = SDHC_RSP_TYPE_R1;
	cmd->rsp_err_flags = SDHC_R1ERR_All_FLAG;

	return usdhc_xfer(priv);
}

static bool usdhc_set_sd_active(USDHC_Type *base)
{
	uint32_t timeout = 0xffff;

	base->SYS_CTRL |= USDHC_SYS_CTRL_INITA_MASK;
	/* Delay some time to wait card become active state. */
	while ((base->SYS_CTRL & USDHC_SYS_CTRL_INITA_MASK) ==
		USDHC_SYS_CTRL_INITA_MASK) {
		if (!timeout) {
			break;
		}
		timeout--;
	}

	return ((!timeout) ? false : true);
}

static void usdhc_get_host_capability(USDHC_Type *base,
	struct usdhc_capability *capability)
{
	uint32_t host_cap;
	uint32_t max_blk_len;

	host_cap = base->HOST_CTRL_CAP;

	/* Get the capability of USDHC. */
	max_blk_len = ((host_cap & USDHC_HOST_CTRL_CAP_MBL_MASK) >>
		USDHC_HOST_CTRL_CAP_MBL_SHIFT);
	capability->max_blk_len = (512U << max_blk_len);
	/* Other attributes not in HTCAPBLT register. */
	capability->max_blk_cnt = USDHC_MAX_BLOCK_COUNT;
	capability->host_flags = (host_cap & (USDHC_SUPPORT_ADMA_FLAG |
		USDHC_SUPPORT_HIGHSPEED_FLAG | USDHC_SUPPORT_DMA_FLAG |
		USDHC_SUPPORT_SUSPEND_RESUME_FLAG | USDHC_SUPPORT_V330_FLAG));
	capability->host_flags |= (host_cap & USDHC_SUPPORT_V300_FLAG);
	capability->host_flags |= (host_cap & USDHC_SUPPORT_V180_FLAG);
	capability->host_flags |=
		(host_cap & (USDHC_SUPPORT_DDR50_FLAG |
			USDHC_SUPPORT_SDR104_FLAG |
			USDHC_SUPPORT_SDR50_FLAG));
	/* USDHC support 4/8 bit data bus width. */
	capability->host_flags |= (USDHC_SUPPORT_4BIT_FLAG |
		USDHC_SUPPORT_8BIT_FLAG);
}

static void usdhc_host_hw_init(USDHC_Type *base,
	const struct usdhc_config *config)
{
	uint32_t proctl, sysctl, wml;
	uint32_t int_mask;

	__ASSERT_NO_MSG(config);
	__ASSERT_NO_MSG((config->write_watermark >= 1U) &&
			(config->write_watermark <= 128U));
	__ASSERT_NO_MSG((config->read_watermark >= 1U) &&
			(config->read_watermark <= 128U));
	__ASSERT_NO_MSG(config->write_burst_len <= 16U);

	/* Reset USDHC. */
	usdhc_hw_reset(base, USDHC_RESET_ALL, 100U);

	proctl = base->PROT_CTRL;
	wml = base->WTMK_LVL;
	sysctl = base->SYS_CTRL;

	proctl &= ~(USDHC_PROT_CTRL_EMODE_MASK | USDHC_PROT_CTRL_DMASEL_MASK);
	/* Endian mode*/
	proctl |= USDHC_PROT_CTRL_EMODE(config->endian);

	/* Watermark level */
	wml &= ~(USDHC_WTMK_LVL_RD_WML_MASK |
			USDHC_WTMK_LVL_WR_WML_MASK |
			USDHC_WTMK_LVL_RD_BRST_LEN_MASK |
			USDHC_WTMK_LVL_WR_BRST_LEN_MASK);
	wml |= (USDHC_WTMK_LVL_RD_WML(config->read_watermark) |
			USDHC_WTMK_LVL_WR_WML(config->write_watermark) |
			USDHC_WTMK_LVL_RD_BRST_LEN(config->read_burst_len) |
			USDHC_WTMK_LVL_WR_BRST_LEN(config->write_burst_len));

	/* config the data timeout value */
	sysctl &= ~USDHC_SYS_CTRL_DTOCV_MASK;
	sysctl |= USDHC_SYS_CTRL_DTOCV(config->data_timeout);

	base->SYS_CTRL = sysctl;
	base->WTMK_LVL = wml;
	base->PROT_CTRL = proctl;

	/* disable internal DMA and DDR mode */
	base->MIX_CTRL &= ~(USDHC_MIX_CTRL_DMAEN_MASK |
		USDHC_MIX_CTRL_DDR_EN_MASK);

	int_mask = (USDHC_INT_CMD_FLAG | USDHC_INT_CARD_DETECT_FLAG |
		USDHC_INT_DATA_FLAG | USDHC_INT_SDR104_TUNING_FLAG |
		USDHC_INT_BLK_GAP_EVENT_FLAG);

	base->INT_STATUS_EN |= int_mask;

}

static void usdhc_cd_gpio_cb(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct usdhc_priv *priv =
		CONTAINER_OF(cb, struct usdhc_priv, detect_cb);
	const struct usdhc_config *config = priv->config;

	gpio_pin_interrupt_configure(dev, config->detect_pin, GPIO_INT_DISABLE);
}

static int usdhc_cd_gpio_init(const struct device *detect_gpio,
			      uint32_t pin, gpio_dt_flags_t flags,
			      struct gpio_callback *callback)
{
	int ret;

	ret = gpio_pin_configure(detect_gpio, pin, GPIO_INPUT | flags);
	if (ret)
		return ret;

	gpio_init_callback(callback, usdhc_cd_gpio_cb, BIT(pin));

	return gpio_add_callback(detect_gpio, callback);
}

static void usdhc_host_reset(struct usdhc_priv *priv)
{
	USDHC_Type *base = priv->config->base;

	usdhc_select_1_8_vol(base, false);
	usdhc_enable_ddr_mode(base, false, 0);
	usdhc_tuning(base, SDHC_STANDARD_TUNING_START, SDHC_TUINIG_STEP, false);
#if FSL_FEATURE_USDHC_HAS_HS400_MODE
	/* Disable HS400 mode */
	/* Disable DLL */
#endif
}

static int usdhc_app_host_cmd(struct usdhc_priv *priv, int retry,
	uint32_t arg, uint8_t app_cmd, uint32_t app_arg, enum sdhc_rsp_type rsp_type,
	enum sdhc_rsp_type app_rsp_type, bool app_cmd_only)
{
	struct usdhc_cmd *cmd = &priv->op_context.cmd;
	int ret;

APP_CMD_XFER_AGAIN:
	priv->op_context.cmd_only = 1;
	cmd->index = SDHC_APP_CMD;
	cmd->argument = arg;
	cmd->rsp_type = rsp_type;
	ret = usdhc_xfer(priv);
	retry--;
	if (ret && retry > 0) {
		goto APP_CMD_XFER_AGAIN;
	}

	priv->op_context.cmd_only = app_cmd_only;
	cmd->index = app_cmd;
	cmd->argument = app_arg;
	cmd->rsp_type = app_rsp_type;
	ret = usdhc_xfer(priv);
	if (ret && retry > 0) {
		goto APP_CMD_XFER_AGAIN;
	}

	return ret;
}

static int usdhc_sd_init(struct usdhc_priv *priv)
{
	const struct usdhc_config *config = priv->config;
	USDHC_Type *base = config->base;
	uint32_t app_cmd_41_arg = 0U;
	int ret, retry;
	struct usdhc_cmd *cmd = &priv->op_context.cmd;
	struct usdhc_data *data = &priv->op_context.data;

	if (!priv->host_ready) {
		return -ENODEV;
	}

	/* reset variables */
	priv->card_info.card_flags = 0U;
	/* set DATA bus width 1bit at beginning*/
	usdhc_set_bus_width(base, USDHC_DATA_BUS_WIDTH_1BIT);
	/*set card freq to 400KHZ at begging*/
	priv->card_info.busclk_hz =
		usdhc_set_sd_clk(base, priv->src_clk_hz,
			SDMMC_CLOCK_400KHZ);
	/* send card active */
	ret = usdhc_set_sd_active(base);
	if (ret == false) {
		return -EIO;
	}

	/* Get host capability. */
	usdhc_get_host_capability(base, &priv->host_capability);

	/* card go idle */
	usdhc_op_ctx_init(priv, 1, SDHC_GO_IDLE_STATE, 0, SDHC_RSP_TYPE_NONE);

	ret = usdhc_xfer(priv);
	if (ret) {
		return ret;
	}

	if (USDHC_SUPPORT_V330_FLAG != SDMMCHOST_NOT_SUPPORT) {
		app_cmd_41_arg |= (SD_OCR_VDD32_33FLAG | SD_OCR_VDD33_34FLAG);
		priv->card_info.voltage = SD_VOL_3_3_V;
	} else if (USDHC_SUPPORT_V300_FLAG != SDMMCHOST_NOT_SUPPORT) {
		app_cmd_41_arg |= SD_OCR_VDD29_30FLAG;
		priv->card_info.voltage = SD_VOL_3_3_V;
	}

	/* allow user select the work voltage, if not select,
	 * sdmmc will handle it automatically
	 */
	if (priv->config->no_1_8_v == false) {
		if (USDHC_SUPPORT_V180_FLAG != SDMMCHOST_NOT_SUPPORT) {
			app_cmd_41_arg |= SD_OCR_SWITCH_18_REQ_FLAG;
		}
	}

	/* Check card's supported interface condition. */
	usdhc_op_ctx_init(priv, 1, SDHC_SEND_IF_COND,
		SDHC_VHS_3V3 | SDHC_CHECK, SDHC_RSP_TYPE_R7);

	retry = 10;
	while (retry) {
		ret = usdhc_xfer(priv);
		if (!ret) {
			if ((cmd->response[0U] & 0xFFU) != SDHC_CHECK) {
				ret = -ENOTSUP;
			} else {
				break;
			}
		}
		retry--;
	}

	if (!ret) {
		/* SDHC or SDXC card */
		app_cmd_41_arg |= SD_OCR_HOST_CAP_FLAG;
		priv->card_info.card_flags |= USDHC_SDHC_FLAG;
	} else {
		/* SDSC card */
		LOG_ERR("USDHC SDSC not implemented yet!");
		return -ENOTSUP;
	}

	/* Set card interface condition according to SDHC capability and
	 * card's supported interface condition.
	 */
	int drop_cnt = 0;
APP_SEND_OP_COND_AGAIN:
	usdhc_op_ctx_init(priv, 1, 0, 0, SDHC_RSP_TYPE_NONE);
	ret = usdhc_app_host_cmd(priv, NXP_SDMMC_MAX_VOLTAGE_RETRIES, 0,
		SDHC_APP_SEND_OP_COND, app_cmd_41_arg,
		SDHC_RSP_TYPE_R1, SDHC_RSP_TYPE_R3, 1);
	if (ret) {
		LOG_ERR("APP Condition CMD failed:%d", ret);
		return ret;
	}
	if (cmd->response[0U] & SD_OCR_PWR_BUSY_FLAG) {
		/* high capacity check */
		if (cmd->response[0U] & SD_OCR_CARD_CAP_FLAG) {
			priv->card_info.card_flags |= SDHC_HIGH_CAPACITY_FLAG;
		}

		if (priv->config->no_1_8_v == false) {
			/* 1.8V support */
			if (cmd->response[0U] & SD_OCR_SWITCH_18_ACCEPT_FLAG) {
				priv->card_info.card_flags |= USDHC_VOL_1_8V_FLAG;
			}
		}
		priv->card_info.raw_ocr = cmd->response[0U];
	} else {
		drop_cnt++;
		goto APP_SEND_OP_COND_AGAIN;
	}
	LOG_DBG("SD was polled %d times before PWR busy flag", drop_cnt);

	/* check if card support 1.8V */
	if ((priv->card_info.card_flags & USDHC_VOL_1_8V_FLAG)) {
		usdhc_op_ctx_init(priv, 1, SDHC_VOL_SWITCH,
			0, SDHC_RSP_TYPE_R1);

		ret = usdhc_xfer(priv);
		if (!ret) {
			ret = usdhc_vol_switch(priv);
		}
		if (ret) {
			LOG_ERR("Voltage switch failed: %d", ret);
			return ret;
		}
		LOG_INF("Card switched to 1.8V");
		priv->card_info.voltage = SD_VOL_1_8_V;
	}

	/* Initialize card if the card is SD card. */
	usdhc_op_ctx_init(priv, 1, SDHC_ALL_SEND_CID, 0, SDHC_RSP_TYPE_R2);

	ret = usdhc_xfer(priv);
	if (!ret) {
		memcpy(priv->card_info.raw_cid, cmd->response,
			sizeof(priv->card_info.raw_cid));
		sdhc_decode_cid(&priv->card_info.cid,
			priv->card_info.raw_cid);
	} else {
		LOG_ERR("All send CID CMD failed: %d", ret);
		return ret;
	}

	usdhc_op_ctx_init(priv, 1, SDHC_SEND_RELATIVE_ADDR,
		0, SDHC_RSP_TYPE_R6);

	ret = usdhc_xfer(priv);
	if (!ret) {
		priv->card_info.relative_addr = (cmd->response[0U] >> 16U);
	} else {
		LOG_ERR("Send relative address CMD failed: %d", ret);
		return ret;
	}

	usdhc_op_ctx_init(priv, 1, SDHC_SEND_CSD,
		(priv->card_info.relative_addr << 16U), SDHC_RSP_TYPE_R2);

	ret = usdhc_xfer(priv);
	if (!ret) {
		memcpy(priv->card_info.raw_csd, cmd->response,
			sizeof(priv->card_info.raw_csd));
		sdhc_decode_csd(&priv->card_info.csd, priv->card_info.raw_csd,
			&priv->card_info.sd_block_count,
			&priv->card_info.sd_block_size);
	} else {
		LOG_ERR("Send CSD CMD failed: %d", ret);
		return ret;
	}

	usdhc_op_ctx_init(priv, 1, SDHC_SELECT_CARD,
		priv->card_info.relative_addr << 16U,
		SDHC_RSP_TYPE_R1);

	ret = usdhc_xfer(priv);
	if (ret || (cmd->response[0U] & SDHC_R1ERR_All_FLAG)) {
		LOG_ERR("Select card CMD failed: %d", ret);
		return -EIO;
	}

	usdhc_op_ctx_init(priv, 0, 0, 0, SDHC_RSP_TYPE_NONE);
	data->block_size = 8;
	data->block_count = 1;
	data->rx_data = &priv->card_info.raw_scr[0];
	ret = usdhc_app_host_cmd(priv, 1, (priv->card_info.relative_addr << 16),
		SDHC_APP_SEND_SCR, 0,
		SDHC_RSP_TYPE_R1, SDHC_RSP_TYPE_R1, 0);

	if (ret) {
		LOG_ERR("Send SCR following APP CMD failed: %d", ret);
		return ret;
	}

	switch (config->endian) {
	case USDHC_LITTLE_ENDIAN:
		priv->card_info.raw_scr[0] =
			SWAP_WORD_BYTE_SEQUENCE(priv->card_info.raw_scr[0]);
		priv->card_info.raw_scr[1] =
			SWAP_WORD_BYTE_SEQUENCE(priv->card_info.raw_scr[1]);
	break;
	case USDHC_BIG_ENDIAN:
	break;
	case USDHC_HALF_WORD_BIG_ENDIAN:
		priv->card_info.raw_scr[0U] =
			SWAP_HALF_WROD_BYTE_SEQUENCE(
				priv->card_info.raw_scr[0U]);
		priv->card_info.raw_scr[1U] =
			SWAP_HALF_WROD_BYTE_SEQUENCE(
				priv->card_info.raw_scr[1U]);
	break;
	default:
		return -EINVAL;
	}

	sdhc_decode_scr(&priv->card_info.scr, priv->card_info.raw_scr,
		&priv->card_info.version);
	if (priv->card_info.scr.sd_width & 0x4U) {
		priv->card_info.card_flags |=
			USDHC_4BIT_WIDTH_FLAG;
	}
	/* speed class control cmd */
	if (priv->card_info.scr.cmd_support & 0x01U) {
		priv->card_info.card_flags |=
			USDHC_SPEED_CLASS_CONTROL_CMD_FLAG;
	}
	/* set block count cmd */
	if (priv->card_info.scr.cmd_support & 0x02U) {
		priv->card_info.card_flags |=
			USDHC_SET_BLK_CNT_CMD23_FLAG;
	}

	/* Set to max frequency in non-high speed mode. */
	priv->card_info.busclk_hz = usdhc_set_sd_clk(base,
		priv->src_clk_hz, SD_CLOCK_25MHZ);

	/* Set to 4-bit data bus mode. */
	if ((priv->host_capability.host_flags & USDHC_SUPPORT_4BIT_FLAG) &&
		(priv->card_info.card_flags & USDHC_4BIT_WIDTH_FLAG)) {
		usdhc_op_ctx_init(priv, 1, 0, 0, SDHC_RSP_TYPE_NONE);

		ret = usdhc_app_host_cmd(priv, 1,
				(priv->card_info.relative_addr << 16),
				SDHC_APP_SET_BUS_WIDTH, 2,
				SDHC_RSP_TYPE_R1, SDHC_RSP_TYPE_R1, 1);

		if (ret) {
			LOG_ERR("Set bus width failed: %d", ret);
			return ret;
		}
		usdhc_set_bus_width(base, USDHC_DATA_BUS_WIDTH_4BIT);
	}

	if (priv->card_info.version >= SD_SPEC_VER3_0) {
		/* set sd card driver strength */
		ret = usdhc_select_fun(priv, SD_GRP_DRIVER_STRENGTH_MODE,
			priv->card_info.driver_strength);
		if (ret) {
			LOG_ERR("Set SD driver strength failed: %d", ret);
			return ret;
		}

		/* set sd card current limit */
		ret = usdhc_select_fun(priv, SD_GRP_CURRENT_LIMIT_MODE,
			priv->card_info.max_current);
		if (ret) {
			LOG_ERR("Set SD current limit failed: %d", ret);
			return ret;
		}
	}

	/* set block size */
	usdhc_op_ctx_init(priv, 1, SDHC_SET_BLOCK_SIZE,
		priv->card_info.sd_block_size, SDHC_RSP_TYPE_R1);

	ret = usdhc_xfer(priv);
	if (ret || cmd->response[0U] & SDHC_R1ERR_All_FLAG) {
		LOG_ERR("Set block size failed: %d", ret);
		return -EIO;
	}

	if (priv->card_info.version > SD_SPEC_VER1_0) {
		/* select bus timing */
		ret = usdhc_select_bus_timing(priv);
		if (ret) {
			LOG_ERR("Select bus timing failed: %d", ret);
			return ret;
		}
	}

	retry = 10;
	ret = -EIO;
	while (ret && retry >= 0) {
		ret = usdhc_read_sector(priv, (uint8_t *)g_usdhc_rx_dummy, 0, 1);
		if (!ret) {
			break;
		}
		retry--;
	}

	if (ret) {
		LOG_ERR("USDHC bus device initalization failed!");
	}

	return ret;
}

static K_MUTEX_DEFINE(z_usdhc_init_lock);

static int usdhc_board_access_init(struct usdhc_priv *priv)
{
	const struct usdhc_config *config = priv->config;
	int ret = 0;
	uint32_t gpio_level;
	USDHC_Type *base = config->base;

	if (config->pwr_name) {
		priv->pwr_gpio = device_get_binding(config->pwr_name);
		if (!priv->pwr_gpio) {
			return -ENODEV;
		}
	}

	if (config->detect_name) {
		priv->detect_type = SD_DETECT_GPIO_CD;
		priv->detect_gpio = device_get_binding(config->detect_name);
		if (!priv->detect_gpio) {
			return -ENODEV;
		}
	}

	if (priv->pwr_gpio) {
		ret = gpio_pin_configure(priv->pwr_gpio,
				config->pwr_pin,
				GPIO_OUTPUT_ACTIVE |
				config->pwr_flags);
		if (ret) {
			return ret;
		}

		/* 100ms delay to make sure SD card is stable,
		 * maybe could be shorter
		 */
		k_busy_wait(100000);
	}

	if (!priv->detect_gpio) {
		LOG_INF("USDHC detection other than GPIO");
		/* DATA3 does not monitor card insertion */
		base->PROT_CTRL &= ~USDHC_PROT_CTRL_D3CD_MASK;
		if ((base->PRES_STATE & USDHC_PRES_STATE_CINST_MASK) != 0) {
			priv->inserted = true;
		} else {
			priv->inserted = false;
			return -ENODEV;
		}
	} else {
		ret = usdhc_cd_gpio_init(priv->detect_gpio,
				config->detect_pin,
				config->detect_flags,
				&priv->detect_cb);
		if (ret) {
			return ret;
		}
		ret = gpio_pin_get(priv->detect_gpio, config->detect_pin);
		if (ret < 0) {
			return ret;
		}

		gpio_level = ret;

		if (gpio_level == 0) {
			priv->inserted = false;
			LOG_ERR("NO SD inserted!");

			return -ENODEV;
		}

		priv->inserted = true;
		LOG_INF("SD inserted!");
	}
	return 0;
}

static void usdhc_transfer_complete_cb(USDHC_Type *base,
					usdhc_handle_t *handle,
					status_t status,
					void *userData)
{
	struct usdhc_priv *priv = userData;

	if (status == kStatus_USDHC_TransferDataFailed ||
			status == kStatus_USDHC_SendCommandFailed) {
		priv->tx_status = -EIO;
		LOG_ERR("Transfer failed");
		k_sem_give(&priv->transfer_complete_sem);
	} else if (status == kStatus_USDHC_TransferDataComplete ||
			status == kStatus_USDHC_SendCommandSuccess) {
		priv->tx_status = 0;
		k_sem_give(&priv->transfer_complete_sem);
	}
}

static void usdhc_host_setup(struct usdhc_priv *priv)
{
	usdhc_config_t host_config;
	const struct usdhc_config *cfg = priv->config;
	usdhc_transfer_callback_t callback;

	/* Todo: remove, only needed for USDHC_Init() */
	host_config.dataTimeout = cfg->data_timeout;
	host_config.endianMode = cfg->endian;
	host_config.readWatermarkLevel = cfg->read_watermark;
	host_config.writeWatermarkLevel = cfg->write_watermark;
	host_config.readBurstLen = cfg->read_burst_len;
	host_config.writeBurstLen = cfg->write_burst_len;

	callback.TransferComplete = usdhc_transfer_complete_cb;
	USDHC_TransferCreateHandle(priv->config->base, &priv->host_handle,
								&callback, priv);
	k_sem_init(&priv->transfer_complete_sem, 0, 1);
}

static int usdhc_access_init(const struct device *dev)
{
	const struct usdhc_config *config = dev->config;
	struct usdhc_priv *priv = dev->data;
	int ret;

	(void)k_mutex_lock(&z_usdhc_init_lock, K_FOREVER);

	memset((char *)priv, 0, sizeof(struct usdhc_priv));
	priv->config = config;

	if (!config->base) {
		k_mutex_unlock(&z_usdhc_init_lock);

		return -ENODEV;
	}

	if (clock_control_get_rate(config->clock_dev,
				   config->clock_subsys,
				   &priv->src_clk_hz)) {
		return -EINVAL;
	}

	ret = usdhc_board_access_init(priv);
	if (ret) {
		k_mutex_unlock(&z_usdhc_init_lock);

		return ret;
	}

	priv->op_context.dma_cfg.dma_mode = USDHC_DMA_ADMA2;
	priv->op_context.dma_cfg.burst_len = USDHC_INCR_BURST_LEN;
	/*No DMA used for this Version*/
	priv->op_context.dma_cfg.adma_table = 0;
	priv->op_context.dma_cfg.adma_table_words = USDHC_ADMA_TABLE_WORDS;
	usdhc_host_hw_init(config->base, config);
	/* Set up host handle */
	usdhc_host_setup(priv);
	priv->host_ready = 1;

	usdhc_host_reset(priv);
	ret = usdhc_sd_init(priv);
	k_mutex_unlock(&z_usdhc_init_lock);

	return ret;
}

static int disk_usdhc_access_status(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct usdhc_priv *priv = dev->data;

	return priv->status;
}

static int disk_usdhc_access_read(struct disk_info *disk, uint8_t *buf,
				 uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct usdhc_priv *priv = dev->data;

	LOG_DBG("sector=%u count=%u", sector, count);

	return usdhc_read_sector(priv, buf, sector, count);
}

static int disk_usdhc_access_write(struct disk_info *disk, const uint8_t *buf,
				  uint32_t sector, uint32_t count)
{
	const struct device *dev = disk->dev;
	struct usdhc_priv *priv = dev->data;

	LOG_DBG("sector=%u count=%u", sector, count);

	return usdhc_write_sector(priv, buf, sector, count);
}

static int disk_usdhc_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buf)
{
	const struct device *dev = disk->dev;
	struct usdhc_priv *priv = dev->data;
	int err;

	err = sdhc_map_disk_status(priv->status);
	if (err != 0) {
		return err;
	}

	switch (cmd) {
	case DISK_IOCTL_CTRL_SYNC:
		break;
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)buf = priv->card_info.sd_block_count;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)buf = priv->card_info.sd_block_size;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)buf = priv->card_info.sd_block_size;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int disk_usdhc_access_init(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct usdhc_priv *priv = dev->data;

	if (priv->status == DISK_STATUS_OK) {
		/* Called twice, don't re-init. */
		return 0;
	}

	return usdhc_access_init(dev);
}

static const struct disk_operations usdhc_disk_ops = {
	.init = disk_usdhc_access_init,
	.status = disk_usdhc_access_status,
	.read = disk_usdhc_access_read,
	.write = disk_usdhc_access_write,
	.ioctl = disk_usdhc_access_ioctl,
};

static struct disk_info usdhc_disk = {
	.name = CONFIG_SDMMC_VOLUME_NAME,
	.ops = &usdhc_disk_ops,
};

static void usdhc_isr(const struct device *dev)
{
	struct usdhc_priv *priv = dev->data;

	USDHC_TransferHandleIRQ(priv->config->base, &priv->host_handle);
}

static int disk_usdhc_init(const struct device *dev)
{
	struct usdhc_priv *priv = dev->data;
	const struct usdhc_config *cfg = dev->config;

	priv->status = DISK_STATUS_UNINIT;

	cfg->irq_config_func(dev);

	usdhc_disk.dev = dev;

	return disk_access_register(&usdhc_disk);
}

#define DISK_ACCESS_USDHC_INIT_NONE(n)

#define DISK_ACCESS_USDHC_INIT_PWR_PROPS(n)				\
	.pwr_name = DT_INST_GPIO_LABEL(n, pwr_gpios),			\
	.pwr_pin = DT_INST_GPIO_PIN(n, pwr_gpios),			\
	.pwr_flags = DT_INST_GPIO_FLAGS(n, pwr_gpios),

#define DISK_ACCESS_USDHC_INIT_CD_PROPS(n)				\
	.detect_name = DT_INST_GPIO_LABEL(n, cd_gpios),			\
	.detect_pin = DT_INST_GPIO_PIN(n, cd_gpios),			\
	.detect_flags = DT_INST_GPIO_FLAGS(n, cd_gpios),

#define DISK_ACCESS_USDHC_INIT_PWR(n)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, pwr_gpios),		\
		    (DISK_ACCESS_USDHC_INIT_PWR_PROPS(n)),		\
		    (DISK_ACCESS_USDHC_INIT_NONE(n)))

#define DISK_ACCESS_USDHC_INIT_CD(n)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, cd_gpios),			\
		    (DISK_ACCESS_USDHC_INIT_CD_PROPS(n)),		\
		    (DISK_ACCESS_USDHC_INIT_NONE(n)))

#define DISK_ACCESS_USDHC_INIT(n)	\
	static void usdhc_##id##_config_func(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
					DT_INST_IRQ(n, priority),	\
					usdhc_isr,			\
					DEVICE_DT_INST_GET(n), 0);	\
		irq_enable(DT_INST_IRQN(n));				\
	}								\
	static const struct usdhc_config usdhc_config_##n = {		\
		.base = (USDHC_Type  *) DT_INST_REG_ADDR(n),		\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),	\
		.clock_subsys =						\
		(clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),	\
		.nusdhc = n,						\
		DISK_ACCESS_USDHC_INIT_PWR(n)				\
		DISK_ACCESS_USDHC_INIT_CD(n)				\
		.no_1_8_v = DT_INST_PROP(n, no_1_8_v),			\
		.data_timeout = USDHC_DATA_TIMEOUT,			\
		.endian = USDHC_LITTLE_ENDIAN,				\
		.read_watermark = USDHC_READ_WATERMARK_LEVEL,		\
		.write_watermark =  USDHC_WRITE_WATERMARK_LEVEL,	\
		.read_burst_len = USDHC_READ_BURST_LEN,			\
		.write_burst_len = USDHC_WRITE_BURST_LEN,		\
		.irq_config_func = usdhc_##id##_config_func,	\
	};								\
									\
	static struct usdhc_priv usdhc_priv_##n;			\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			    &disk_usdhc_init,				\
			    NULL,					\
			    &usdhc_priv_##n,				\
			    &usdhc_config_##n,				\
			    POST_KERNEL,				\
			    CONFIG_SDMMC_INIT_PRIORITY,			\
			    NULL);

DT_INST_FOREACH_STATUS_OKAY(DISK_ACCESS_USDHC_INIT)
