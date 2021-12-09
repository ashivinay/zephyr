/*
 * Copyright (c) 2021 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <device.h>
#include <pm/pm.h>
#include <fsl_dcdc.h>
#include <fsl_pmu.h>
#include <fsl_gpc.h>
#include <fsl_lpuart.h>

#include <logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

static void lpm_set_sleep_mode_config(clock_mode_t mode)
{
	uint32_t clpcr;

    /*
     * ERR050143: CCM: When improper low-power sequence is used,
     * the SoC enters low power mode before the ARM core executes WFI.
     *
     * Software workaround:
     * 1) Software should trigger IRQ #41 (GPR_IRQ) to be always pending
     *      by setting IOMUXC_GPR_GPR1_GINT.
     * 2) Software should then unmask IRQ #41 in GPC before setting CCM
     *      Low-Power mode.
     * 3) Software should mask IRQ #41 right after CCM Low-Power mode
     *      is set (set bits 0-1 of CCM_CLPCR).
     */
	GPC_EnableIRQ(GPC, GPR_IRQ_IRQn);
	clpcr      = CCM->CLPCR & (~(CCM_CLPCR_LPM_MASK | CCM_CLPCR_ARM_CLK_DIS_ON_LPM_MASK));
	/* Note: if CCM_CLPCR_ARM_CLK_DIS_ON_LPM_MASK is set,
	 * debugger will not connect in sleep mode
	 */
	/* Set clock control module to transfer system to idle mode */
	CCM->CLPCR = clpcr | CCM_CLPCR_LPM(mode) | CCM_CLPCR_MASK_SCU_IDLE_MASK |
					CCM_CLPCR_MASK_L2CC_IDLE_MASK |
					CCM_CLPCR_STBY_COUNT_MASK | CCM_CLPCR_BYPASS_LPM_HS0_MASK |
					CCM_CLPCR_BYPASS_LPM_HS1_MASK;
	GPC_DisableIRQ(GPC, GPR_IRQ_IRQn);
}

static void lpm_enter_sleep_mode(clock_mode_t mode)
{
	/* FIXME: When this function is entered the Kernel has disabled
	 * interrupts using BASEPRI register. This is incorrect as it prevents
	 * waking up from any interrupt which priority is not 0. Work around the
	 * issue and disable interrupts using PRIMASK register as recommended
	 * by ARM.
	 */

	/* Set PRIMASK */
	__disable_irq();
	/* Set BASEPRI to 0 */
	irq_unlock(0);

	if (mode == kCLOCK_ModeWait) {
		/* Clear the SLEEPDEEP bit to go into sleep mode (WAIT) */
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	} else {
		/* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	}
	/* WFI instruction will start entry into WAIT/STOP mode */
	__WFI();

	/* Clear PRIMASK after wakeup*/
	__enable_irq();
}

void lpm_set_run_mode_config(void)
{
	CCM->CLPCR &= ~(CCM_CLPCR_LPM_MASK | CCM_CLPCR_ARM_CLK_DIS_ON_LPM_MASK);
}

/* Sets device into low power mode */
__weak void pm_power_state_set(struct pm_state_info info)
{
	clock_mode_t clk_mode;

	switch (info.state) {
	case PM_STATE_RUNTIME_IDLE:
		clk_mode = kCLOCK_ModeWait;
		break;
	case PM_STATE_SUSPEND_TO_IDLE:
		clk_mode = kCLOCK_ModeWait;
		break;
	case PM_STATE_SOFT_OFF:
		clk_mode = kCLOCK_ModeStop;
	default:
		LOG_DBG("Unsupported power state: %u", info.state);
		return;
	}
	lpm_set_sleep_mode_config(clk_mode);
	LOG_DBG("SOC entering sleep mode %s",
		clk_mode == kCLOCK_ModeWait ? "WAIT" : "STOP");
	lpm_enter_sleep_mode(clk_mode);
	/* Set run mode config after wakeup */
	lpm_set_run_mode_config();
}

/* Handle SOC specific activity after Low Power Mode Exit */
__weak void pm_power_state_exit_post_ops(struct pm_state_info info)
{
	ARG_UNUSED(info);

	LOG_DBG("SOC woke from LPM");
}

/* Initialize power system */
static int rt10xx_power_init(const struct device *dev)
{
	dcdc_internal_regulator_config_t reg_config;

	ARG_UNUSED(dev);

	/* Ensure clocks to ARM core memory will not be gated in low power mode
	 * if interrupt is pending
	 */
	CCM->CGPR |= CCM_CGPR_INT_MEM_CLK_LPM_MASK;

	/* Errata ERR050143 */
	IOMUXC_GPR->GPR1 |= IOMUXC_GPR_GPR1_GINT_MASK;

	/* Configure DCDC */
	DCDC_BootIntoDCM(DCDC);
	/* Set target voltage for low power mode to 0.925V*/
	DCDC_AdjustLowPowerTargetVoltage(DCDC, 0x1);
	/* Reconfigure DCDC to disable internal load resistor */
	reg_config.enableLoadResistor = false;
	reg_config.feedbackPoint = 0x1; /* 1.0V with 1.3V reference voltage */
	DCDC_SetInternalRegulatorConfig(DCDC, &reg_config);

	/* Enable high gate drive on power FETs to reduce leakage current */
	PMU_CoreEnableIncreaseGateDrive(PMU, true);

	/* Set GPC wakeup config to GPT timer interrupt */
	GPC_EnableIRQ(GPC, DT_IRQN(DT_INST(0, nxp_gpt_hw_timer)));
	return 0;
}

SYS_INIT(rt10xx_power_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
