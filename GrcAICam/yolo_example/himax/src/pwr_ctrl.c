#include "pwr_ctrl.h"

static void set_pd_mode_cb()
{
	xprintf("!!!!!!!!!!!!! set_pd_mode_cb\n");
}

static void set_dpd_mode_cb()
{
	xprintf("!!!!!!!!!!!!! set_dpd_mode_cb\n");
}

static void set_pa01_alarm_cb(uint8_t group, uint8_t idx)
{
	xprintf("!!!!!!!!!!!!! set_pa01_alarm_cb\n");
}

// 0 - use PA0 for wakeup, 1 - use PA1 for wakeup

#ifndef HIMAX_TARGET_OLD_PCB // both HIMAX_PROTOTYPE and HIMAX_TARGE
#define WAKE_UP_PIN_PAn 0	// for prototype pin PA0 is used for wakeup
#else
#define WAKE_UP_PIN_PAn 1	// for prototype pin PA1 is used for wakeup
#endif


#if ((WAKE_UP_PIN_PAn != 0) && (WAKE_UP_PIN_PAn != 1))
	#error "Bad wakeup pin index"
#endif

static void setPA01AlarmPMU(bool DPD)
{
#if WAKE_UP_PIN_PAn == 0
	if (DPD) { // for Deep Power Down Mode (High Level is active)
		hx_drv_scu_set_PA0_pinmux(SCU_PA0_PINMUX_PMU_SINT0, 1);
	} else { // for Power Down Mode (GPIO_IRQ_TRIG_TYPE_..)
		hx_drv_scu_set_PA0_pinmux(SCU_PA0_PINMUX_AON_GPIO0_0, 1);
		hx_drv_gpio_set_input(AON_GPIO0);
		hx_drv_gpio_cb_register(AON_GPIO0, set_pa01_alarm_cb);
		hx_drv_gpio_set_int_type(AON_GPIO0, GPIO_IRQ_TRIG_TYPE_EDGE_RISING);
		hx_drv_gpio_set_int_enable(AON_GPIO0, 1);
	}
#elif WAKE_UP_PIN_PAn == 1
	if (DPD) { // for Deep Power Down Mode (High Level is active)
		hx_drv_scu_set_PA1_pinmux(SCU_PA1_PINMUX_AON_PMU_SINT1, 1);
	} else { // for Power Down Mode (GPIO_IRQ_TRIG_TYPE_..)
		hx_drv_scu_set_PA1_pinmux(SCU_PA1_PINMUX_AON_GPIO1, 1);
		hx_drv_gpio_set_input(AON_GPIO1);
		hx_drv_gpio_cb_register(AON_GPIO1, set_pa01_alarm_cb);
		hx_drv_gpio_set_int_type(AON_GPIO1, GPIO_IRQ_TRIG_TYPE_EDGE_RISING);
		hx_drv_gpio_set_int_enable(AON_GPIO1, 1);
	}
#endif
}

int pwr_ctrl_init()
{
    if (hx_lib_pm_init() != PM_NO_ERROR) {
        return -1;
    }
    return 0;
}

void pwr_dpd_enter()
{
	SCU_PLL_FREQ_E pmuwakeup_pll_freq;
	SCU_HSCCLKDIV_E pmuwakeup_cm55m_div;
	SCU_LSCCLKDIV_E pmuwakeup_cm55s_div;

// Get PMU Wake up bootrom cpu clk
	hx_drv_swreg_aon_get_pmuwakeup_freq(&pmuwakeup_pll_freq,	// bootrom cpu PMU wake up PLL frequency
	                                    &pmuwakeup_cm55m_div,	// bootrom cpu PMU wake up CM55M PLL Div
										&pmuwakeup_cm55s_div);	// bootrom cpu PMU wake up CM55S PLL Div

// Get PLL Frequency
	uint32_t freq;
	hx_drv_swreg_aon_get_pllfreq(&freq);

// Power manager configuration for DPD
	PM_DPD_CFG_T cfg;

// Configure the power manager library for specific power mode
	hx_lib_pm_get_defcfg_bymode(&cfg, PM_MODE_PS_DPD);
// Setup bootrom clock speed when PMU Warm boot wakeup
	cfg.bootromspeed.bootromclkfreq = SCU_PLL_FREQ_ENABLE;	// 
	cfg.bootromspeed.pll_freq = freq;				  		// bootrom PLL frequency
	cfg.bootromspeed.cm55m_div = pmuwakeup_cm55m_div; 		// bootrom CM55M Frequency Division
	cfg.bootromspeed.cm55s_div = pmuwakeup_cm55s_div; 		// bootrom CM55S Frequency Division
// CM55Reset when warm boot
	cfg.cm55s_reset = SWREG_AON_PMUWAKE_CM55S_RERESET_YES;
#if WAKE_UP_PIN_PAn == 0
	cfg.pmu_pad_pa0_mask = PM_IP_INT_MASK_ALL_UNMASK;		// PA0 is used for wakeup
	cfg.pmu_pad_pa1_mask = PM_IP_INT_MASK;
#else // WAKE_UP_PIN_PAn == 1
	cfg.pmu_pad_pa0_mask = PM_IP_INT_MASK;
	cfg.pmu_pad_pa1_mask = PM_IP_INT_MASK_ALL_UNMASK;		// PA1 is used for wakeup
#endif
	cfg.pmu_rtc_mask = PM_RTC_INT_MASK_ALLMASK;				// Mask RTC Interrupt for PMU
	cfg.pmu_anti_mask = PM_IP_INT_MASK;						// Mask ANTI TAMPER Interrupt for PMU
	cfg.support_debugdump = 0;								// No Debug Dump message

	setPA01AlarmPMU(1);

	// hx_drv_scu_set_dp_wakeupctrl_sel(SCU_DP_WAKEUPCTRL_SEL_PMU);
	// hx_drv_scu_set_PB11_pinmux(SCU_PB11_PINMUX_SRSTN, 1);
	hx_lib_pm_cfg_set(&cfg, set_dpd_mode_cb, PM_MODE_PS_DPD);
	xprintf("!!!!!!!!!!!!! dpd_mode\n");
	SCU_LSC_CLK_CFG_T lsc_cfg;
	SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;
	memset(&hsc_cfg, 0, sizeof(SCU_PDHSC_HSCCLK_CFG_T));
	memset(&lsc_cfg, 0, sizeof(SCU_LSC_CLK_CFG_T));
	hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYPMLIB);
	// hx_lib_pm_force_state_set(PM_MODE_PS_DPD);
}

void pwr_pd_enter()
{
	SCU_PLL_FREQ_E pmuwakeup_pll_freq;
	SCU_HSCCLKDIV_E pmuwakeup_cm55m_div;
	SCU_LSCCLKDIV_E pmuwakeup_cm55s_div;
	hx_drv_swreg_aon_get_pmuwakeup_freq(&pmuwakeup_pll_freq, &pmuwakeup_cm55m_div, &pmuwakeup_cm55s_div);

	uint32_t freq;
	hx_drv_swreg_aon_get_pllfreq(&freq);

	PM_PD_NOVIDPRE_CFG_T cfg;
	hx_lib_pm_get_defcfg_bymode(&cfg, PM_MODE_PS_NOVID_PREROLLING);

	cfg.bootromspeed.bootromclkfreq = SCU_PLL_FREQ_ENABLE;
	cfg.bootromspeed.pll_freq = freq;
	cfg.bootromspeed.cm55m_div = pmuwakeup_cm55m_div;
	cfg.bootromspeed.cm55s_div = pmuwakeup_cm55s_div;
	cfg.cm55s_reset = SWREG_AON_PMUWAKE_CM55S_RERESET_YES;	// Setup CM55 Small can be reset
	cfg.pmu_pad_pa01_mask = PM_IP_INT_MASK_ALL_UNMASK;		// UnMask PA01 Interrupt for PMU
	cfg.pmu_rtc_mask = PM_RTC_INT_MASK_ALLUNMASK;			// UnMask RTC Interrupt for PMU
	cfg.pmu_pad_pa23_mask = PM_IP_INT_MASK_ALL_UNMASK;		// UnMask PA23 Interrupt for PMU
	cfg.pmu_i2cw_mask = PM_IP_INT_MASK;						// Mask I2CWakeup Interrupt for PMU
	cfg.pmu_timer_mask = 0x1FE; 							// UnMask Timer0 Interrupt others timer interrupt are mask for PMU
	cfg.pmu_cmp_mask = PM_IP_INT_MASK;						// Mask CMP Interrupt for PMU
	cfg.pmu_ts_mask = PM_IP_INT_MASK;						// Mask TS Interrupt for PMU
	cfg.pmu_anti_mask = PM_IP_INT_MASK_ALL_UNMASK; 			// Mask ANTI TAMPER Interrupt for PMU
	cfg.support_debugdump = 0;								// No Debug Dump message
	cfg.tcm_retention = PM_MEM_RET_NO;						// CM55M TCM Retention
	cfg.hscsram_retention[0] = PM_MEM_RET_NO;				// HSC SRAM Retention
	cfg.hscsram_retention[1] = PM_MEM_RET_NO;				// HSC SRAM Retention
	cfg.hscsram_retention[2] = PM_MEM_RET_NO;				// HSC SRAM Retention
	cfg.hscsram_retention[3] = PM_MEM_RET_NO;				// HSC SRAM Retention
	cfg.lscsram_retention = PM_MEM_RET_NO;					// LSC SRAM Retention
	cfg.skip_bootflow.sec_mem_flag = SWREG_AON_NO_RETENTION;				// Skip Boot Flow
	cfg.skip_bootflow.first_bl_flag = SWREG_AON_NO_RETENTION; 				// First BL Retention
	cfg.skip_bootflow.cm55m_s_app_flag = SWREG_AON_NO_RETENTION;	 		// cm55m_s_app Retention
	cfg.skip_bootflow.cm55m_ns_app_flag = SWREG_AON_NO_RETENTION; 			// cm55m_ns_app Retention
	cfg.skip_bootflow.cm55s_s_app_flag = SWREG_AON_NO_RETENTION; 			// cm55s_s_app Retention
	cfg.skip_bootflow.cm55s_ns_app_flag = SWREG_AON_NO_RETENTION; 			// cm55s_ns_app Retention
	cfg.skip_bootflow.cm55m_model_flag = SWREG_AON_NO_RETENTION; 			// cm55m model Retention
	cfg.skip_bootflow.cm55s_model_flag = SWREG_AON_NO_RETENTION; 			// cm55s model Retention
	cfg.skip_bootflow.cm55m_appcfg_flag = SWREG_AON_NO_RETENTION; 			// cm55m appcfg Retention
	cfg.skip_bootflow.cm55s_appcfg_flag = SWREG_AON_NO_RETENTION; 			// cm55s appcfg Retention
	cfg.skip_bootflow.cm55m_s_app_rwdata_flag = SWREG_AON_NO_RETENTION;		// cm55m_s_app RW Data Retention
	cfg.skip_bootflow.cm55m_ns_app_rwdata_flag = SWREG_AON_NO_RETENTION;	// cm55m_ns_app RW Data Retention
	cfg.skip_bootflow.cm55s_s_app_rwdata_flag = SWREG_AON_NO_RETENTION;		// cm55s_s_app RW Data Retention
	cfg.skip_bootflow.cm55s_ns_app_rwdata_flag = SWREG_AON_NO_RETENTION;	// cm55s_ns_app RW Data Retention
	cfg.skip_bootflow.secure_debug_flag = SWREG_AON_NO_RETENTION;
	cfg.support_bootwithcap = PM_BOOTWITHCAP_NO;
	cfg.pmu_dcdc_outpin = PM_CFG_DCDC_MODE_OFF;								// Not DCDC pin output
	cfg.ioret = PM_CFG_PD_IORET_ON;											// No Pre-capture when boot up
	cfg.sensor_type = PM_SENSOR_TIMING_FVLDLVLD_CON;
	cfg.simo_pd_onoff = PM_SIMO_PD_ONOFF_ON;								// SIMO on in PD

	hx_drv_scu_set_dp_wakeupctrl_sel(SCU_DP_WAKEUPCTRL_SEL_PMU);
	hx_lib_pm_cfg_set(&cfg, set_pd_mode_cb, PM_MODE_PS_NOVID_PREROLLING);

	setPA01AlarmPMU(0);

/*Use PMU lib to control HSC_CLK and LSC_CLK so set thoes parameter to 0*/
	SCU_LSC_CLK_CFG_T lsc_cfg;
	SCU_PDHSC_HSCCLK_CFG_T hsc_cfg;
	memset(&hsc_cfg, 0, sizeof(SCU_PDHSC_HSCCLK_CFG_T));
	memset(&lsc_cfg, 0, sizeof(SCU_LSC_CLK_CFG_T));
/*Trigger to PMU mode*/
	hx_lib_pm_trigger(hsc_cfg, lsc_cfg, PM_CLK_PARA_CTRL_BYPMLIB);
}

static uint8_t PendST __attribute__((section(".bss.os")));

static void SysTickDisable()
{
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  	if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U) {
    	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    	PendST = 1U;
  	}
}

static void SysTickEnable()
{
	if (PendST != 0U) {
		PendST = 0U;
		SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
	}
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void pm_sleep()
{
	static bool first = 1;

	if (first) {
		first = 0;
#if WAKE_UP_PIN_PAn == 0
		hx_drv_scu_set_PA0_pinmux(SCU_PA0_PINMUX_AON_GPIO0_0, 1);
		hx_drv_gpio_set_input(AON_GPIO0);
		hx_drv_gpio_cb_register(AON_GPIO0, set_pa01_alarm_cb);
		hx_drv_gpio_set_int_type(AON_GPIO0, GPIO_IRQ_TRIG_TYPE_EDGE_RISING);
#else // WAKE_UP_PIN_PAn == 1
		hx_drv_scu_set_PA1_pinmux(SCU_PA1_PINMUX_AON_GPIO1, 1);
		hx_drv_gpio_set_input(AON_GPIO1);
		hx_drv_gpio_cb_register(AON_GPIO1, set_pa01_alarm_cb);
		hx_drv_gpio_set_int_type(AON_GPIO1, GPIO_IRQ_TRIG_TYPE_EDGE_RISING);
#endif
	}

	__disable_irq();

	SysTickDisable();

#if WAKE_UP_PIN_PAn == 0
	hx_drv_gpio_set_int_enable(AON_GPIO0, 1);
#else // WAKE_UP_PIN_PAn == 1
	hx_drv_gpio_set_int_enable(AON_GPIO1, 1);
#endif

    __WFI();

#if WAKE_UP_PIN_PAn == 0
	hx_drv_gpio_set_int_enable(AON_GPIO0, 0);
	hx_drv_gpio_clr_int_status(AON_GPIO0);
#else // WAKE_UP_PIN_PAn == 1
	hx_drv_gpio_set_int_enable(AON_GPIO1, 0);
	hx_drv_gpio_clr_int_status(AON_GPIO1);
#endif

	SysTickEnable();

    __enable_irq();
}

#ifdef HIMAX_PROTOTYPE
void pwr_modes_test()
{
	hx_drv_scu_set_SEN_D0_pinmux(SCU_SEN_D0_PINMUX_GPIO18); // user button
    hx_drv_gpio_set_input(GPIO18);
    bool dpd = 1;
    uint8_t key;
    for (int i = 0; i < 5; i++) {
        hx_drv_gpio_get_in_value(GPIO18, &key);
        xprintf(key ? "x" : "X");
        if (key == 1)
            dpd = false;
        board_delay_ms(1000);
    }
    xprintf("\n");
	if (dpd) {
		pwr_dpd_enter();
	}
	else {
		pwr_pd_enter();
	}
}
#endif // HIMAX_PROTOTYPE
