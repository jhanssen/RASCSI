//---------------------------------------------------------------------------
//
//	SCSI Target Emulator RaSCSI (*^..^*)
//	for Raspberry Pi
//
//	Powered by XM6 TypeG Technology.
//	Copyright (C) 2016-2020 GIMONS
//  Copyright (C) 2022 akuker
//
//	[ Low level system timer ]
//
//---------------------------------------------------------------------------

#pragma once

#include "config.h"
#include "scsi.h"


#define SYST_CS			0
#define SYST_CLO		1
#define SYST_CHI		2
#define SYST_C0			3
#define SYST_C1			4
#define SYST_C2			5
#define SYST_C3			6
#define ARMT_LOAD		0
#define ARMT_VALUE		1
#define ARMT_CTRL		2
#define ARMT_CLRIRQ		3
#define ARMT_RAWIRQ		4
#define ARMT_MSKIRQ		5
#define ARMT_RELOAD		6
#define ARMT_PREDIV		7
#define ARMT_FREERUN	8

#define SYST_OFFSET		0x00003000
#define ARMT_OFFSET		0x0000B400

//===========================================================================
//
//	System timer
//
//===========================================================================
class SysTimer
{
public:
	static void Init(DWORD *syst, DWORD *armt);
	static void Init();
										// Initialization
	static DWORD GetTimerLow();
										// Get system timer low byte
	static DWORD GetTimerHigh();
										// Get system timer high byte
	static void SleepNsec(DWORD nsec);
										// Sleep for N nanoseconds
	static void SleepUsec(DWORD usec);
										// Sleep for N microseconds

private:
	static volatile DWORD *systaddr;
										// System timer address
	static volatile DWORD *armtaddr;
										// ARM timer address
	static volatile DWORD corefreq;
										// Core frequency
	static bool m_use_linux_timers;

	static volatile uint32_t *hr_timer_map;

		static const DWORD hs_timer_base_addr = 0x01C60000;
		static const DWORD HS_TMR3_CTRL_REG = 0x0070; // HS Timer 3 Control Register
		static const DWORD HS_TMR3_INTV_LO_REG = 0x0074; // HS Timer 3 Interval Value Low Register
		static const DWORD HS_TMR3_INTV_HI_REG = 0x0078; // HS Timer 3 Interval Value High Register
		static const DWORD HS_TMR3_CURNT_LO_REG = 0x007C; // HS Timer 3 Current Value Low Register
		static const DWORD HS_TMR3_CURNT_HI_REG = 0x0080; // HS Timer 3 Current Value High Register


		static const DWORD HS_TMR_IRQ_EN_REG =0x0000; //HS Timer IRQ Enable Register
		static const DWORD HS_TMR_IRQ_STAS_REG =0x0004; //HS Timer Status Register
		static const DWORD HS_TMR0_CTRL_REG =0x0010; //HS Timer 0 Control Register
		static const DWORD HS_TMR0_INTV_LO_REG =0x0014; //HS Timer 0 Interval Value Low Register
		static const DWORD HS_TMR0_INTV_HI_REG =0x0018; //HS Timer 0 Interval Value High Register
		static const DWORD HS_TMR0_CURNT_LO_REG = 0x001C; //HS Timer 0 Current Value Low Register
		static const DWORD HS_TMR0_CURNT_HI_REG = 0x0020; //HS Timer 0 Current Value High Register



		static DWORD *m_hr_timer_ctrl_reg;
		static DWORD *m_hr_timer_curnt_lo_reg;
		static DWORD *m_hr_timer_curnt_hi_reg;


		static uint32_t readl(uint32_t addr);
		static void writel(uint32_t val, uint32_t addr);
		static const DWORD MAP_SIZE	= (4096*2);
		static const DWORD MAP_MASK	= (MAP_SIZE - 1);
		static const DWORD PAGE_SIZE = (4*1024);
		static const DWORD BLOCK_SIZE = (4*1024);
		static const DWORD GPIO_BASE_BP = (0x01C20000);
		static const DWORD HR_TIMER_BASE_BP = (0x01C60000);

};
