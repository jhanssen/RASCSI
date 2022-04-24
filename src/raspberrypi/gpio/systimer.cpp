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

#include "systimer.h"
#include "sbc_version.h"
#include "log.h"
#include <sys/mman.h>
#include <unistd.h>


		DWORD *SysTimer::m_hr_timer_ctrl_reg;
		DWORD *SysTimer::m_hr_timer_curnt_lo_reg;
		DWORD *SysTimer::m_hr_timer_curnt_hi_reg;
		volatile uint32_t *SysTimer::hr_timer_map;

//---------------------------------------------------------------------------
//
//	System timer address
//
//---------------------------------------------------------------------------
volatile DWORD* SysTimer::systaddr;

//---------------------------------------------------------------------------
//
//	ARM timer address
//
//---------------------------------------------------------------------------
volatile DWORD* SysTimer::armtaddr;

 uint32_t SysTimer::readl(uint32_t addr)
{
   uint32_t val = 0;
//    uint32_t mmap_base = (addr & ~MAP_MASK);
//    uint32_t mmap_seek = ((addr - mmap_base) >> 2);
   val = *(hr_timer_map + (addr));
   
 		// printf("mmap_base = 0x%x\t mmap_seek = 0x%x\t gpio_map = 0x%x\t total = 0x%x\n",(unsigned int)mmap_base,(unsigned int)mmap_seek,(unsigned int)hr_timer_map,(unsigned int)(hr_timer_map + mmap_seek));
		
   return val;
}

void SysTimer::writel(uint32_t val, uint32_t addr)
{
//   uint32_t mmap_base = (addr & ~MAP_MASK);
//   uint32_t mmap_seek = ((addr - mmap_base) >> 2);
  *(hr_timer_map + (addr)) = val;
}




//---------------------------------------------------------------------------
//
//	Core frequency
//
//---------------------------------------------------------------------------
volatile DWORD SysTimer::corefreq;

struct allwinner_timer
{
	volatile uint8_t pad[0xC00];
	volatile uint32_t TMR_IRQ_EN_REG; //0x0000 Timer IRQ Enable Register
	volatile uint32_t TMR_IRQ_STA_REG; // 0x0004 Timer Status Register
	volatile uint32_t PAD_08;
	volatile uint32_t PAD_0C;
	volatile uint32_t TMR0_CTRL_REG; // 0x0010 Timer 0 Control Register
	volatile uint32_t TMR0_INTV_VALUE_REG; // 0x0014 Timer 0 Interval Value Register
	volatile uint32_t TMR0_CUR_VALUE_REG; // 0x0018 Timer 0 Current Value Register
	volatile uint32_t TMR1_CTRL_REG; // 0x0020 Timer 1 Control Register
	volatile uint32_t TMR1_INTV_VALUE_REG; // 0x0024 Timer 1 Interval Value Register
	volatile uint32_t TMR1_CUR_VALUE_REG; // 
};

struct allwinner_high_speed_timer
{
	volatile uint32_t HS_TMR_IRQ_EN_REG; // 0x00 HS Timer IRQ Enable Register
	volatile uint32_t HS_TMR_IRQ_STAS_REG; // 0x04 HS Timer Status Register
	volatile uint32_t PAD_08;
	volatile uint32_t PAD_0C;
	volatile uint32_t HS_TMR_CTRL_REG; // 0x10 HS Timer Control Register
	volatile uint32_t HS_TMR_INTV_LO_REG; // 0x14 HS Timer Interval Value Low Register
	volatile uint32_t HS_TMR_INTV_HI_REG; // 0x18 HS Timer Interval Value High Register
	volatile uint32_t HS_TMR_CURNT_LO_REG; // 0x1C HS Timer Current Value Low Register
	volatile uint32_t HS_TMR_CURNT_HI_REG; // 0x20 HS Timer Current Value High Register
};

// #define ALLWINNER_CPU_BASE  0x01C25000

// struct allwinner_cpu_registers
// {
// 	volatile uint8_t pad2[0x0E80]; // CPU registers start at 0xC00
// 	volatile uint32_t CNT64_CTRL_REG; // 0x0280 64-bit Counter Control Register
// 	volatile uint32_t CNT64_LOW_REG;  // 0x0284 64-bit Counter Low Register
// 	volatile uint32_t CNT64_HIGH_REG; // 0x0288 64-bit Counter High Register
// };

#define ALLWINNER_TIMER_BASE    0x01C20000
#define ALLWINNER_CCU_BASE      0x01C20000
#define ALLWINNER_HS_TIMER_BASE 0x01C60000



struct allwinner_ccu_registers //0x01C2_0000
{
	volatile uint8_t pad[0x60];
	volatile uint32_t BUS_CLK_GATING_REG0; // 0x0060 Bus Clock Gating Register 0
	volatile uint32_t BUS_CLK_GATING_REG1; // 0x0064 Bus Clock Gating Register 1
	volatile uint32_t BUS_CLK_GATING_REG2; // 0x0068 Bus Clock Gating Register 2
	volatile uint32_t BUS_CLK_GATING_REG3; // 0x006C Bus Clock Gating Register 3
	volatile uint8_t pad2[0x2C0 - 0x6C];
	volatile uint32_t BUS_SOFT_RST_REG0;   // 0x02C0 Bus Software Reset Register 0
	volatile uint32_t BUS_SOFT_RST_REG1;   // 0x02C4 Bus Software Reset Register 1
	volatile uint32_t BUS_SOFT_RST_REG2;   // 0x02C8 Bus Software Reset Register 2
	volatile uint32_t BUS_SOFT_RST_REG3;   // 0x02D0 Bus Software Reset Register 3
	volatile uint32_t BUS_SOFT_RST_REG4;   // 0x02D8 Bus Software Reset Register 4
	volatile uint8_t pad3[0x00010000 - 0x02D8];
	volatile uint8_t pad_01C3_0000[0x00010000];
	volatile uint8_t pad_01C4_0000[0x00010000];
	volatile uint8_t pad_01C5_0000[0x00010000];
	volatile uint32_t HS_TMR_IRQ_EN_REG; // 0x00 HS Timer IRQ Enable Register
	volatile uint32_t HS_TMR_IRQ_STAS_REG; // 0x04 HS Timer Status Register
	volatile uint32_t PAD_08;
	volatile uint32_t PAD_0C;
	volatile uint32_t HS_TMR_CTRL_REG; // 0x10 HS Timer Control Register
	volatile uint32_t HS_TMR_INTV_LO_REG; // 0x14 HS Timer Interval Value Low Register
	volatile uint32_t HS_TMR_INTV_HI_REG; // 0x18 HS Timer Interval Value High Register
	volatile uint32_t HS_TMR_CURNT_LO_REG; // 0x1C HS Timer Current Value Low Register
	volatile uint32_t HS_TMR_CURNT_HI_REG; // 0x20 HS Timer Current Value High Register
};


#define BUS_SOFT_RST_REG0_HSTIMR_RST  (1<<19)
#define BUS_CLK_GATING_REG0_HSTMR_GATING  (1<<19)


void SysTimer::Init()
{

	printf("opening /dev/mem\n");
	int fd1 = open("/dev/mem", O_RDWR| O_SYNC);
	if (fd1 == -1) {
		LOGERROR("Error: Unable to open /dev/mem. Are you running as root?");
		return;
	}
	printf("DONE! opened /dev/mem\n");

	// volatile struct allwinner_ccu_registers *ccu = (struct allwinner_ccu_registers *)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd1, ALLWINNER_CCU_BASE);
	volatile struct allwinner_ccu_registers *ccu = (struct allwinner_ccu_registers *)mmap(NULL, 0x60000, PROT_READ | PROT_WRITE, MAP_SHARED, fd1, ALLWINNER_CCU_BASE);
	if(ccu == MAP_FAILED){
		int reason = errno;
		LOGERROR("Unable to map ccu timer (%d)", reason);
	}
	printf("CCU struct address %08X\n",(DWORD)ccu);
	printf("BUS_CLK_GATING_REG0 %08X\n", ccu->BUS_CLK_GATING_REG0);
	printf("BUS_CLK_GATING_REG1 %08X\n", ccu->BUS_CLK_GATING_REG1);
	printf("BUS_CLK_GATING_REG2 %08X\n", ccu->BUS_CLK_GATING_REG2);
	printf("BUS_CLK_GATING_REG3 %08X\n", ccu->BUS_CLK_GATING_REG3);
	printf("BUS_SOFT_RST_REG0 %08X\n", ccu->BUS_SOFT_RST_REG0);
	printf("BUS_SOFT_RST_REG1 %08X\n", ccu->BUS_SOFT_RST_REG1);
	printf("BUS_SOFT_RST_REG2 %08X\n", ccu->BUS_SOFT_RST_REG2);
	printf("BUS_SOFT_RST_REG3 %08X\n", ccu->BUS_SOFT_RST_REG3);
	printf("BUS_SOFT_RST_REG4 %08X\n", ccu->BUS_SOFT_RST_REG4);

	// Enable the HS Timer and clear its reset
	// ccu->BUS_CLK_GATING_REG0 = (ccu->BUS_CLK_GATING_REG0 | BUS_CLK_GATING_REG0_HSTMR_GATING);
	uint32_t d = ccu->BUS_CLK_GATING_REG0;
	d = d | BUS_CLK_GATING_REG0_HSTMR_GATING;
	ccu->BUS_CLK_GATING_REG0 = d;
	printf("  New val %08X added %08X\n", d, BUS_CLK_GATING_REG0_HSTMR_GATING);
	// ccu->BUS_SOFT_RST_REG0 = (ccu->BUS_SOFT_RST_REG0 | BUS_SOFT_RST_REG0_HSTIMR_RST);
	d = ccu->BUS_SOFT_RST_REG0;
	d = d | BUS_SOFT_RST_REG0_HSTIMR_RST;
	ccu->BUS_SOFT_RST_REG0 = d;
	printf("  New val %08X added %08X\n", d, BUS_SOFT_RST_REG0_HSTIMR_RST);
	printf("Timer should be enabled...\n");
	printf("BUS_CLK_GATING_REG0 %08X\n", ccu->BUS_CLK_GATING_REG0);
	printf("BUS_SOFT_RST_REG0 %08X\n", ccu->BUS_SOFT_RST_REG0);


	// volatile struct allwinner_high_speed_timer *timer = (struct allwinner_high_speed_timer *)mmap( NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd1, HR_TIMER_BASE_BP);

	// struct allwinner_high_speed_timer *timer  = (struct allwinner_high_speed_timer *)(ccu + (ALLWINNER_HS_TIMER_BASE - ALLWINNER_CCU_BASE));
	// printf("done with map %08X\n", (uint32_t)timer);
	// if(timer == MAP_FAILED){
	// 	int reason = errno;
	// 	LOGERROR("Unable to map hr timer (%d)", reason); 
	// }

	printf("running the manual's procedure....\n");
	// writel(0x0, HS_TMR_INTV_HI_REG); //Set interval value Hi 0x0
	ccu->HS_TMR_INTV_HI_REG = 0;
	// writel(0x32, HS_TMR_INTV_LO_REG); //Set interval value Lo 0x32
	ccu->HS_TMR_INTV_LO_REG = 0x32;
	// writel(0x90, HS_TMR_CTRL_REG); //Select n_mode,2 pre-scale,single mode
	ccu->HS_TMR_CTRL_REG = 0x90;
	// writel(readl(HS_TMR_CTRL_REG)|(1<<1), HS_TMR_CTRL_REG); //Set Reload bit
	ccu->HS_TMR_CTRL_REG = (ccu->HS_TMR_CTRL_REG) | (1<<1);
	// writel(readl(HS_TMR_CTRL_REG)|(1<<0), HS_TMR_CTRL_REG); //Enable HSTimer
	ccu->HS_TMR_CTRL_REG = (ccu->HS_TMR_CTRL_REG) | (1<<0);
	// While(!(readl(HS_TMR_IRQ_STAS_REG)&1)); //Wait for HSTimer to generate pending
	// while( !(timer->HS_TMR_CTRL_REG & 1)){
	// 	printf(".");
	// }
	// Writel(1, HS_TMR_IRQ_STAS_REG); //Clear HSTimer pending 
	ccu->HS_TMR_IRQ_STAS_REG = 1;

	printf("-------------------\n");
	printf("HS_TMR_IRQ_EN_REG %08X\n", ccu->HS_TMR_IRQ_EN_REG);
	printf("HS_TMR_IRQ_STAS_REG %08X\n", ccu->HS_TMR_IRQ_STAS_REG);
	printf("HS_TMR_CTRL_REG %08X\n", ccu->HS_TMR_CTRL_REG);
	printf("HS_TMR_INTV_LO_REG %08X\n", ccu->HS_TMR_INTV_LO_REG);
	printf("HS_TMR_INTV_HI_REG %08X\n", ccu->HS_TMR_INTV_HI_REG);
	printf("HS_TMR_CURNT_LO_REG %08X\n", ccu->HS_TMR_CURNT_LO_REG);
	printf("HS_TMR_CURNT_HI_REG %08X\n", ccu->HS_TMR_CURNT_HI_REG);
	printf("done\n");


	#define HS_TMR_CTRL_REG_EN (1 << 0)
	#define HS_TMR_CTRL_REG_RELOAD (1 << 1)
	#define HS_TMR_CTRL_REG_MODE (1 << 7)
	#define HS_TMR_CTRL_REG_TEST (1 << 31)

	ccu->HS_TMR_IRQ_EN_REG = 0x000F;
	ccu->HS_TMR_INTV_LO_REG = 0xDEADBEAF;
	ccu->HS_TMR_INTV_HI_REG = 0x0000FFFF;
	ccu->HS_TMR_CTRL_REG = 0x90;
	uint32_t val = ccu->HS_TMR_CTRL_REG;
	ccu->HS_TMR_CTRL_REG = (val | HS_TMR_CTRL_REG_RELOAD);
	val = ccu->HS_TMR_CTRL_REG;
	ccu->HS_TMR_CTRL_REG = (val | HS_TMR_CTRL_REG_EN);


	printf("-------------------\n");
	printf("HS_TMR_IRQ_EN_REG %08X\n", ccu->HS_TMR_IRQ_EN_REG);
	printf("HS_TMR_IRQ_STAS_REG %08X\n", ccu->HS_TMR_IRQ_STAS_REG);
	printf("HS_TMR_CTRL_REG %08X\n", ccu->HS_TMR_CTRL_REG);
	printf("HS_TMR_INTV_LO_REG %08X\n", ccu->HS_TMR_INTV_LO_REG);
	printf("HS_TMR_INTV_HI_REG %08X\n", ccu->HS_TMR_INTV_HI_REG);
	printf("HS_TMR_CURNT_LO_REG %08X\n", ccu->HS_TMR_CURNT_LO_REG);
	printf("HS_TMR_CURNT_HI_REG %08X\n", ccu->HS_TMR_CURNT_HI_REG);
	printf("-------------------\n");
	for(volatile int i=0 ; i < 0xFFFF; i++){}
	printf("HS_TMR_CURNT_LO_REG %08X\n", ccu->HS_TMR_CURNT_LO_REG);
	printf("HS_TMR_CURNT_HI_REG %08X\n", ccu->HS_TMR_CURNT_HI_REG);
	printf("-------------------\n");

	while(!(ccu->HS_TMR_IRQ_STAS_REG & 0x1) && (ccu->HS_TMR_CURNT_LO_REG != 0)){
	for(volatile int i=0 ; i < 0xFFFF; i++){}
		printf("HS_TMR_CURNT_LO_REG %08X\n", ccu->HS_TMR_CURNT_LO_REG);
		printf("HS_TMR_CURNT_HI_REG %08X\n", ccu->HS_TMR_CURNT_HI_REG);
	}; //Wait for HSTimer to generate pending

	ccu->HS_TMR_IRQ_STAS_REG = 1;
	// Writel(1, HS_TMR_IRQ_STAS_REG); //Clear HSTimer pending

		close(fd1);
}

//---------------------------------------------------------------------------
//
//	Initialize the system timer
//
//---------------------------------------------------------------------------
void SysTimer::Init(DWORD *syst, DWORD *armt)
{
	// if(m_use_linux_timers)
	// // RPI Mailbox property interface
	// // Get max clock rate
	// //  Tag: 0x00030004
	// //
	// //  Request: Length: 4
	// //   Value: u32: clock id
	// //  Response: Length: 8
	// //   Value: u32: clock id, u32: rate (in Hz)
	// //
	// // Clock id
	// //  0x000000004: CORE
	// DWORD maxclock[32] = { 32, 0, 0x00030004, 8, 0, 4, 0, 0 };

	// // Save the base address
	// systaddr = syst;
	// armtaddr = armt;

	// // Change the ARM timer to free run mode
	// armtaddr[ARMT_CTRL] = 0x00000282;

	// // Get the core frequency
	// corefreq = 0;
	// int fd = open("/dev/vcio", O_RDONLY);
	// if (fd >= 0) {
	// 	int res = ioctl(fd, _IOWR(100, 0, char *), maxclock);
	// 	if(res == 0){
	// 		corefreq = maxclock[6] / 1000000;
	// 	}
	// 	else
	// 	{
	// 		LOGERROR("%s vcio ioctl failed", __PRETTY_FUNCTION__);
	// 	}
	// }
	// else
	// {
	// 	LOGERROR("%s Unable to open /dev/vcio", __PRETTY_FUNCTION__);
	// }
	// LOGINFO("core frequency %u", corefreq);
	// close(fd);
}

//---------------------------------------------------------------------------
//
//	Get system timer low byte
//
//---------------------------------------------------------------------------
DWORD SysTimer::GetTimerLow() {
	return readl(HS_TMR0_CURNT_LO_REG);
	// return systaddr[SYST_CLO];
}

//---------------------------------------------------------------------------
//
//	Get system timer high byte
//
//---------------------------------------------------------------------------
DWORD SysTimer::GetTimerHigh() {
	return readl(HS_TMR0_CURNT_HI_REG);
	// return *m_hr_timer_curnt_hi_reg;
	// return systaddr[SYST_CHI];
}

//---------------------------------------------------------------------------
//
//	Sleep in nanoseconds
//
//---------------------------------------------------------------------------
void SysTimer::SleepNsec(DWORD nsec)
{
	// If time is 0, don't do anything
	if (nsec == 0) {
		return;
	}

	// Calculate the timer difference
	DWORD diff = corefreq * nsec / 1000;

	// Return if the difference in time is too small
	if (diff == 0) {
		return;
	}

	// Start
	DWORD start = armtaddr[ARMT_FREERUN];

	// Loop until timer has elapsed
	while ((armtaddr[ARMT_FREERUN] - start) < diff);
}

//---------------------------------------------------------------------------
//
//	Sleep in microseconds
//
//---------------------------------------------------------------------------
void SysTimer::SleepUsec(DWORD usec)
{
	// If time is 0, don't do anything
	if (usec == 0) {
		return;
	}

	DWORD now = GetTimerLow();
	while ((GetTimerLow() - now) < usec);
}
