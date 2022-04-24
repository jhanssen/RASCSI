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

#define ALLWINNER_CPU_BASE  0x01C25000

struct allwinner_cpu_registers
{
	volatile uint8_t pad2[0x0E80]; // CPU registers start at 0xC00
	volatile uint32_t CNT64_CTRL_REG; // 0x0280 64-bit Counter Control Register
	volatile uint32_t CNT64_LOW_REG;  // 0x0284 64-bit Counter Low Register
	volatile uint32_t CNT64_HIGH_REG; // 0x0288 64-bit Counter High Register
};

#define ALLWINNER_TIMER_BASE 0x01C20000

void SysTimer::Init()
{

	printf("opening thing\n");
	int fd1 = open("/dev/mem", O_RDWR| O_SYNC);
	if (fd1 == -1) {
		LOGERROR("Error: Unable to open /dev/mem. Are you running as root?");
		return;
	}
	printf("opened /dev/mem\n");


	struct allwinner_cpu_registers *cpu = (struct allwinner_cpu_registers *)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd1, ALLWINNER_CPU_BASE);

	if(cpu == MAP_FAILED){
		int reason = errno;
		LOGERROR("Unable to map cpu timer (%d)", reason);
	}
	printf("cpu struct address %08X\n",(DWORD)cpu);
	printf("CNT64_CTRL_REG %08X\n", cpu->CNT64_CTRL_REG);
	printf("CNT64_CTRL_REG %08X\n", cpu->CNT64_CTRL_REG);
	printf("CNT64_CTRL_REG %08X\n", cpu->CNT64_CTRL_REG);
	printf("CNT64_CTRL_REG %08X\n", cpu->CNT64_CTRL_REG);
	// cpu->CNT64_CTRL_REG = 0x1;
	printf("CNT64_CTRL_REG %08X\n", cpu->CNT64_CTRL_REG);
	printf("\n");
	printf("CNT64 REG %08X:%08X\n", cpu->CNT64_HIGH_REG, cpu->CNT64_LOW_REG);
	for(volatile int i=0 ; i < 0xFF; i++){}
	printf("CNT64 REG %08X:%08X\n", cpu->CNT64_HIGH_REG, cpu->CNT64_LOW_REG);
	for(volatile int i=0 ; i < 0xFF; i++){}
	printf("CNT64 REG %08X:%08X\n", cpu->CNT64_HIGH_REG, cpu->CNT64_LOW_REG);
	for(volatile int i=0 ; i < 0xFF; i++){}
	printf("CNT64 REG %08X:%08X\n", cpu->CNT64_HIGH_REG, cpu->CNT64_LOW_REG);
	for(volatile int i=0 ; i < 0xFF; i++){}
	printf("CNT64 REG %08X:%08X\n", cpu->CNT64_HIGH_REG, cpu->CNT64_LOW_REG);
	for(volatile int i=0 ; i < 0xFF; i++){}
	printf("CNT64 REG %08X:%08X\n", cpu->CNT64_HIGH_REG, cpu->CNT64_LOW_REG);





	struct allwinner_timer *t = (struct allwinner_timer *)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd1, ALLWINNER_TIMER_BASE);
   	// hr_timer_map = (uint32_t *)mmap( NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, HR_TIMER_BASE_BP);

	if(t == MAP_FAILED){
		int reason = errno;
		LOGERROR("Unable to map t timer (%d)", reason);
	}
	printf("struct address %08X\n",(DWORD)t);
	// TODO: check for errors
	DWORD val = t->TMR_IRQ_EN_REG;
	printf("TMR_IRQ_EN_REG %08X\n", val);
	 val = t->TMR_IRQ_STA_REG;
	printf("TMR_IRQ_STA_REG %08X\n", val);
	 val = t->TMR0_CTRL_REG;
	printf("TMR0_CTRL_REG %08X (should be 4)\n", val);
	 val = t->TMR0_INTV_VALUE_REG;
	printf("TMR0_INTV_VALUE_REG %08X\n", val);
	 val = t->TMR0_CUR_VALUE_REG;
	printf("TMR0_CUR_VALUE_REG %08X\n", val);
	close(fd1);
	for(volatile int i=0 ; i < 0xFFFF; i++){}

	 val = t->TMR0_CUR_VALUE_REG;
	printf("TMR0_CUR_VALUE_REG %08X\n", val);
	for(volatile int i=0 ; i < 0xFFFF; i++){}
	
	 val = t->TMR0_CUR_VALUE_REG;
	printf("TMR0_CUR_VALUE_REG %08X\n", val);


	// t->IRQ_EN_REG = 0;

	// if(SBC_Version::IsRaspberryPi()){
	// 	DWORD baseaddr;						// Base address
	// 	void *map;

	// 	// Get the base address
	// 	// baseaddr = (DWORD)bcm_host_get_peripheral_address();
	// 	baseaddr = SBC_Version::GetPeripheralAddress();

	// 	// Open /dev/mem
	// 	int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	// 	if (mem_fd == -1) {
	// 		LOGERROR("Error: Unable to open /dev/mem. Are you running as root?");
	// 		return;
	// 	}

	// 	// Map peripheral region memory
	// 	map = mmap(NULL, 0x1000100, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, baseaddr);
	// 	if (map == MAP_FAILED) {
	// 		LOGERROR("Error: Unable to map memory %08X", baseaddr);
	// 		close(mem_fd);
	// 		return;
	// 	}

	// 	LOGDEBUG("SYST_OFFSET is %08X", (DWORD)map + SYST_OFFSET / sizeof(DWORD) )
	// 	LOGDEBUG("SYST_OFFSET is %08X", (DWORD)map + ARMT_OFFSET / sizeof(DWORD) )

	// 	SysTimer::Init(
	// 		(DWORD *)map + SYST_OFFSET / sizeof(DWORD),
	// 		(DWORD *)map + ARMT_OFFSET / sizeof(DWORD));

	// 	close(mem_fd);
	// }
	// else
	// {


    	// uint8_t *hr_timer_mem;



		// DWORD baseaddr;						// Base address
		// void *map;

		// // Get the base address
		// // baseaddr = (DWORD)bcm_host_get_peripheral_address();
		// baseaddr = SBC_Version::GetPeripheralAddress();

		// Open /dev/mem
		int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
		if (mem_fd == -1) {
			LOGERROR("Error: Unable to open /dev/mem. Are you running as root?");
			return;
		}
		printf("opened /dev/mem\n");

		printf("Page size: %ld\n", sysconf(_SC_PAGE_SIZE));
		// hr_timer_mem = (uint8_t*)malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL)

		// if ((uint32_t)hr_timer_mem % PAGE_SIZE)
		// 	hr_timer_mem += PAGE_SIZE - ((uint32_t)hr_timer_mem % PAGE_SIZE);
    
    	// hr_timer_map = (uint32_t *)mmap( NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, GPIO_BASE_BP);
		volatile struct allwinner_high_speed_timer *timer;
    	timer = (struct allwinner_high_speed_timer *)mmap( NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, HR_TIMER_BASE_BP);


printf("running the manual's procedure....\n");
// writel(0x0, HS_TMR_INTV_HI_REG); //Set interval value Hi 0x0
timer->HS_TMR_INTV_HI_REG = 0;
// writel(0x32, HS_TMR_INTV_LO_REG); //Set interval value Lo 0x32
timer->HS_TMR_INTV_LO_REG = 0x32;
// writel(0x90, HS_TMR_CTRL_REG); //Select n_mode,2 pre-scale,single mode
timer->HS_TMR_CTRL_REG = 0x90;
// writel(readl(HS_TMR_CTRL_REG)|(1<<1), HS_TMR_CTRL_REG); //Set Reload bit
timer->HS_TMR_CTRL_REG = (timer->HS_TMR_CTRL_REG) | (1<<1);
// writel(readl(HS_TMR_CTRL_REG)|(1<<0), HS_TMR_CTRL_REG); //Enable HSTimer
timer->HS_TMR_CTRL_REG = (timer->HS_TMR_CTRL_REG) | (1<<0);
// While(!(readl(HS_TMR_IRQ_STAS_REG)&1)); //Wait for HSTimer to generate pending
// while( !(timer->HS_TMR_CTRL_REG & 1)){
// 	printf(".");
// }
// Writel(1, HS_TMR_IRQ_STAS_REG); //Clear HSTimer pending 
timer->HS_TMR_IRQ_STAS_REG = 1;
printf("done\n");




		// map = mmap(NULL, 8192,
		// 	PROT_READ | PROT_WRITE, MAP_SHARED, fd, ARM_GICD_BASE);

		if(timer == MAP_FAILED){
			int reason = errno;
			LOGERROR("Unable to map hr timer (%d)", reason);
		}

		printf("done with map %08X\n", (uint32_t)timer);

	//	writel(HS_TMR0_CTRL_REG, 0);
	//	writel(HS_TMR0_INTV_HI_REG, 0xFFFF);
	//	writel(HS_TMR0_INTV_LO_REG, 0xFFFF);

    	// hr_timer_map = (uint32_t *)mmap( NULL, 8192, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0xFF841000);

		// if(hr_timer_map == MAP_FAILED){
		// 	int reason = errno;
		// 	LOGERROR("Second try failed too.... Unable to map hr timer (%d)", reason);
		// }

		// map = mmap(NULL, 8192,
		// 	PROT_READ | PROT_WRITE, MAP_SHARED, fd, ARM_GICD_BASE);

		// if(hr_timer_map == MAP_FAILED){
		// 	int reason = errno;
		// 	LOGERROR("Unable to map hr timer (%d)", reason);
		// }


		// // Map peripheral region memory
		// map = mmap(NULL, 0x1000100, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, hs_timer_base_addr);
		// if (map == MAP_FAILED) {
		// 	LOGERROR("Error: Unable to map memory %08X", hs_timer_base_addr);
		// 	close(mem_fd);
		// 	return;
		// }

		// LOGDEBUG("SYST_OFFSET is %08X", (DWORD)map + SYST_OFFSET / sizeof(DWORD) )
		// LOGDEBUG("ARMT_OFFSET is %08X", (DWORD)map + ARMT_OFFSET / sizeof(DWORD) )

		// m_hr_timer_ctrl_reg = (DWORD *)map + HS_TMR3_CTRL_REG / sizeof(DWORD);
		// m_hr_timer_curnt_lo_reg = (DWORD *)map + HS_TMR3_CURNT_LO_REG / sizeof(DWORD);
		// m_hr_timer_curnt_hi_reg = (DWORD *)map + HS_TMR3_CURNT_HI_REG / sizeof(DWORD);



	// // Save the base address
	// systaddr = syst;
	// armtaddr = armt;

	// // Change the ARM timer to free run mode
	// armtaddr[ARMT_CTRL] = 0x00000282;



		// SysTimer::Init(
		// 	(DWORD *)map + SYST_OFFSET / sizeof(DWORD),
		// 	(DWORD *)map + ARMT_OFFSET / sizeof(DWORD));

#define HS_TMR_CTRL_REG_EN (1 << 0)
#define HS_TMR_CTRL_REG_RELOAD (1 << 1)
#define HS_TMR_CTRL_REG_MODE (1 << 7)
#define HS_TMR_CTRL_REG_TEST (1 << 31)

	timer->HS_TMR_IRQ_EN_REG = 0x000F;
	timer->HS_TMR_INTV_LO_REG = 0xDEADBEAF;
	timer->HS_TMR_INTV_HI_REG = 0x0000FFFF;
	timer->HS_TMR_CTRL_REG = 0x90;
	val = timer->HS_TMR_CTRL_REG;
	timer->HS_TMR_CTRL_REG = (val | HS_TMR_CTRL_REG_RELOAD);
	val = timer->HS_TMR_CTRL_REG;
	timer->HS_TMR_CTRL_REG = (val | HS_TMR_CTRL_REG_EN);


	printf("-------------------\n");
	printf("HS_TMR_IRQ_EN_REG %08X\n", timer->HS_TMR_IRQ_EN_REG);
	printf("HS_TMR_IRQ_STAS_REG %08X\n", timer->HS_TMR_IRQ_STAS_REG);
	printf("HS_TMR_CTRL_REG %08X\n", timer->HS_TMR_CTRL_REG);
	printf("HS_TMR_INTV_LO_REG %08X\n", timer->HS_TMR_INTV_LO_REG);
	printf("HS_TMR_INTV_HI_REG %08X\n", timer->HS_TMR_INTV_HI_REG);
	printf("HS_TMR_CURNT_LO_REG %08X\n", timer->HS_TMR_CURNT_LO_REG);
	printf("HS_TMR_CURNT_HI_REG %08X\n", timer->HS_TMR_CURNT_HI_REG);
	printf("-------------------\n");
	for(volatile int i=0 ; i < 0xFFFF; i++){}
	printf("HS_TMR_CURNT_LO_REG %08X\n", timer->HS_TMR_CURNT_LO_REG);
	printf("HS_TMR_CURNT_HI_REG %08X\n", timer->HS_TMR_CURNT_HI_REG);
	printf("-------------------\n");


	while(!(timer->HS_TMR_IRQ_STAS_REG & 0x1) && (timer->HS_TMR_CURNT_LO_REG != 0)){
	for(volatile int i=0 ; i < 0xFFFF; i++){}
		printf("HS_TMR_CURNT_LO_REG %08X\n", timer->HS_TMR_CURNT_LO_REG);
		printf("HS_TMR_CURNT_HI_REG %08X\n", timer->HS_TMR_CURNT_HI_REG);
	}; //Wait for HSTimer to generate pending

	timer->HS_TMR_IRQ_STAS_REG = 1;
	// Writel(1, HS_TMR_IRQ_STAS_REG); //Clear HSTimer pending


		close(mem_fd);

// 	for (int i=0; i<30; i++){
// 		DWORD val = *(hr_timer_map + (i<<2));
// 		printf("%08X ", val);
// 	}

// printf("HS_TMR0_INTV_HI_REG\n");
// writel(0x32, HS_TMR0_INTV_HI_REG); //Set interval value Hi 0x0
// printf("New val is %08X\n", readl(HS_TMR0_INTV_HI_REG));
// printf("HS_TMR0_INTV_LO_REG\n");
// writel(0x32, HS_TMR0_INTV_LO_REG); //Set interval value Lo 0x32
// printf("New val is %08X\n", readl(HS_TMR0_INTV_LO_REG));
// printf("HS_TMR0_CTRL_REG\n");
// writel(0x90, HS_TMR0_CTRL_REG); //Select n_mode,2 pre-scale,single mode
// printf("HS_TMR0_CTRL_REG 1<<1\n");
// writel(readl(HS_TMR0_CTRL_REG)|(1<<1), HS_TMR0_CTRL_REG); //Set Reload bit
// printf("HS_TMR0_CTRL_REG 1<<0\n");
// writel(readl(HS_TMR0_CTRL_REG)|(1<<0), HS_TMR0_CTRL_REG); //Enable HSTimer
// // while(!(readl(HS_TMR0_IRQ_STAT_REG)&1)); //Wait for HSTimer to generate pending
// // writel(1,HS_TMR0_IRQ_STAT_REG); //Clear HSTimer pending

// 	printf("\n----------------------\n");
// 	for (int i=0; i<30; i++){
// 		DWORD val = *(hr_timer_map + (i<<2));
// 		printf("%08X ", val);
// 	}

// 	usleep(1000);
// 	printf("\n----------------------\n");
// 	for (int i=0; i<30; i++){
// 		DWORD val = *(hr_timer_map + (i<<2));
// 		printf("%08X ", val);
// 	}


// 	printf("\n\nbase data %08X\n", *hr_timer_map);

// 	DWORD new_ctrl_value = 0x1;
// 	printf("checking value...\n");
// 	printf("Previous ctrl value = %08X\n", readl(HS_TMR0_CTRL_REG));
// 	writel(new_ctrl_value, HS_TMR0_CTRL_REG);
// 	printf("New ctrl value = %08X\n", readl(HS_TMR0_CTRL_REG));
	




	// }
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
