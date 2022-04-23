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


		DWORD *SysTimer::m_hr_timer_ctrl_reg;
		DWORD *SysTimer::m_hr_timer_curnt_lo_reg;
		DWORD *SysTimer::m_hr_timer_curnt_hi_reg;

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
   uint32_t mmap_base = (addr & ~MAP_MASK);
   uint32_t mmap_seek = ((addr - mmap_base) >> 2);
   val = *(hr_timer_map + mmap_seek);
   
 		printf("mmap_base = 0x%x\t mmap_seek = 0x%x\t gpio_map = 0x%x\t total = 0x%x\n",(unsigned int)mmap_base,(unsigned int)mmap_seek,(unsigned int)hr_timer_map,(unsigned int)(hr_timer_map + mmap_seek));
		
   return val;
}

void SysTimer::writel(uint32_t val, uint32_t addr)
{
  uint32_t mmap_base = (addr & ~MAP_MASK);
  uint32_t mmap_seek = ((addr - mmap_base) >> 2);
  *(hr_timer_map + mmap_seek) = val;
}




//---------------------------------------------------------------------------
//
//	Core frequency
//
//---------------------------------------------------------------------------
volatile DWORD SysTimer::corefreq;

void SysTimer::Init()
{
	if(SBC_Version::IsRaspberryPi()){
		DWORD baseaddr;						// Base address
		void *map;

		// Get the base address
		// baseaddr = (DWORD)bcm_host_get_peripheral_address();
		baseaddr = SBC_Version::GetPeripheralAddress();

		// Open /dev/mem
		int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
		if (mem_fd == -1) {
			LOGERROR("Error: Unable to open /dev/mem. Are you running as root?");
			return;
		}

		// Map peripheral region memory
		map = mmap(NULL, 0x1000100, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, baseaddr);
		if (map == MAP_FAILED) {
			LOGERROR("Error: Unable to map memory %08X", baseaddr);
			close(mem_fd);
			return;
		}

		LOGDEBUG("SYST_OFFSET is %08X", (DWORD)map + SYST_OFFSET / sizeof(DWORD) )
		LOGDEBUG("SYST_OFFSET is %08X", (DWORD)map + ARMT_OFFSET / sizeof(DWORD) )

		SysTimer::Init(
			(DWORD *)map + SYST_OFFSET / sizeof(DWORD),
			(DWORD *)map + ARMT_OFFSET / sizeof(DWORD));

		close(mem_fd);
	}
	else
	{


    	// uint8_t *hr_timer_mem;



		// DWORD baseaddr;						// Base address
		void *map;

		// // Get the base address
		// // baseaddr = (DWORD)bcm_host_get_peripheral_address();
		// baseaddr = SBC_Version::GetPeripheralAddress();

		// Open /dev/mem
		int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
		if (mem_fd == -1) {
			LOGERROR("Error: Unable to open /dev/mem. Are you running as root?");
			return;
		}
		// hr_timer_mem = (uint8_t*)malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL)

		// if ((uint32_t)hr_timer_mem % PAGE_SIZE)
		// 	hr_timer_mem += PAGE_SIZE - ((uint32_t)hr_timer_mem % PAGE_SIZE);
    
    	hr_timer_map = (uint32_t *)mmap( NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, HR_TIMER_BASE_BP);


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

		close(mem_fd);




	DWORD new_ctrl_value = 0x1;
	printf("Previous ctrl value = %08X", readl(HS_TMR0_CTRL_REG));
	writel(new_ctrl_value, HS_TMR0_CTRL_REG);
	printf("New ctrl value = %08X", readl(HS_TMR0_CTRL_REG));
	




	}
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
