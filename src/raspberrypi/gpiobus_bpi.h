//---------------------------------------------------------------------------
//
//	SCSI Target Emulator RaSCSI (*^..^*)
//	for Raspberry Pi
//
//	Powered by XM6 TypeG Technology.
//	Copyright (C) 2016-2020 GIMONS
//	[ GPIO-SCSI bus ]
//
//---------------------------------------------------------------------------

#if !defined(gpiobus_bpi_h)
#define gpiobus_bpi_h

#include "config.h"
#include "scsi.h"
#include "gpiobus.h"
//---------------------------------------------------------------------------
//
//	Class definition
//
//---------------------------------------------------------------------------
class GPIOBUS_BPI : public BUS
{
public:
	// Basic Functions
	GPIOBUS_BPI();
										// Constructor
	virtual ~GPIOBUS_BPI();
										// Destructor
	BOOL Init(mode_e mode = TARGET);
										// Initialization
	void Reset();
										// Reset
	void Cleanup();
										// Cleanup

	//---------------------------------------------------------------------------
	//
	//	Bus signal acquisition
	//
	//---------------------------------------------------------------------------
	inline DWORD Aquire() override
	{
	#if defined(__x86_64__) || defined(__X86__)
		// Only used for development/debugging purposes. Isn't really applicable
		// to any real-world RaSCSI application
		return 0;
	#else
		signals = *level;

	#if SIGNAL_CONTROL_MODE < 2
		// Invert if negative logic (internal processing is unified to positive logic)
		signals = ~signals;
	#endif	// SIGNAL_CONTROL_MODE

		return signals;
	#endif // ifdef __x86_64__ || __X86__
	}

	void SetENB(BOOL ast);
										// Set ENB signal

	bool GetBSY() override;
										// Get BSY signal
	void SetBSY(bool ast) override;
										// Set BSY signal

	BOOL GetSEL() override;
										// Get SEL signal
	void SetSEL(BOOL ast) override;
										// Set SEL signal

	BOOL GetATN() override;
										// Get ATN signal
	void SetATN(BOOL ast) override;
										// Set ATN signal

	BOOL GetACK() override;
										// Get ACK signal
	void SetACK(BOOL ast) override;
										// Set ACK signal

	BOOL GetACT();
										// Get ACT signal
	void SetACT(BOOL ast);
										// Set ACT signal

	BOOL GetRST() override;
										// Get RST signal
	void SetRST(BOOL ast) override;
										// Set RST signal

	BOOL GetMSG() override;
										// Get MSG signal
	void SetMSG(BOOL ast) override;
										// Set MSG signal

	BOOL GetCD() override;
										// Get CD signal
	void SetCD(BOOL ast) override;
										// Set CD signal

	BOOL GetIO() override;
										// Get IO signal
	void SetIO(BOOL ast) override;
										// Set IO signal

	BOOL GetREQ() override;
										// Get REQ signal
	void SetREQ(BOOL ast) override;
										// Set REQ signal

	BYTE GetDAT() override;
										// Get DAT signal
	void SetDAT(BYTE dat) override;
										// Set DAT signal
	BOOL GetDP() override;
										// Get Data parity signal
	int CommandHandShake(BYTE *buf, bool) override;
										// Command receive handshake
	int ReceiveHandShake(BYTE *buf, int count) override;
										// Data receive handshake
	int SendHandShake(BYTE *buf, int count, int delay_after_bytes) override;
										// Data transmission handshake

	static BUS::phase_t GetPhaseRaw(DWORD raw_data);
										// Get the phase based on raw data

	static int GetCommandByteCount(BYTE opcode);

	#ifdef USE_SEL_EVENT_ENABLE
	// SEL signal interrupt
	int PollSelectEvent();
										// SEL signal event polling
	void ClearSelectEvent();
										// Clear SEL signal event
#endif	// USE_SEL_EVENT_ENABLE

private:

	BOOL setup_raspberry_pi(void* map, int fd);
	BOOL setup_banana_pi(void* map, int fd);

	// SCSI I/O signal control
	void MakeTable();
										// Create work data
	void SetControl(int pin, BOOL ast);
										// Set Control Signal
	void SetMode(int pin, int mode);
										// Set SCSI I/O mode
	BOOL GetSignal(int pin);
										// Get SCSI input signal value
	void SetSignal(int pin, BOOL ast);
										// Set SCSI output signal value
	BOOL WaitSignal(int pin, BOOL ast);
										// Wait for a signal to change
	// Interrupt control
	void DisableIRQ();
										// IRQ Disabled
	void EnableIRQ();
										// IRQ Enabled

	//  GPIO pin functionality settings
	void PinConfig(int pin, int mode);
										// GPIO pin direction setting
	void PullConfig(int pin, int mode);
										// GPIO pin pull up/down resistor setting
	void PinSetSignal(int pin, BOOL ast);
										// Set GPIO output signal
	void DrvConfig(DWORD drive);
										// Set GPIO drive strength


	mode_e actmode;						// Operation mode

	DWORD baseaddr;						// Base address

	int rpitype;						// Type of Raspberry Pi

	volatile DWORD *gpio;				// GPIO register

	volatile DWORD *pads;				// PADS register

	volatile DWORD *level;				// GPIO input level

	volatile DWORD *irpctl;				// Interrupt control register

	volatile DWORD irptenb;				// Interrupt enabled state

	volatile DWORD *qa7regs;			// QA7 register

	volatile int tintcore;				// Interupt control target CPU.

	volatile DWORD tintctl;				// Interupt control

	volatile DWORD giccpmr;				// GICC priority setting

	volatile DWORD *gicd;				// GIC Interrupt distributor register

	volatile DWORD *gicc;				// GIC CPU interface register

	DWORD gpfsel[4];					// GPFSEL0-4 backup values

	DWORD signals;						// All bus signals

#ifdef USE_SEL_EVENT_ENABLE
	struct gpioevent_request selevreq = {};	// SEL signal event request

	int epfd;							// epoll file descriptor
#endif	// USE_SEL_EVENT_ENABLE

#if SIGNAL_CONTROL_MODE == 0
	DWORD tblDatMsk[3][256];			// Data mask table

	DWORD tblDatSet[3][256];			// Data setting table
#else
	DWORD tblDatMsk[256];				// Data mask table

	DWORD tblDatSet[256];				// Table setting table
#endif

	static const int SignalTable[19];	// signal table
};

#endif	// gpiobus_h
