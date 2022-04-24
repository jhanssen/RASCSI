//---------------------------------------------------------------------------
//
//	SCSI Target Emulator RaSCSI (*^..^*)
//	for Raspberry Pi
//
//	Powered by XM6 TypeG Technology.
//	Copyright (C) 2016-2020 GIMONS
//
//---------------------------------------------------------------------------

#include "os.h"
#include "log.h"
#include "gpiobus.h"
#include "rascsi_version.h"
#include "spdlog/spdlog.h"
#include <sys/time.h>
#include <climits>
#include <sstream>
#include <iostream>
#include <getopt.h>
#include <unistd.h>
#include <sched.h>
#include "monitor/sm_reports.h"
#include "monitor/data_sample.h"

// #include "board/bpi-gpio.h"
#include "gpio/systimer.h"

#include "bpi_c_gpio.h"
#include "bpi_common.h"

using namespace std;

//---------------------------------------------------------------------------
//
//  Constant declarations
//
//---------------------------------------------------------------------------

// Symbol definition for the VCD file
// These are just arbitrary symbols. They can be anything allowed by the VCD file format,
// as long as they're consistently used.
#define SYMBOL_PIN_DAT '#'
#define SYMBOL_PIN_ATN '+'
#define SYMBOL_PIN_RST '$'
#define SYMBOL_PIN_ACK '%'
#define SYMBOL_PIN_REQ '^'
#define SYMBOL_PIN_MSG '&'
#define SYMBOL_PIN_CD '*'
#define SYMBOL_PIN_IO '('
#define SYMBOL_PIN_BSY ')'
#define SYMBOL_PIN_SEL '-'
#define SYMBOL_PIN_PHASE '='

//---------------------------------------------------------------------------
//
//	Variable declarations
//
//---------------------------------------------------------------------------
static volatile bool running; // Running flag
GPIOBUS *bus;                 // GPIO Bus

DWORD buff_size = 10;
data_capture *data_buffer;
DWORD data_idx = 0;

double ns_per_loop;

bool print_help = false;
bool import_data = false;

double diff_timespec(const struct timespec *time1, const struct timespec *time0) {
  return (time1->tv_sec - time0->tv_sec) * 1000000000.0
      + (time1->tv_nsec - time0->tv_nsec) ;
}

void test_system_delay_sleepusec(){
    struct timespec start_time, stop_time;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    SysTimer::SleepNsec(100);
    clock_gettime(CLOCK_MONOTONIC, &stop_time);

    LOGINFO("%ld %ld Time differnce = %f", start_time.tv_nsec, stop_time.tv_nsec, diff_timespec(&stop_time, &start_time));
}


void test_system_delay()
{
        // struct timespec start_time, stop_time;
	// System timer
    LOGINFO("SysTimer::Init()");
	SysTimer::Init();
    exit(0);

 test_system_delay_sleepusec();

//     LOGINFO("SysTimer::SleepNsec()");
//     SysTimer::SleepNsec(100);
//     // SysTimer::SleepUsec(100);

    #define COUNT 100
    DWORD times_low[COUNT];
//    DWORD times_high[COUNT];
    for(int i=0 ; i<COUNT; i++){
        times_low[i] = SysTimer::GetTimerLow();
  //      times_high[i] = SysTimer::GetTimerHigh();
        usleep(100);
    }

    printf("Times:  ");
    for(int i=0; i<COUNT; i++){
        printf("%f  ", (times_low[i]/(100.0*1000.0)));
    }
    printf("\n\n\n");




}




#define RPI_DISC_COUNT (40)
//---------------------------------------------------------------------------
//
//	Initialization
//
//---------------------------------------------------------------------------
bool Init()
{
    // // Interrupt handler settings
    // if (signal(SIGINT, KillHandler) == SIG_ERR)
    // {
    //     return FALSE;
    // }
    // if (signal(SIGHUP, KillHandler) == SIG_ERR)
    // {
    //     return FALSE;
    // }
    // if (signal(SIGTERM, KillHandler) == SIG_ERR)
    // {
    //     return FALSE;
    // }

    // /*************************************
    // printf("Running setup...\n");
    // bpi_c_gpio::setup();

    // for (unsigned int i = 0; i < ARRAY_SIZE(pinTobcm_BPI_M2U); i++)
    // {
    //     int disc = pinTobcm_BPI_M2U[i];
    //     if (disc >= 0)
    //     {
    //         printf("++++Setup pin %d as gpio %d [%X]\n", i, disc, disc);
    //         bpi_c_gpio::setup_gpio(disc, BPI_OUTPUT, BPI_PUD_UP);
    //     }
    // }
    // printf("done\n\n");
    // */
 

// pinTobcm_BPI_M2U

//     int data_pins[18] = {
//         PIN_DT0,PIN_DT1,PIN_DT2,PIN_DT3,
//         PIN_DT4,PIN_DT5,PIN_DT6,PIN_DT7,
//         PIN_DP,	PIN_ATN,PIN_RST,PIN_ACK,
//         PIN_REQ,PIN_MSG,PIN_CD,	PIN_IO,	
//         PIN_BSY,PIN_SEL,
//     };

// while(1){
//     printf("\nData Direction OUT\n");
//     bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[PIN_IND], !IND_IN);
//     bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[PIN_TAD], !TAD_IN);
//     bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[PIN_DTD], !PIN_DTD);

//     printf("\tSet all outputs high\n");
//     for(unsigned int pin_idx = 0; pin_idx<ARRAY_SIZE(data_pins); pin_idx++){
//         bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[data_pins[pin_idx]], BPI_HIGH);
//     }
//     sleep(2);

//     printf("\tSet all outputs low\n");
//     for(unsigned int pin_idx = 0; pin_idx<ARRAY_SIZE(data_pins); pin_idx++){
//         bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[data_pins[pin_idx]], BPI_LOW);
//     }
//     sleep(2);



//     printf("\n\nData Direction IN\n");
//     bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[PIN_IND], IND_IN);
//     bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[PIN_TAD], TAD_IN);
//     bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[PIN_DTD], DTD_IN);

//     printf("\tSet all outputs high\n");
//     for(unsigned int pin_idx = 0; pin_idx<ARRAY_SIZE(data_pins); pin_idx++){
//         bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[data_pins[pin_idx]], BPI_HIGH);
//     }
//     sleep(2);

//     printf("\tSet all outputs low\n");
//     for(unsigned int pin_idx = 0; pin_idx<ARRAY_SIZE(data_pins); pin_idx++){
//         bpi_c_gpio::output_gpio(pinTobcm_BPI_M2U[data_pins[pin_idx]], BPI_LOW);
//     }
//     sleep(2);
// }

//     exit(0);

//     for (unsigned int test_disc = 0; test_disc < RPI_DISC_COUNT; test_disc++)
//     {
//         if (physToGpio_BPI_M2U[test_disc] < 0)
//         {
//             continue;
//         }

//         bpi_c_gpio::output_gpio(physToGpio_BPI_M2U[test_disc], BPI_LOW);
//         for (unsigned int x = 0; x < buff_size; x++)
//         {

//             for (unsigned int i = 0; i < RPI_DISC_COUNT; i++)
//             {
//                 if (i == test_disc)
//                 {
//                     printf("!!!!!!!!!!! SKIPPING %d\n",test_disc);
//                     continue;
//                 }
//                 int disc =physToGpio_BPI_M2U[i];
//                 if (disc >= 0)
//                 {
//                     printf("*********Output pin %d as gpio %d [%X]\n", i, disc, disc);
//                     bpi_c_gpio::output_gpio(disc, BPI_LOW);
//                 }
//             }

//             usleep(500000);

//             for (unsigned int i = 0; i < RPI_DISC_COUNT; i++)
//             {
//                 if (i == test_disc)
//                 {
//                     printf("!!!!!!!!!!! SKIPPING %d\n",test_disc);
//                     continue;
//                 }
//                 int disc = physToGpio_BPI_M2U[i];
//                 if (disc >= 0)
//                 {
//                     printf("'''''''''''''Setup pin %d as gpio %d [%X]\n", i, disc, disc);
//                     bpi_c_gpio::output_gpio(disc, BPI_HIGH);
//                 }
//             }
//             usleep(500000);

//         }
//     }
//     exit(0);

    // // GPIO Initialization
    // bus = new GPIOBUS();
    // if (!bus->Init())
    // {
    //     LOGERROR("Unable to intiailize the GPIO bus. Exiting....");
    //     return false;
    // }

    // // Bus Reset
    // bus->Reset();

    // Other
    running = false;

    return true;
}

void Cleanup()
{
    if (!import_data)
    {
        LOGINFO("Stopping data collection....");
    }
    // LOGINFO(" ");
    // LOGINFO("Generating %s...", vcd_file_name);
    // scsimon_generate_value_change_dump(vcd_file_name, data_buffer, data_idx);
    // LOGINFO("Generating %s...", json_file_name);
    // scsimon_generate_json(json_file_name, data_buffer, data_idx);
    // LOGINFO("Generating %s...", html_file_name);
    // scsimon_generate_html(html_file_name, data_buffer, data_idx);

    if (bus)
    {
        // Cleanup the Bus
        bus->Cleanup();
        delete bus;
    }
}

void Reset()
{
    // Reset the bus
    bus->Reset();
}


//---------------------------------------------------------------------------
//
//	Main processing
//
//---------------------------------------------------------------------------
int main(int argc, char *argv[])
{

// #ifdef DEBUG
    spdlog::set_level(spdlog::level::trace);
// #else
//     spdlog::set_level(spdlog::level::info);
// #endif
    spdlog::set_pattern("%^[%l]%$ %v");


    test_system_delay();


    // Initialize
    int ret = 0;
    if (!Init())
    {
        ret = EPERM;
        goto init_exit;
    }


    Cleanup();

init_exit:
    exit(ret);
}
