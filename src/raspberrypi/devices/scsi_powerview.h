//---------------------------------------------------------------------------
//
//	SCSI Target Emulator RaSCSI (*^..^*)
//	for Raspberry Pi
//
//  Copyright (C) 2020-2021 akuker
//  Copyright (C) 2020 joshua stein <jcs@jcs.org>
//
//  Licensed under the BSD 3-Clause License. 
//  See LICENSE file in the project root folder.
//
//  [ Emulation of the Radius PowerView SCSI Display Adapter ]
//
//  Note: This requires the Radius RadiusWare driver.
//
//  Framebuffer integration originally done by Joshua Stein:
//     https://github.com/jcs/RASCSI/commit/6da9e9f3ffcd38eb89413cd445f7407739c54bca
//
//---------------------------------------------------------------------------
#pragma once

#include "os.h"
#include "disk.h"
#include <map>
#include <string>
#include "../rascsi.h"


//===========================================================================
//
//	Radius PowerView
//
//===========================================================================
class SCSIPowerView: public Disk
{

private:
	typedef struct _command_t {
		const char* name;
		void (SCSIPowerView::*execute)(SASIDEV *);

		_command_t(const char* _name, void (SCSIPowerView::*_execute)(SASIDEV *)) : name(_name), execute(_execute) { };
	} command_t;
	std::map<SCSIDEV::scsi_command, command_t*> commands;

	SASIDEV::ctrl_t *ctrl;

	void AddCommand(SCSIDEV::scsi_command, const char*, void (SCSIPowerView::*)(SASIDEV *));

public:
	SCSIPowerView();
	~SCSIPowerView();

	bool Init(const map<string, string>&) override;
	void Open(const Filepath& path) override;

	// // Commands
	int Inquiry(const DWORD *cdb, BYTE *buffer) override;
	int Read(const DWORD *cdb, BYTE *buf, uint64_t block) override;
	bool Write(const DWORD *cdb, const BYTE *buf, DWORD block) override;
	// int WriteCheck(DWORD block) override;	// WRITE check

	// int RetrieveStats(const DWORD *cdb, BYTE *buffer);
	// bool EnableInterface(const DWORD *cdb);

	// void SetMacAddr(const DWORD *cdb, BYTE *buffer);	// Set MAC address

	// void TestUnitReady(SASIDEV *) override;
	// void Read6(SASIDEV *) override;
	// void Write6(SASIDEV *) override;
	// void RetrieveStatistics(SASIDEV *);
	// void SetInterfaceMode(SASIDEV *);
	// void SetMcastAddr(SASIDEV *);
	// void EnableInterface(SASIDEV *);

	bool Dispatch(SCSIDEV *) override;

    bool ReceiveBuffer(int len, BYTE *buffer);
	// const int DAYNAPORT_BUFFER_SIZE = 0x1000000;

	// static const BYTE CMD_SCSILINK_STATS        = 0x09;
	// static const BYTE CMD_SCSILINK_ENABLE       = 0x0E;
	// static const BYTE CMD_SCSILINK_SET          = 0x0C;
	// static const BYTE CMD_SCSILINK_SETMODE      = 0x80;
	// static const BYTE CMD_SCSILINK_SETMAC       = 0x40;

	// // When we're reading the Linux tap device, most of the messages will not be for us, so we
	// // need to filter through those. However, we don't want to keep re-reading the packets
	// // indefinitely. So, we'll pick a large-ish number that will cause the emulated DaynaPort
	// // to respond with "no data" after MAX_READ_RETRIES tries.
	// static const int MAX_READ_RETRIES               = 50;

	// // The READ response has a header which consists of:
	// //   2 bytes - payload size
	// //   4 bytes - status flags
	// static const DWORD DAYNAPORT_READ_HEADER_SZ = 2 + 4;

private:

    int screen_width;
	int screen_height;

	int fbfd;
	char *fb;
	int fbwidth;
	int fbheight;
	int fblinelen;
	int fbsize;
	int fbbpp;
	// typedef struct __attribute__((packed)) {
	// 	BYTE operation_code;
	// 	BYTE misc_cdb_information;
	// 	BYTE logical_block_address;
	// 	uint16_t length;
	// 	BYTE format;
	// } scsi_cmd_daynaport_write_t;

	// enum read_data_flags_t : uint32_t {
	// 	e_no_more_data = 0x00000000,
	// 	e_more_data_available = 0x00000001,
	// 	e_dropped_packets = 0xFFFFFFFF,
	// };

	// typedef struct __attribute__((packed)) {
	// 	uint32_t length;
	// 	read_data_flags_t flags;
	// 	BYTE pad;
	// 	BYTE data[ETH_FRAME_LEN + sizeof(uint32_t)]; // Frame length + 4 byte CRC
	// } scsi_resp_read_t;

	// typedef struct __attribute__((packed)) {
	// 	BYTE mac_address[6];
	// 	uint32_t frame_alignment_errors;
	// 	uint32_t crc_errors;
	// 	uint32_t frames_lost;
	// } scsi_resp_link_stats_t;

	// scsi_resp_link_stats_t m_scsi_link_stats = {
	// 	.mac_address = { 0x00, 0x80, 0x19, 0x10, 0x98, 0xE3 },//MAC address of @PotatoFi's DayanPort
	// 	.frame_alignment_errors = 0,
	// 	.crc_errors = 0,
	// 	.frames_lost = 0,
	// };

	// const BYTE m_daynacom_mac_prefix[3] = { 0x00, 0x80, 0x19 };

	// CTapDriver *m_tap;
	// 									// TAP driver
	// bool m_bTapEnable;
	// 									// TAP valid flag
	// BYTE m_mac_addr[6];
	// 									// MAC Address
	// static const BYTE m_bcast_addr[6];
	// static const BYTE m_apple_talk_addr[6];
};
