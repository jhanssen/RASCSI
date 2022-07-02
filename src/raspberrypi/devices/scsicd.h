//---------------------------------------------------------------------------
//
//	SCSI Target Emulator RaSCSI (*^..^*)
//	for Raspberry Pi
//
//	Copyright (C) 2001-2006 ＰＩ．(ytanaka@ipc-tokai.or.jp)
//	Copyright (C) 2014-2020 GIMONS
//	Copyright (C) akuker
//
//	Licensed under the BSD 3-Clause License. 
//	See LICENSE file in the project root folder.
//
//	[ SCSI CD-ROM for Apple Macintosh ]
//
//---------------------------------------------------------------------------
#pragma once

#include "os.h"
#include "disk.h"
#include "filepath.h"
#include "interfaces/scsi_mmc_commands.h"
#include "interfaces/scsi_primary_commands.h"
#include <alsa/asoundlib.h>
#include <atomic>
#include <tuple>

//---------------------------------------------------------------------------
//
//	Class precedence definition
//
//---------------------------------------------------------------------------
class SCSICD;

//===========================================================================
//
//	CD-ROM Track
//
//===========================================================================
class CDTrack
{
public:
	CDTrack(SCSICD *scsicd);
	virtual ~CDTrack();

    void Init(int track, DWORD pre, DWORD first, DWORD last, DWORD startByte, off_t offset, bool rawtrack);

	// Properties
	void SetPath(bool cdda, const Filepath& path);			// Set the path
	void GetPath(Filepath& path) const;				// Get the path
	void AddIndex(int index, DWORD lba);				// Add index
    bool GetRaw() const;
    DWORD GetStartByte() const;
    DWORD GetPre() const;
	DWORD GetFirst() const;					// Get the start LBA
	DWORD GetLast() const;						// Get the last LBA
	DWORD GetBlocks() const;					// Get the number of blocks
	int GetTrackNo() const;					// Get the track number
	bool IsValid(DWORD lba) const;					// Is this a valid LBA?
	bool IsAudio() const;						// Is this an audio track?
    off_t GetOffset() const;

private:
	SCSICD *cdrom;								// Parent device
	bool valid;								// Valid track
	int track_no;								// Track number
    DWORD pre_lba;
	DWORD first_lba;							// First LBA
	DWORD last_lba;								// Last LBA
    DWORD start_byte;
    off_t offset_byte;
	bool audio;								// Audio track flag
	bool raw;								// RAW data flag
	Filepath imgpath;							// Image file path
};


class CDAudioPlayer
{
public:
    enum { SectorSize = 2352 };

    CDAudioPlayer();
    ~CDAudioPlayer();

    void setWritePipe(int wp);

    bool play(DWORD start, DWORD end, std::function<int32_t(uint8_t*, uint32_t)>&& read);
    bool pause();
    bool resume();
    void stop();

    void finish();

    DWORD playingSector() const;
    bool isPaused() const;
    bool isPlaying() const;
    bool isFinished() const;

private:
    static void readCallback(snd_async_handler_t* callback);

private:
    class RingBuffer
    {
    public:
        RingBuffer(uint32_t size = 0) { if (size > 0) resize(size); }

        using ReadAddresses = std::tuple<const uint8_t*, uint32_t, const uint8_t*, uint32_t>;
        using WriteAddresses = std::tuple<uint8_t*, uint32_t, uint8_t*, uint32_t>;

        void resize(uint32_t size);

        uint32_t read(uint8_t* out, uint32_t size);
        uint32_t write(const uint8_t* in, uint32_t size);

        WriteAddresses writeAddresses(uint32_t size);
        void commitWrite(uint32_t size);

        ReadAddresses readAddresses(uint32_t size);
        void commitRead(uint32_t size);

        uint32_t writeAvailable() const;
        uint32_t readAvailable() const;

    private:
        std::vector<uint8_t> data { };
        uint32_t readStart { 0 }, writeStart { 0 };
    };

private:
    DWORD startSector { 0 }, endSector { 0 };
    std::function<int32_t(uint8_t*, uint32_t)> readFunction;
    snd_pcm_t* pcmHandle { nullptr };
    snd_async_handler_t* pcmCallback { nullptr };
    snd_pcm_uframes_t bufferSize { 0 }, periodSize { 0 }, periodSize4 { 0 };
    int writePipe { -1 };

    enum { RingTotal = 100, RingHalf = 50 };
    RingBuffer ringBuffer;

    std::vector<uint8_t> sectorData;
    std::vector<uint8_t> periodData;
    uint32_t audioSector { 0 };
    bool paused { false };
    bool finished { false };
    std::atomic<bool> inCallback { false };
};

//===========================================================================
//
//	SCSI CD-ROM
//
//===========================================================================
class SCSICD : public Disk, public ScsiMmcCommands, public FileSupport
{
private:
	typedef struct _command_t {
		const char* name;
		void (SCSICD::*execute)(SASIDEV *);

		_command_t(const char* _name, void (SCSICD::*_execute)(SASIDEV *)) : name(_name), execute(_execute) { };
	} command_t;
	std::map<SCSIDEV::scsi_command, command_t*> commands;

	SASIDEV::ctrl_t *ctrl;

	void AddCommand(SCSIDEV::scsi_command, const char*, void (SCSICD::*)(SASIDEV *));

public:
	enum {
		TrackMax = 96							// Maximum number of tracks
	};

public:
	SCSICD();
	~SCSICD();

	bool Dispatch(SCSIDEV *) override;

	void Open(const Filepath& path) override;

	// Commands
	int Inquiry(const DWORD *cdb, BYTE *buf) override;	// INQUIRY command
	int Read(const DWORD *cdb, BYTE *buf, uint64_t block) override;		// READ command
	int ReadToc(const DWORD *cdb, BYTE *buf);			// READ TOC command
    int ReadSubChannel(const DWORD* cdb, BYTE* buf); // READ SUB-CHANNEL command
    bool PlayAudioMSF(const DWORD* cdb, BYTE* buf);
    bool PlayAudioIndex(const DWORD* cdb, BYTE* buf);
    bool PauseResume(const DWORD* cdb, BYTE* buf);
    bool StopPlayScan(const DWORD* cdb, BYTE* buf);
    bool PlayAudioLBA(DWORD start, DWORD end);
    bool ModeSelect(const DWORD *cdb, const BYTE *buf, int length) override;

private:
	// Open
	void OpenCue(const Filepath& path);				// Open(CUE)
	void OpenIso(const Filepath& path);				// Open(ISO)
	void OpenPhysical(const Filepath& path);			// Open(Physical)

	void ReadToc(SASIDEV *) override;
	void GetEventStatusNotification(SASIDEV *) override;
    void ReadSubChannel(SASIDEV*) override;
    void PlayAudioMSF(SASIDEV*) override;
    void PlayAudioIndex(SASIDEV*) override;
    void PauseResume(SASIDEV*) override;
    void StopPlayScan(SASIDEV*) override;

	void LBAtoMSF(DWORD lba, BYTE *msf) const;			// LBA→MSF conversion

    void UpdateReadTrack(int index);

	// Track management
	void ClearTrack();						// Clear the track
	int SearchTrack(DWORD lba) const;				// Track search
	CDTrack* track[TrackMax];						// Track opbject references
	int tracks;								// Effective number of track objects
	int dataindex;								// Current data track
    int audioPipe[2] { -1, -1 };

	int frame;								// Frame number

    CDAudioPlayer audioPlayer;
};
