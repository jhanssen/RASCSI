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

#include "scsicd.h"
#include "fileio.h"
#include "exceptions.h"
#include "bincue/CueParser.h"
#include <sstream>
#include <sys/mman.h>
#include "../rascsi.h"
#include "../log.h"
#include "../gpiobus.h"

static inline uint32_t MSFToLBA(uint8_t mm, uint8_t ss, uint8_t ff)
{
    return (((static_cast<uint32_t>(mm) * 60) + static_cast<uint32_t>(ss)) * 75) + static_cast<uint32_t>(ff);
}

#define EINTRWRAP(e, statement)                 \
    do {                                        \
        e = statement;                          \
    } while (e == -1 && errno == EINTR)

//===========================================================================
//
//	CD Track
//
//===========================================================================

//---------------------------------------------------------------------------
//
//	Constructor
//
//---------------------------------------------------------------------------
CDTrack::CDTrack(SCSICD *scsicd)
{
	ASSERT(scsicd);

	// Set parent CD-ROM device
	cdrom = scsicd;

	// Track defaults to disabled
	valid = false;

	// Initialize other data
	track_no = -1;
	first_lba = 0;
	last_lba = 0;
	audio = false;
	raw = false;
}

//---------------------------------------------------------------------------
//
//	Destructor
//
//---------------------------------------------------------------------------
CDTrack::~CDTrack()
{
}

//---------------------------------------------------------------------------
//
//	Init
//
//---------------------------------------------------------------------------
void CDTrack::Init(int track, DWORD pre, DWORD first, DWORD last, DWORD startByte, off_t offset, bool rawtrack)
{
	ASSERT(!valid);
	ASSERT(track >= 1);
	ASSERT(first < last);

	// Set and enable track number
	track_no = track;
	valid = TRUE;

	// Remember LBA
        pre_lba = pre;
	first_lba = first;
	last_lba = last;
        start_byte = startByte;
        offset_byte = offset;

        raw = rawtrack;
}

//---------------------------------------------------------------------------
//
//	Set Path
//
//---------------------------------------------------------------------------
void CDTrack::SetPath(bool cdda, const Filepath& path)
{
	ASSERT(valid);

	// CD-DA or data
	audio = cdda;

	// Remember the path
	imgpath = path;
}

//---------------------------------------------------------------------------
//
//	Get Path
//
//---------------------------------------------------------------------------
void CDTrack::GetPath(Filepath& path) const
{
	ASSERT(valid);

	// Return the path (by reference)
	path = imgpath;
}

//---------------------------------------------------------------------------
//
//	Add Index
//
//---------------------------------------------------------------------------
void CDTrack::AddIndex(int index, DWORD lba)
{
	ASSERT(valid);
	ASSERT(index > 0);
	ASSERT(first_lba <= lba);
	ASSERT(lba <= last_lba);

	// Currently does not support indexes
	ASSERT(FALSE);
}

//---------------------------------------------------------------------------
//
//	Gets the start of LBA
//
//---------------------------------------------------------------------------
DWORD CDTrack::GetPre() const
{
    ASSERT(valid);
    ASSERT(pre_lba <= first_lba);

    return pre_lba;
}

DWORD CDTrack::GetFirst() const
{
	ASSERT(valid);
	ASSERT(first_lba < last_lba);

	return first_lba;
}

bool CDTrack::GetRaw() const
{
    ASSERT(valid);
    return raw;
}

DWORD CDTrack::GetStartByte() const
{
    ASSERT(valid);

    return start_byte;
}

off_t CDTrack::GetOffset() const
{
    ASSERT(valid);

    return offset_byte;
}

//---------------------------------------------------------------------------
//
//	Get the end of LBA
//
//---------------------------------------------------------------------------
DWORD CDTrack::GetLast() const
{
	ASSERT(valid);
	ASSERT(first_lba < last_lba);

	return last_lba;
}

//---------------------------------------------------------------------------
//
//	Get the number of blocks
//
//---------------------------------------------------------------------------
DWORD CDTrack::GetBlocks() const
{
	ASSERT(valid);
	ASSERT(first_lba < last_lba);

	// Calculate from start LBA and end LBA
	return (DWORD)(last_lba - first_lba + 1);
}

//---------------------------------------------------------------------------
//
//	Get track number
//
//---------------------------------------------------------------------------
int CDTrack::GetTrackNo() const
{
	ASSERT(valid);
	ASSERT(track_no >= 1);

	return track_no;
}

//---------------------------------------------------------------------------
//
//	Is valid block
//
//---------------------------------------------------------------------------
bool CDTrack::IsValid(DWORD lba) const
{
	// FALSE if the track itself is invalid
	if (!valid) {
		return false;
	}

	// If the block is BEFORE the first block
        if (lba < pre_lba) {
		return false;
	}

	// If the block is AFTER the last block
	if (last_lba < lba) {
		return false;
	}

	// This track is valid
	return true;
}

//---------------------------------------------------------------------------
//
//	Is audio track
//
//---------------------------------------------------------------------------
bool CDTrack::IsAudio() const
{
	ASSERT(valid);

	return audio;
}

//===========================================================================
//
//	SCSI CD-ROM
//
//===========================================================================

//---------------------------------------------------------------------------
//
//	Constructor
//
//---------------------------------------------------------------------------
SCSICD::SCSICD() : Disk("SCCD"), ScsiMmcCommands(), FileSupport()
{
	// Frame initialization
	frame = 0;

	// Track initialization
	for (int i = 0; i < TrackMax; i++) {
		track[i] = NULL;
	}
	tracks = 0;
	dataindex = -1;

	AddCommand(SCSIDEV::eCmdReadToc, "ReadToc", &SCSICD::ReadToc);
	AddCommand(SCSIDEV::eCmdGetEventStatusNotification, "GetEventStatusNotification", &SCSICD::GetEventStatusNotification);
        AddCommand(SCSIDEV::eCmdReadSubChannel, "ReadSubChannel", &SCSICD::ReadSubChannel);
        AddCommand(SCSIDEV::eCmdPlayAudioMSF, "PlayAudioMSF", &SCSICD::PlayAudioMSF);
        AddCommand(SCSIDEV::eCmdPlayAudioIndex, "PlayAudioIndex", &SCSICD::PlayAudioIndex);
        AddCommand(SCSIDEV::eCmdPauseResume, "PauseResume", &SCSICD::PauseResume);
        AddCommand(SCSIDEV::eCmdStopPlayScan, "StopPlay/Scan", &SCSICD::StopPlayScan);

        int e;
        EINTRWRAP(e, pipe2(audioPipe, O_NONBLOCK));
        if (e == -1) {
            LOGERROR("Unable to make pipe for audio player: %d", errno);
        } else {
            GPIOBUS::instance()->addFileDescriptor(audioPipe[0], [this]() {
                LOGERROR("got audio fd");
                audioPlayer.stop();
            });
            audioPlayer.setWritePipe(audioPipe[1]);
        }
}

//---------------------------------------------------------------------------
//
//	Destructor
//
//---------------------------------------------------------------------------
SCSICD::~SCSICD()
{
	// Clear track
	ClearTrack();

	for (auto const& command : commands) {
		delete command.second;
	}
}

void SCSICD::AddCommand(SCSIDEV::scsi_command opcode, const char* name, void (SCSICD::*execute)(SASIDEV *))
{
	commands[opcode] = new command_t(name, execute);
}

bool SCSICD::Dispatch(SCSIDEV *controller)
{
	ctrl = controller->GetCtrl();

	if (commands.count(static_cast<SCSIDEV::scsi_command>(ctrl->cmd[0]))) {
		command_t *command = commands[static_cast<SCSIDEV::scsi_command>(ctrl->cmd[0])];

                LOGINFO("%s Executing %s ($%02X)", __PRETTY_FUNCTION__, command->name, (unsigned int)ctrl->cmd[0]);

		(this->*command->execute)(controller);

                /*
                if (ctrl->length > 0) {
                    LOGINFO("CD sending data %d", ctrl->length);
                    char buf[65];
                    uint32_t len = ctrl->length;
                    uint32_t pos = 0, start = 0, num = 0;
                    while (len > 0) {
                        if (pos + 3 > 64) {
                            buf[pos] = '\0';
                            LOGINFO("0x%02x %s", start, buf);
                            start += pos;
                            pos = 0;
                        }
                        sprintf(buf + pos, "%02x ", ctrl->buffer[num++]);
                        pos += 3;
                        --len;
                    }
                    if (pos > 0) {
                        buf[pos] = '\0';
                        LOGINFO("0x%02x %s", start, buf);
                    }
                }
                */

		return true;
	}

	LOGTRACE("%s Calling base class for dispatching $%02X", __PRETTY_FUNCTION__, (unsigned int)ctrl->cmd[0]);

	// The base class handles the less specific commands
	return Disk::Dispatch(controller);
}

//---------------------------------------------------------------------------
//
//	Open
//
//---------------------------------------------------------------------------
void SCSICD::Open(const Filepath& path)
{
	off_t size;

	ASSERT(!IsReady());

	// Initialization, track clear
	SetBlockCount(0);
	ClearTrack();

	// Open as read-only
	Fileio fio;
	if (!fio.Open(path, Fileio::ReadOnly)) {
		throw file_not_found_exception("Can't open CD-ROM file");
	}

	// Default sector size is 2048 bytes
	SetSectorSizeInBytes(GetConfiguredSectorSize() ? GetConfiguredSectorSize() : 2048, false);

	// Close and transfer for physical CD access
	if (path.GetPath()[0] == _T('\\')) {
		// Close
		fio.Close();

		// Open physical CD
		OpenPhysical(path);
	} else {
		// Get file size
        size = fio.GetFileSize();
		if (size <= 4) {
			fio.Close();
			throw io_exception("CD-ROM file size must be at least 4 bytes");
		}

		// Judge whether it is a CUE sheet or an ISO file
		TCHAR file[5];
		fio.Read(file, 4);
		file[4] = '\0';
		fio.Close();

                // If it starts with FILE or filename ends with '.cue'
                if (!strncasecmp(file, _T("FILE"), 4) || !strncasecmp(path.GetFileExt(), ".CUE", 4)) {
			// Open as CUE
			OpenCue(path);
		} else {
			// Open as ISO
			OpenIso(path);
		}
	}

        if (dataindex == -1 && tracks > 0) {
            UpdateReadTrack(0);
        }

	// Successful opening
	ASSERT(GetBlockCount() > 0);

	FileSupport::SetPath(path);

	// Attention if ready
	if (IsReady()) {
		SetAttn(true);
	}
}

//---------------------------------------------------------------------------
//
//	Open (CUE)
//
//---------------------------------------------------------------------------
void SCSICD::OpenCue(const Filepath& path)
{
    const auto sheet = CueParser::parseFile(path.GetPath());
    const auto dir = std::string(path.GetDir());

    if (sheet.files.empty()) {
        throw io_exception("No tracks in CUE file");
    }

    auto msfToLba = [](const CueParser::Length& len) {
        return MSFToLBA(len.mm, len.ss, len.ff);
    };

    auto sectorSize = [](const CueParser::Track::Type type) -> uint32_t {
        switch (type) {
        case CueParser::Track::Type::Mode1_2048:
            return 2048;
        case CueParser::Track::Type::Mode1_2352:
        case CueParser::Track::Type::Audio:
            return 2352;
        default:
            break;
        }
        return 0;
    };

    struct Track {
        std::string filename { };
        CueParser::Length prestart { };
        CueParser::Length start { };
        CueParser::Track::Type type { };
        uint32_t preStartByte { 0 };
        uint32_t startByte { 0 };
    };
    std::array<Track, TrackMax> pretracks;

    uint32_t currentSectorSize = 0;
    uint32_t numTracks = 0;
    for (const auto& file : sheet.files) {
        for (const auto& track : file.tracks) {
            const auto trackNo = track.number - 1;

            if (trackNo >= TrackMax) {
                throw io_exception("Too many tracks in CUE file");
            }

            if (trackNo >= numTracks)
                numTracks = trackNo + 1;

            bool foundStart = false, foundPreStart = false;
            CueParser::Length prestart { };
            CueParser::Length start { };
            for (const auto& len : track.index) {
                if (len.number == 0) {
                    prestart = len.length;
                    foundPreStart = true;
                } else if (len.number == 1) {
                    start = len.length;
                    foundStart = true;
                    break;
                }
            }

            if (!foundStart) {
                throw io_exception("No length for track in CUE file");
            }

            if (!foundPreStart) {
                prestart = start;
            }

            if (currentSectorSize == 0) {
                currentSectorSize = sectorSize(track.type);
            }
            if (currentSectorSize != sectorSize(track.type)) {
                throw io_exception("Mixed sector size CUE files not currently supported");
            }

            pretracks[trackNo] = {
                dir + file.filename,
                prestart,
                start,
                track.type,
                msfToLba(prestart) * sectorSize(track.type),
                msfToLba(start) * sectorSize(track.type)
            };
        }
    }

    if (numTracks == 0) {
        throw io_exception("No tracks in CUE file");
    }

    std::string currentfile;
    uint32_t lastFileSize = 0;
    Fileio fio;
    int32_t dataTrack = -1;
    uint32_t nextStart = 0;

    off_t changeOffset = 0;
    for (uint32_t trackNo = 0; trackNo < numTracks; ++trackNo) {
        const auto& tt = pretracks[trackNo];
        if (tt.filename.empty()) {
            throw io_exception("Missing track in CUE file");
        }

        Filepath tpath;
        tpath.SetPath(tt.filename.c_str());

        bool fileChanged = false;

        if (currentfile != tt.filename) {
            currentfile = tt.filename;

            if (!fio.Open(tpath, Fileio::ReadOnly)) {
                throw io_exception("Can't open track for CUE file");
            }

            // Get file size
            lastFileSize = fio.GetFileSize();
            fio.Close();

            fileChanged = true;

            LOGINFO("updated changeOffset to %lld", static_cast<long long>(changeOffset));
        }

        uint32_t pre = msfToLba(tt.prestart);
        uint32_t start = msfToLba(tt.start);
        uint32_t end;

        bool needsUpdate = true;
        if (trackNo + 1 < numTracks) {
            if (pretracks[trackNo + 1].filename == currentfile) {
                end = msfToLba(pretracks[trackNo + 1].prestart) - 1;
                needsUpdate = false;
            }
        }
        if (needsUpdate) {
            if (fileChanged) {
                pre += nextStart;
                start += nextStart;
            }
            end = start + ((lastFileSize - tt.startByte) / sectorSize(tt.type)) - 1;
        }
        nextStart = end + 1;

        const bool isDataTrack = (tt.type == CueParser::Track::Type::Mode1_2048
                                  || tt.type == CueParser::Track::Type::Mode1_2352);
        if (dataTrack == -1 && isDataTrack)
            dataTrack = trackNo;

        const bool isRaw = (tt.type == CueParser::Track::Type::Mode1_2352
                            || tt.type == CueParser::Track::Type::Audio);

        if (trackNo > 0 && fileChanged) {
            const auto preDelta = start - pre;
            changeOffset = (static_cast<off_t>(start) * -1) + preDelta;
        }

        LOGINFO("attaching track %u file '%s' at %u %u offset %lld %s %s", trackNo + 1, tt.filename.c_str(), start, end,
                static_cast<long long>(changeOffset), isDataTrack ? "data" : "audio", isRaw ? "raw": "cooked");

        track[trackNo] = new CDTrack(this);
        track[trackNo]->Init(trackNo + 1, pre, start, end, tt.preStartByte, changeOffset, isRaw);
        track[trackNo]->SetPath(isDataTrack ? false : true, tpath);
    }

    LOGINFO("cue initialized, %u tracks, data track is %d", numTracks, dataTrack);
    SetStatusCode(STATUS_NOERROR);

    tracks = numTracks;

    if (dataTrack != -1) {
        UpdateReadTrack(dataTrack);
    } else {
        dataindex = -1;
    }
}

//---------------------------------------------------------------------------
//
//	Open (ISO)
//
//---------------------------------------------------------------------------
void SCSICD::OpenIso(const Filepath& path)
{
	// Open as read-only
	Fileio fio;
	if (!fio.Open(path, Fileio::ReadOnly)) {
		throw io_exception("Can't open ISO CD-ROM file");
	}

	// Get file size
	off_t size = fio.GetFileSize();
	if (size < 0x800) {
		fio.Close();
		throw io_exception("ISO CD-ROM file size must be at least 2048 bytes");
	}

	// Read the first 12 bytes and close
	BYTE header[12];
	if (!fio.Read(header, sizeof(header))) {
		fio.Close();
		throw io_exception("Can't read header of ISO CD-ROM file");
	}

	// Check if it is RAW format
	BYTE sync[12];
	memset(sync, 0xff, sizeof(sync));
	sync[0] = 0x00;
	sync[11] = 0x00;
        bool rawfile = false;
	if (memcmp(header, sync, sizeof(sync)) == 0) {
		// 00,FFx10,00, so it is presumed to be RAW format
		if (!fio.Read(header, 4)) {
			fio.Close();
			throw io_exception("Can't read header of raw ISO CD-ROM file");
		}

		// Supports MODE1/2048 or MODE1/2352 only
		if (header[3] != 0x01) {
			// Different mode
			fio.Close();
			throw io_exception("Illegal raw ISO CD-ROM file header");
		}

		// Set to RAW file
		rawfile = true;
	}
	fio.Close();

        DWORD blockCount;
	if (rawfile) {
		// Size must be a multiple of 2536
		if (size % 2536) {
			stringstream error;
			error << "Raw ISO CD-ROM file size must be a multiple of 2536 bytes but is " << size << " bytes";
			throw io_exception(error.str());
		}

		// Set the number of blocks
                blockCount = (DWORD)(size / 0x930);
	} else {
		// Set the number of blocks
            blockCount = (DWORD)(size >> GetSectorSizeShiftCount());
	}

	// Create only one data track
	ASSERT(!track[0]);
	track[0] = new CDTrack(this);
        track[0]->Init(1, 0, 0, blockCount - 1, 0, 0, rawfile);
	track[0]->SetPath(false, path);
	tracks = 1;
        dataindex = -1;
}

//---------------------------------------------------------------------------
//
//	Open (Physical)
//
//---------------------------------------------------------------------------
void SCSICD::OpenPhysical(const Filepath& path)
{
	// Open as read-only
	Fileio fio;
	if (!fio.Open(path, Fileio::ReadOnly)) {
		throw io_exception("Can't open CD-ROM file");
	}

	// Get size
	off_t size = fio.GetFileSize();
	if (size < 0x800) {
		fio.Close();
		throw io_exception("CD-ROM file size must be at least 2048 bytes");
	}

	// Close
	fio.Close();

	// Effective size must be a multiple of 512
	size = (size / 512) * 512;

	// Create only one data track
	ASSERT(!track[0]);
	track[0] = new CDTrack(this);
        track[0]->Init(1, 0, 0, (DWORD)(size >> GetSectorSizeShiftCount()) - 1, 0, 0, false);
	track[0]->SetPath(false, path);
	tracks = 1;
        dataindex = -1;
}

void SCSICD::ReadToc(SASIDEV *controller)
{
	ctrl->length = ReadToc(ctrl->cmd, ctrl->buffer);
	if (ctrl->length <= 0) {
		// Failure (Error)
		controller->Error();
		return;
	}

	controller->DataIn();
}

void SCSICD::ReadSubChannel(SASIDEV* controller)
{
    ctrl->length = ReadSubChannel(ctrl->cmd, ctrl->buffer);
    if (ctrl->length <= 0) {
        // Failure (Error)
        controller->Error(ERROR_CODES::sense_key::ILLEGAL_REQUEST, ERROR_CODES::asc::INVALID_COMMAND_OPERATION_CODE);
        return;
    }

    controller->DataIn();
}

void SCSICD::PlayAudioMSF(SASIDEV* controller)
{
    if (!PlayAudioMSF(ctrl->cmd, ctrl->buffer)) {
        // Failure (Error)
        controller->Error(ERROR_CODES::sense_key::ILLEGAL_REQUEST, ERROR_CODES::asc::INVALID_COMMAND_OPERATION_CODE);
        return;
    }

    controller->Status();
}

void SCSICD::PlayAudioIndex(SASIDEV* controller)
{
    if (!PlayAudioIndex(ctrl->cmd, ctrl->buffer)) {
        // Failure (Error)
        controller->Error(ERROR_CODES::sense_key::ILLEGAL_REQUEST, ERROR_CODES::asc::INVALID_COMMAND_OPERATION_CODE);
        return;
    }

    controller->Status();
}

void SCSICD::PauseResume(SASIDEV* controller)
{
    if (!PauseResume(ctrl->cmd, ctrl->buffer)) {
        // Failure (Error)
        controller->Error(ERROR_CODES::sense_key::ILLEGAL_REQUEST, ERROR_CODES::asc::INVALID_COMMAND_OPERATION_CODE);
        return;
    }

    controller->Status();
}

void SCSICD::StopPlayScan(SASIDEV* controller)
{
    if (!StopPlayScan(ctrl->cmd, ctrl->buffer)) {
        // Failure (Error)
        controller->Error(ERROR_CODES::sense_key::ILLEGAL_REQUEST, ERROR_CODES::asc::INVALID_COMMAND_OPERATION_CODE);
        return;
    }

    controller->Status();
}

//---------------------------------------------------------------------------
//
//	INQUIRY
//
//---------------------------------------------------------------------------
int SCSICD::Inquiry(const DWORD *cdb, BYTE *buf)
{
	ASSERT(cdb);
	ASSERT(buf);

	// EVPD check
	if (cdb[1] & 0x01) {
		SetStatusCode(STATUS_INVALIDCDB);
		return FALSE;
	}

	// Basic data
	// buf[0] ... CD-ROM Device
	// buf[1] ... Removable
	// buf[2] ... SCSI-2 compliant command system
	// buf[3] ... SCSI-2 compliant Inquiry response
	// buf[4] ... Inquiry additional data
	memset(buf, 0, 8);
	buf[0] = 0x05;
	buf[1] = 0x80;
	buf[2] = 0x02;
	buf[3] = 0x02;
	buf[4] = 36 - 5;	// Required

	// Fill with blanks
	memset(&buf[8], 0x20, buf[4] - 3);

	// Padded vendor, product, revision
	memcpy(&buf[8], GetPaddedName().c_str(), 28);

//
// The following code worked with the modified Apple CD-ROM drivers. Need to
// test with the original code to see if it works as well....
//	buf[4] = 42;	// Required
//
//	// Fill with blanks
//	memset(&buf[8], 0x20, buf[4] - 3);
//
//	// Vendor name
//	memcpy(&buf[8], BENDER_SIGNATURE, strlen(BENDER_SIGNATURE));
//
//	// Product name
//	memcpy(&buf[16], "CD-ROM CDU-8003A", 16);
//
//	// Revision (XM6 version number)
////	sprintf(rev, "1.9a",
//	////			(int)major, (int)(minor >> 4), (int)(minor & 0x0f));
//	memcpy(&buf[32], "1.9a", 4);
//
//	//strcpy(&buf[35],"A1.9a");
//	buf[36]=0x20;
//	memcpy(&buf[37],"1999/01/01",10);

	// Size of data that can be returned
	int size = (buf[4] + 5);

	// Limit if the other buffer is small
	if (size > (int)cdb[4]) {
		size = (int)cdb[4];
	}

	return size;
}

void SCSICD::UpdateReadTrack(int index)
{
		// Delete current disk cache (no need to save)
		delete disk.dcache;
		disk.dcache = NULL;

		// Reset the number of blocks
		SetBlockCount(track[index]->GetBlocks());
    SetTotalBlockCount(track[tracks - 1]->GetLast() + 1);
		ASSERT(GetBlockCount() > 0);

    //const auto extra = track[index]->GetFirst() % 256;
    SetImageOffset(track[index]->GetOffset() * (track[index]->GetRaw() ? 2352 : 2048));

		// Recreate the disk cache
		Filepath path;
		track[index]->GetPath(path);

    Disk::Open(path);

    ASSERT(disk.dcache);
    disk.dcache->SetRawMode(track[index]->GetRaw());

		// Reset data index
		dataindex = index;
	}

//---------------------------------------------------------------------------
//
//	READ
//
//---------------------------------------------------------------------------
int SCSICD::Read(const DWORD *cdb, BYTE *buf, uint64_t block)
{
    ASSERT(buf);

    // LOGINFO("SCSICD::read");

	// Status check
	if (!CheckReady()) {
		return 0;
        }

        // Search for the track
	int index = SearchTrack(block);

        // LOGINFO("wanting to read block %llu, found track %d", static_cast<long long unsigned>(block), index);

	// if invalid, out of range
	if (index < 0) {
            LOGTRACE("status cd 2");
                SetStatusCode(STATUS_INVALIDLBA);
		return 0;
	}
	ASSERT(track[index]);

	// If different from the current data track
        if (dataindex != index) {
            UpdateReadTrack(index);
        }

	// Base class
	ASSERT(dataindex >= 0);
	return Disk::Read(cdb, buf, block);
}

//---------------------------------------------------------------------------
//
//	PLAY AUDIO
//
//---------------------------------------------------------------------------
bool SCSICD::PlayAudioLBA(DWORD start, DWORD end)
{
    const int startIndex = SearchTrack(start);
    const int endIndex = SearchTrack(end);

    if (startIndex == -1 || endIndex == -1 || startIndex > endIndex || endIndex >= tracks) {
        SetStatusCode(STATUS_INVALIDLBA);
        return false;
    }

    const int startTrackNo = SearchTrack(start);
    const int endTrackNo = SearchTrack(end);

    // verify that the tracks are audio tracks
    for (int t = startTrackNo; t <= endTrackNo; ++t) {
        if (!track[t] || !track[t]->IsAudio()) {
            LOGERROR("PlayAudioLBA no audio track for %d", t);
            SetStatusCode(STATUS_INVALIDLBA);
            return false;
        }
    }

    struct AudioState
    {
        ~AudioState() { if (data != nullptr) { munmap(data, dataLength); } }

        int currentTrack { -1 };
        uint64_t startOffset { 0 };
        uint32_t trackOffset { 0 };

        uint8_t* data { nullptr };
        size_t dataOffset { 0 };
        size_t dataLength { 0 };
        std::string dataPath { };
    };

    // > 0 is success, < 0 is failure, == 0 is stop
    std::shared_ptr<AudioState> state = std::make_shared<AudioState>();
    state->startOffset = start;
    state->trackOffset = (start - track[startTrackNo]->GetFirst()) * CDAudioPlayer::SectorSize;
    auto readData = [this, state](uint8_t* buf, uint32_t sectors) -> int32_t {
        auto trackNo = SearchTrack(state->startOffset);
        if (trackNo == -1) {
            LOGERROR("CD Audio: No audio data at offset %llu", static_cast<unsigned long long>(state->startOffset));
            return -1;
        }
        // LOGINFO("CD audio reading track %u %llu", trackNo, static_cast<unsigned long long>(state->startOffset));
        DWORD trackEnd = track[trackNo]->GetLast();
        uint32_t bufOffset = 0;
        uint32_t readSector = 0;
        for (; readSector < sectors; ++readSector) {
            // read sector
            if (trackNo != state->currentTrack) {
                // load track
                Filepath file;
                track[trackNo]->GetPath(file);
                std::string filePath(file.GetPath());

                if (filePath != state->dataPath) {
                    state->dataPath = std::move(filePath);
                    if (state->data != nullptr) {
                        munmap(state->data, state->dataLength);
                    }

                    Fileio fio;
                    if (!fio.Open(file, Fileio::ReadOnly)) {
                        LOGERROR("CD Audio: Unable to open '%s'", file.GetPath());
                        return -2;
                    }

                    state->dataLength = fio.GetFileSize();
                    state->data = reinterpret_cast<uint8_t*>(mmap(nullptr, state->dataLength, PROT_READ, MAP_PRIVATE, fio.Handle(), 0));

                    fio.Close();

                    if (state->data == MAP_FAILED) {
                        state->data = nullptr;
                        LOGERROR("CD Audio: Unable to mmap '%s'", file.GetPath());
                        return -3;
                    }

                }

                state->dataOffset = track[trackNo]->GetStartByte();

                // keep initial trackOffset
                if (state->currentTrack != -1) {
                    state->trackOffset = 0;
                }
                state->currentTrack = trackNo;

                char bufd[64];
                for (int i = 0; i < 15; ++i) {
                    sprintf(bufd + (i * 3), "%02x ", state->data[state->dataOffset + state->trackOffset + i]);
                }
                LOGINFO("CD initial data at %llu(%llu) %s", static_cast<unsigned long long>(state->dataOffset + state->trackOffset), static_cast<unsigned long long>(state->trackOffset), bufd);
            }
            // LOGINFO("CD audio reading sector %llu %llu", static_cast<unsigned long long>(bufOffset), static_cast<unsigned long long>(state->trackOffset));
            memcpy(buf + bufOffset, state->data + state->dataOffset + state->trackOffset, CDAudioPlayer::SectorSize);
            bufOffset += CDAudioPlayer::SectorSize;
            state->trackOffset += CDAudioPlayer::SectorSize;

            if (state->startOffset + readSector > trackEnd) {
                // load next track
                if (trackNo + 1 >= tracks) {
                    // done
                    state->startOffset += readSector;
                    return readSector;
                } else {
                    ++trackNo;
                    trackEnd = track[trackNo]->GetLast();
                }
            }
        }
        state->startOffset += readSector;
        return readSector;
    };

    audioPlayer.play(start, end, std::move(readData));
    return true;
}

bool SCSICD::PlayAudioMSF(const DWORD *cdb, BYTE *buf)
{
    const auto startLba = MSFToLBA(cdb[3], cdb[4], cdb[5]) - 150;
    const auto endLba = MSFToLBA(cdb[6], cdb[7], cdb[8]) - 150;

    LOGINFO("PlayAudioMSF %u %u %u -> %u %u %u", cdb[3], cdb[4], cdb[5], cdb[6], cdb[7], cdb[8]);

    return PlayAudioLBA(startLba, endLba);
}

bool SCSICD::PlayAudioIndex(const DWORD *cdb, BYTE *buf)
{
    const auto startTrackNo = cdb[4];
    const bool startPre = (cdb[5] == 0);

    const auto endTrackNo = cdb[7];
    const bool endPre = (cdb[8] == 0);
    const bool endEnd = (cdb[8] > 1);

    LOGINFO("PlayAudioIndex %u %u -> %u %u", cdb[4], cdb[5], cdb[7], cdb[8]);

    auto findTrack = [this](DWORD tno) -> CDTrack* {
        for (int t = 0; t < tracks; ++t) {
            if (track[t]->GetTrackNo() == static_cast<int>(tno))
                return track[t];
        }
        return nullptr;
    };

    auto startTrack = findTrack(startTrackNo);
    auto endTrack = findTrack(endTrackNo);
    if (startTrack == nullptr) {
        LOGERROR("PlayAudioIndex no start track");
        SetStatusCode(STATUS_INVALIDLBA);
        return 0;
    }

    const auto startLba = startPre ? startTrack->GetPre() : startTrack->GetFirst();
    DWORD endLba = 0;
    if (endTrack == nullptr) {
        endLba = track[tracks - 1]->GetLast();
    } else {
        if (endEnd) {
            endLba = endTrack->GetLast();
        } else if (endPre) {
            endLba = endTrack->GetPre();
        } else {
            endLba = endTrack->GetFirst();
        }
    }

    return PlayAudioLBA(startLba, endLba);
}

bool SCSICD::PauseResume(const DWORD *cdb, BYTE *buf)
{
    const bool resume = cdb[8] & 0x1;

    char buf2[64];
    for (int i = 0; i < 10; ++i) {
        sprintf(buf2 + (i * 3), "%02x ", cdb[i] & 0xFF);
    }
    LOGINFO("PAUSE %s", buf2);

    if (resume) {
        audioPlayer.resume();
    } else {
        audioPlayer.pause();
    }
    return true;
}

bool SCSICD::StopPlayScan(const DWORD */*cdb*/, BYTE */*buf*/)
{
    audioPlayer.stop();
    return true;
}

//---------------------------------------------------------------------------
//
//	READ SUB-CHANNEL
//
//---------------------------------------------------------------------------
int SCSICD::ReadSubChannel(const DWORD *cdb, BYTE *buf)
{
    ASSERT(cdb);
    ASSERT(buf);

    // Check if ready
    if (!CheckReady()) {
        return 0;
    }

    LOGINFO("SCSICD::readsubchannel");

    char buf2[64];
    for (int i = 0; i < 10; ++i) {
        sprintf(buf2 + (i * 3), "%02x ", cdb[i] & 0xFF);
    }
    LOGINFO("SUB %s", buf2);

    int length = cdb[7] << 8;
    length |= cdb[8];
    memset(buf, 0, length);

    // Get MSF Flag
    bool msf = cdb[1] & 0x02;

    // get SubQ Flag
    bool subq = cdb[2] & 0x40;

    LOGINFO("wanting to read sub msf %d subq %d length %d format %d track %d control %d",
            msf, subq, length, cdb[3], cdb[6], cdb[9]);

    if (!subq) {
        // no subq data
        return length;
    }

    switch (cdb[3]) {
    case 0x00: {
        // Q sub-code data
        break; }
    case 0x01: {
        // CD-ROM current position

        buf[3] = (BYTE)0xc;
        buf[4] = (BYTE)0x1;

        // audio status
        // no audio status to return
        if (!audioPlayer.isPlaying())
            return length;

        if (audioPlayer.isPaused()) {
            buf[1] = 0x12;
        } else {
            buf[1] = 0x11;
        }

        const auto sector = audioPlayer.playingSector();
        const auto trackIdx = SearchTrack(sector);
        if (trackIdx == -1) {
            // not good
            LOGERROR("Couldn't find track for subq 0x01: %u", sector);
            return 0;
        }

        buf[5] = 0x10;

        // track number
        buf[6] = track[trackIdx]->GetTrackNo();
        // index number, should handle pre here and set index 0x00
        buf[7] = 0x01;

        if (msf) {
            LBAtoMSF(sector, &buf[8]);
        } else {
            buf[10] = (BYTE)(sector >> 8);
            buf[11] = (BYTE)sector;
        }

        const int64_t relative = static_cast<int64_t>(sector) - track[trackIdx]->GetFirst();

        if (relative > 0) {
            if (msf) {
                LBAtoMSF(static_cast<DWORD>(relative), &buf[12]);
            } else {
                buf[14] = (BYTE)((relative & 0xFF00) >> 8);
                buf[15] = (BYTE)(relative & 0xFF);
            }
        }

        return length; }
    case 0x02: {
        // Media catalog number

        buf[3] = (BYTE)0x14;
        buf[4] = (BYTE)0x02;

        if (audioPlayer.isPlaying()) {
            if (audioPlayer.isPaused()) {
                buf[1] = 0x12;
            } else {
                buf[1] = 0x11;
            }
        }

        // TODO: read upc from image
        return length; }
    case 0x03: {
        // ISRC

        buf[3] = (BYTE)0x14;
        buf[4] = (BYTE)0x03;

        if (audioPlayer.isPlaying()) {
            if (audioPlayer.isPaused()) {
                buf[1] = 0x12;
            } else {
                buf[1] = 0x11;
            }

            const auto sector = audioPlayer.playingSector();
            const auto trackIdx = SearchTrack(sector);
            if (trackIdx == -1) {
                // not good
                LOGERROR("Couldn't find track for subq 0x03: %u", sector);
                return 0;
            }

            buf[5] = 0x10;

            // track number
            buf[6] = track[trackIdx]->GetTrackNo();
        }

        break; }
    }

    return 0;
}

//---------------------------------------------------------------------------
//
//	MODE SELECT
//
//---------------------------------------------------------------------------
bool SCSICD::ModeSelect(const DWORD *cdb, const BYTE *buf, int length)
{
    char buf2[492];
    uint32_t off = 0;

    uint32_t pageLength = 0;

    if (length >= 4) {
        for (uint32_t i = 0; i < 4; ++i) {
            sprintf(buf2 + ((off++) * 3), "%02x ", buf[i] & 0xFF);
        }
        uint32_t coff = 4;
        if (buf[3] == 8 && length >= 12) {
            for (uint32_t i = 4; i < 12; ++i) {
                sprintf(buf2 + ((off++) * 3), "%02x ", buf[i] & 0xFF);
            }
            coff += 8;

            pageLength = (buf[9] << 16) | (buf[10] << 8) | buf[11];
        }

        LOGINFO("MODESELECT %u %s", pageLength, buf2);

        while (coff < static_cast<uint32_t>(length)) {
            LOGINFO("- PAGE 0x%02x", buf[coff] & 0x3F);
            coff += buf[coff + 1] + 1;
        }
    }

    return pageLength == 2048;
}

//---------------------------------------------------------------------------
//
//	READ TOC
//
//---------------------------------------------------------------------------
int SCSICD::ReadToc(const DWORD *cdb, BYTE *buf)
{
	ASSERT(cdb);
	ASSERT(buf);

	// Check if ready
	if (!CheckReady()) {
		return 0;
	}

	// If ready, there is at least one track
	ASSERT(tracks > 0);
	ASSERT(track[0]);

	// Get allocation length, clear buffer
	int length = cdb[7] << 8;
	length |= cdb[8];
	memset(buf, 0, length);

	// Get MSF Flag
	bool msf = cdb[1] & 0x02;

        // Get format
        int format = (cdb[9] >> 6) & 0x3;

        LOGINFO("SCSICD::readtoc length %d msf %d starting %d control %d", length, msf, cdb[6], cdb[9]);

        char buf2[64];
        for (int i = 0; i < 10; ++i) {
            sprintf(buf2 + (i * 3), "%02x ", cdb[i] & 0xFF);
        }
        LOGINFO("TOC %s", buf2);

        if (format == 0x1) {
            // session information

            // Create header
            buf[0] = (BYTE)0x0;
            buf[1] = (BYTE)0xa;
            buf[2] = (BYTE)0x1;
            buf[3] = (BYTE)0x1;

            if (track[0]->IsAudio()) {
                // audio track
                buf[5] = 0x10;
            } else {
                // data track
                buf[5] = 0x14;
            }


            buf[6] = (BYTE)track[0]->GetTrackNo();

            DWORD lba = track[0]->GetFirst();
            if (msf) {
                LBAtoMSF(lba, &buf[8]);
            } else {
                buf[10] = (BYTE)(lba >> 8);
                buf[11] = (BYTE)lba;
            }

            return length;
        }

	// Get and check the last track number
	int last = track[tracks - 1]->GetTrackNo();
	if ((int)cdb[6] > last) {
		// Except for AA
		if (cdb[6] != 0xaa) {
                    LOGINFO("status cd 3");
			SetStatusCode(STATUS_INVALIDCDB);
			return 0;
		}
	}

        auto writeLeadOut = [this, msf](BYTE* lbuf) {
            if (track[tracks - 1]->IsAudio()) {
                // audio track
                lbuf[1] = 0x10;
            } else {
                // data track
                lbuf[1] = 0x14;
            }
            lbuf[2] = 0xaa;
            DWORD lba = track[tracks - 1]->GetLast() + 1;
            if (msf) {
                LBAtoMSF(lba, &lbuf[4]);
            } else {
                lbuf[6] = (BYTE)(lba >> 8);
                lbuf[7] = (BYTE)lba;
            }
        };

        if (cdb[6] == 0xaa) {
            // Returns the final LBA+1 because it is AA
            buf[0] = 0x00;
            buf[1] = 0x0a;
            buf[2] = (BYTE)track[0]->GetTrackNo();
            buf[3] = (BYTE)last;
            writeLeadOut(buf + 4);
            return length;
        }

	// Check start index
	int index = 0;
	if (cdb[6] != 0x00) {
		// Advance the track until the track numbers match
		while (track[index]) {
			if ((int)cdb[6] == track[index]->GetTrackNo()) {
				break;
			}
			index++;
		}

		if (!track[index]) {
                    LOGINFO("status cd 4");
			SetStatusCode(STATUS_INVALIDCDB);
			return 0;
		}
	}

	// Number of track descriptors returned this time (number of loops)
	int loop = last - track[index]->GetTrackNo() + 1;
	ASSERT(loop >= 1);

	// Create header
        buf[0] = (BYTE)((((loop + 1) << 3) + 2) >> 8);
        buf[1] = (BYTE)(((loop + 1) << 3) + 2);
        buf[2] = (BYTE)track[index]->GetTrackNo();
	buf[3] = (BYTE)last;
	buf += 4;

	// Loop....
	for (int i = 0; i < loop; i++) {
		// ADR and Control
		if (track[index]->IsAudio()) {
			// audio track
			buf[1] = 0x10;
		} else {
			// data track
			buf[1] = 0x14;
		}

		// track number
		buf[2] = (BYTE)track[index]->GetTrackNo();

		// track address
		if (msf) {
			LBAtoMSF(track[index]->GetFirst(), &buf[4]);
		} else {
			buf[6] = (BYTE)(track[index]->GetFirst() >> 8);
			buf[7] = (BYTE)(track[index]->GetFirst());
		}

		// Advance buffer pointer and index
		buf += 8;
		index++;
	}

        // lead out
        writeLeadOut(buf);

	// Always return only the allocation length
	return length;
}

void SCSICD::GetEventStatusNotification(SASIDEV *controller)
{
	if (!(ctrl->cmd[1] & 0x01)) {
		// Asynchronous notification is optional and not supported by rascsi
		controller->Error(ERROR_CODES::sense_key::ILLEGAL_REQUEST, ERROR_CODES::asc::INVALID_FIELD_IN_CDB);
		return;
	}

	LOGTRACE("Received request for event polling, which is currently not supported");
	controller->Error(ERROR_CODES::sense_key::ILLEGAL_REQUEST, ERROR_CODES::asc::INVALID_FIELD_IN_CDB);
}

//---------------------------------------------------------------------------
//
//	LBA→MSF Conversion
//
//---------------------------------------------------------------------------
void SCSICD::LBAtoMSF(DWORD lba, BYTE *msf) const
{
	// 75 and 75*60 get the remainder
	DWORD m = lba / (75 * 60);
	DWORD s = lba % (75 * 60);
	DWORD f = s % 75;
	s /= 75;

	// The base point is M=0, S=2, F=0
	s += 2;
	if (s >= 60) {
		s -= 60;
		m++;
	}

	// Store
	ASSERT(m < 0x100);
	ASSERT(s < 60);
	ASSERT(f < 75);
	msf[0] = 0x00;
	msf[1] = (BYTE)m;
	msf[2] = (BYTE)s;
	msf[3] = (BYTE)f;
}

//---------------------------------------------------------------------------
//
//	Clear Track
//
//---------------------------------------------------------------------------
void SCSICD::ClearTrack()
{
	// delete the track object
	for (int i = 0; i < TrackMax; i++) {
		if (track[i]) {
			delete track[i];
			track[i] = NULL;
		}
	}

	// Number of tracks is 0
	tracks = 0;

	// No settings for data and audio
	dataindex = -1;
}

//---------------------------------------------------------------------------
//
//	Track Search
//	* Returns -1 if not found
//
//---------------------------------------------------------------------------
int SCSICD::SearchTrack(DWORD lba) const
{
	// Track loop
	for (int i = 0; i < tracks; i++) {
		// Listen to the track
		ASSERT(track[i]);
		if (track[i]->IsValid(lba)) {
			return i;
		}
	}

	// Track wasn't found
	return -1;
}

void CDAudioPlayer::RingBuffer::resize(uint32_t size)
{
    data.resize(size);
    readStart = writeStart = 0;
}

CDAudioPlayer::RingBuffer::ReadAddresses CDAudioPlayer::RingBuffer::readAddresses(uint32_t size)
{
    if (readStart <= writeStart) {
        const uint32_t num = std::min<uint32_t>(writeStart - readStart, size);
        if (num == 0) {
            return std::make_tuple(nullptr, 0, nullptr, 0);
        }
        const uint8_t* ptr = data.data() + readStart;
        return std::make_tuple(ptr, num, nullptr, 0);
    } else {
        const uint32_t num1 = std::min<uint32_t>(data.size() - readStart, size);
        const uint8_t* ptr1 = data.data() + readStart;
        if (readStart + num1 == data.size() && num1 < size && writeStart > 0) {
            const uint32_t num2 = std::min(size - num1, writeStart);
            const uint8_t* ptr2 = data.data();
            return std::make_tuple(ptr1, num1, ptr2, num2);
        }
        return std::make_tuple(ptr1, num1, nullptr, 0);
    }
}

void CDAudioPlayer::RingBuffer::commitRead(uint32_t size)
{
    if (readStart <= writeStart) {
        assert(readStart + size <= writeStart);
        readStart += size;
    } else if (readStart + size < data.size()) {
        readStart += size;
    } else {
        const auto over = (readStart + size) - data.size();
        ASSERT(over <= writeStart);
        readStart = over;
    }
}

uint32_t CDAudioPlayer::RingBuffer::read(uint8_t* out, uint32_t size)
{
    const auto wa = readAddresses(size);
    const auto ptr1 = std::get<0>(wa);
    const auto sz1 = std::get<1>(wa);
    if (ptr1 == nullptr)
        return 0;

    ASSERT(sz1 > 0);
    memcpy(out, ptr1, sz1);

    const auto ptr2 = std::get<2>(wa);
    const auto sz2 = std::get<3>(wa);
    if (ptr2 == nullptr) {
        commitRead(sz1);
        return sz1;
    }

    ASSERT(sz2 > 0);
    memcpy(out + sz1, ptr2, sz2);

    commitRead(sz1 + sz2);
    return sz1 + sz2;
}

CDAudioPlayer::RingBuffer::WriteAddresses CDAudioPlayer::RingBuffer::writeAddresses(uint32_t size)
{
    if (writeStart < readStart) {
        const uint32_t num = std::min<uint32_t>((readStart - writeStart) - 1, size);
        if (num == 0) {
            return std::make_tuple(nullptr, 0, nullptr, 0);
        }
        uint8_t* ptr = data.data() + writeStart;
        return std::make_tuple(ptr, num, nullptr, 0);
    } else {
        if (writeStart == data.size() - 1 && readStart == 0) {
            return std::make_tuple(nullptr, 0, nullptr, 0);
        }
        const uint32_t num1 = std::min<uint32_t>((data.size() - writeStart) - ((readStart == 0) ? 1 : 0), size);
        uint8_t* ptr1 = data.data() + writeStart;
        if (writeStart + num1 == data.size() && num1 < size) {
            ASSERT(readStart > 0);
            const uint32_t num2 = std::min(size - num1, readStart - 1);
            uint8_t* ptr2 = data.data();
            return std::make_tuple(ptr1, num1, ptr2, num2);
        }
        return std::make_tuple(ptr1, num1, nullptr, 0);
    }
}

void CDAudioPlayer::RingBuffer::commitWrite(uint32_t size)
{
    if (writeStart < readStart) {
        ASSERT(writeStart + size < readStart);
        writeStart += size;
    } else if (writeStart + size < data.size()) {
        writeStart += size;
    } else {
        const auto over = (writeStart + size) - data.size();
        ASSERT(readStart > 0);
        ASSERT(over < readStart);
        writeStart = over;
    }
}

uint32_t CDAudioPlayer::RingBuffer::write(const uint8_t* in, uint32_t size)
{
    const auto wa = writeAddresses(size);
    auto ptr1 = std::get<0>(wa);
    const auto sz1 = std::get<1>(wa);
    if (ptr1 == nullptr)
        return 0;

    ASSERT(sz1 > 0);
    memcpy(ptr1, in, sz1);

    auto ptr2 = std::get<2>(wa);
    const auto sz2 = std::get<3>(wa);
    if (ptr2 == nullptr) {
        commitWrite(sz1);
        return sz1;
    }

    ASSERT(sz2 > 0);
    memcpy(ptr2, in + sz1, sz2);

    commitWrite(sz1 + sz2);
    return sz1 + sz2;
}

uint32_t CDAudioPlayer::RingBuffer::writeAvailable() const
{
    if (writeStart < readStart)
        return (readStart - writeStart) - 1;
    else if (readStart == 0)
        return data.size() - writeStart - 1;
    return (data.size() - writeStart) + (readStart - 1);
}

uint32_t CDAudioPlayer::RingBuffer::readAvailable() const
{
    if (readStart <= writeStart)
        return writeStart - readStart;
    return (data.size() - readStart) + writeStart;
}

CDAudioPlayer::CDAudioPlayer()
{
}

CDAudioPlayer::~CDAudioPlayer()
{
    stop();
}

#define CHECKALSA(...)                          \
    if (err < 0) {                              \
        LOGERROR(__VA_ARGS__);                  \
        return false;                           \
    }

#define EAGAINALSA(e, cnt, statement)           \
    cnt = 0;                                    \
    do {                                        \
        e = statement;                          \
        if (e == -EAGAIN)                       \
            usleep(100);                        \
    } while (e == -EAGAIN && ++cnt <= 5)

void CDAudioPlayer::setWritePipe(int wp)
{
    writePipe = wp;
}

bool CDAudioPlayer::play(DWORD start, DWORD end, std::function<int32_t(uint8_t*, uint32_t)>&& read)
{
    if (pcmHandle != nullptr)
        stop();

    startSector = start;
    endSector = end;
    readFunction = std::move(read);

    paused = false;
    finished = false;

    int err = snd_pcm_open(&pcmHandle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    CHECKALSA("Unable to open alsa device: %d", err);

    snd_pcm_hw_params_t* hwParams;
    snd_pcm_hw_params_malloc(&hwParams);

    err = snd_pcm_hw_params_any(pcmHandle, hwParams);
    CHECKALSA("Unable to set alsa hw any: %d", err);

    unsigned int rrate = 44100;

    err = snd_pcm_hw_params_set_access(pcmHandle, hwParams, SND_PCM_ACCESS_RW_INTERLEAVED);
    CHECKALSA("Unable to set alsa hw SND_PCM_ACCESS_RW_INTERLEAVED: %d", err);
    err = snd_pcm_hw_params_set_format(pcmHandle, hwParams, SND_PCM_FORMAT_S16_LE);
    CHECKALSA("Unable to set alsa hw SND_PCM_FORMAT_S16_LE: %d", err);
    err = snd_pcm_hw_params_set_rate_near(pcmHandle, hwParams, &rrate, nullptr);
    CHECKALSA("Unable to set alsa hw rate: %d", err);
    err = snd_pcm_hw_params_set_channels(pcmHandle, hwParams, 2);
    CHECKALSA("Unable to set alsa hw channels: %d", err);

    periodSize = 294;
    int dir = -1;
    err = snd_pcm_hw_params_set_period_size_near(pcmHandle, hwParams, &periodSize, &dir);
    CHECKALSA("Unable to set alsa hw period_size");

    err = snd_pcm_hw_params(pcmHandle, hwParams);
    CHECKALSA("Unable to set alsa hw params: %d", err);

    // err = snd_pcm_hw_params_get_buffer_size(hwParams, &bufferSize);
    // CHECKALSA("Unable to set alsa hw buffer_size: %d", err);

    // LOGINFO("alsa got period %llu bufsiz %llu", static_cast<unsigned long long>(periodSize), static_cast<unsigned long long>(bufferSize));

    snd_pcm_hw_params_free(hwParams);

    LOGINFO("alsa params period %llu rate %u", static_cast<unsigned long long>(periodSize), rrate);

    periodSize4 = periodSize * 4;
    periodData.resize(periodSize4 * 2);
    sectorData.resize(RingHalf * SectorSize);

    if (periodSize4 * 2 > RingHalf * SectorSize) {
        // this won't be good
        LOGERROR("period size too large %llu", static_cast<unsigned long long>(periodSize));
        stop();
        return false;
    }

    /*
    snd_pcm_sw_params_t* swParams;
    snd_pcm_sw_params_malloc(&swParams);
    err = snd_pcm_sw_params_current(pcmHandle, swParams);
    CHECKALSA("Unable to set alsa sw current: %d", err);
    err = snd_pcm_sw_params_set_start_threshold(pcmHandle, swParams, bufferSize - periodSize);
    CHECKALSA("Unable to set alsa sw start_threshold: %d", err);
    err = snd_pcm_sw_params_set_avail_min(pcmHandle, swParams, periodSize);
    CHECKALSA("Unable to set alsa sw avail_min: %d", err);

    err = snd_pcm_sw_params(pcmHandle, swParams);
    CHECKALSA("Unable to set alsa sw params: %d", err);

    snd_pcm_sw_params_free(swParams);
    */

    // find the first multiple of SectorSize that's larger than periodSize4 * 2 and also divisible by periodSize
    /*
    uint32_t sectorm = 1;
    while ((SectorSize * sectorm) < periodSize4 * 2)
        ++sectorm;
    uint32_t mulLimit = 0;
    while ((++mulLimit < 200) && ((SectorSize * sectorm) % periodSize)) {
        ++sectorm;
    }
    if ((SectorSize * sectorm) % periodSize) {
        LOGERROR("Unable to find multiple of SectorSize that's divisible by period size %u", static_cast<uint32_t>(periodSize));
        return false;
    }
    LOGINFO("CD Audio: Found sector multiple %u", sectorm);
    sectorMultiple = sectorm;

    audioData.resize(SectorSize * sectorMultiple);
    */

    ringBuffer.resize(SectorSize * RingTotal);
    const auto wa = ringBuffer.writeAddresses(RingHalf * SectorSize);
    ASSERT(std::get<2>(wa) == nullptr);

    auto nread = readFunction(std::get<0>(wa), std::min<uint32_t>(50, endSector - startSector + 1));
    ringBuffer.commitWrite(nread * SectorSize);

    if (ringBuffer.readAvailable() < periodSize4 * 2) {
        LOGERROR("Unable to read audio data");
        return false;
    }

    audioSector = nread;

    err = snd_pcm_prepare(pcmHandle);
    CHECKALSA("Unable to prepare alsa: %d", err);

    ringBuffer.read(periodData.data(), periodSize4 * 2);

    int cnt;
    EAGAINALSA(err, cnt, snd_pcm_writei(pcmHandle, periodData.data(), periodSize * 2));
    CHECKALSA("Unable to write initial alsa data: %d", err);

    char bufd[64];
    for (int i = 0; i < 15; ++i) {
        sprintf(bufd + (i * 3), "%02x ", periodData.data()[i]);
    }
    LOGINFO("CD actual data data hepp %s", bufd);

    err = snd_async_add_pcm_handler(&pcmCallback, pcmHandle, readCallback, this);
    CHECKALSA("Unable to add alsa callback: %d", err);

    err = snd_pcm_start(pcmHandle);
    CHECKALSA("Unable to add start pcm: %d", err);

    LOGINFO("started audio %llu %llu", static_cast<unsigned long long>(start), static_cast<unsigned long long>(end));
    return true;
}

DWORD CDAudioPlayer::playingSector() const
{
    return startSector + audioSector;
}

bool CDAudioPlayer::isPaused() const
{
    return paused;
}

bool CDAudioPlayer::isFinished() const
{
    return finished;
}

bool CDAudioPlayer::isPlaying() const
{
    return pcmHandle != nullptr;
}

bool CDAudioPlayer::pause()
{
    if (paused || finished)
        return true;
    if (pcmHandle == nullptr)
        return false;
    paused = true;
    int err, cnt;
    EAGAINALSA(err, cnt, snd_pcm_drain(pcmHandle));
    CHECKALSA("Unable to drain pcm (pause): %d", err);
    if (pcmCallback != nullptr) {
        snd_async_del_handler(pcmCallback);
        pcmCallback = nullptr;
    }
    return true;
}

bool CDAudioPlayer::resume()
{
    if (!paused)
        return true;
    if (pcmHandle == nullptr)
        return false;
    paused = false;
    int err;
    err = snd_async_add_pcm_handler(&pcmCallback, pcmHandle, readCallback, this);
    CHECKALSA("Unable to add alsa callback (resume): %d", err);
    err = snd_pcm_prepare(pcmHandle);
    CHECKALSA("Unable to prepare alsa (resume): %d", err);

    err = snd_pcm_start(pcmHandle);
    CHECKALSA("Unable to add start pcm: %d", err);

    return true;
}

void CDAudioPlayer::finish()
{
    // finished is set from readCallback, otherwise it makes no sense to call this function
    if (!finished)
        return;
    if (pcmHandle != nullptr && pcmCallback != nullptr) {
        int err, cnt;
        EAGAINALSA(err, cnt, snd_pcm_drain(pcmHandle));
        snd_async_del_handler(pcmCallback);
        pcmCallback = nullptr;
    }
}

void CDAudioPlayer::stop()
{
    if (pcmHandle == nullptr)
        return;
    int e, cnt;
    EAGAINALSA(e, cnt, snd_pcm_drain(pcmHandle));
    if (pcmCallback != nullptr)
        snd_async_del_handler(pcmCallback);
    EAGAINALSA(e, cnt, snd_pcm_close(pcmHandle));
    readFunction = {};
    pcmHandle = nullptr;
    pcmCallback = nullptr;
}

void CDAudioPlayer::readCallback(snd_async_handler_t* callback)
{
    CDAudioPlayer* player = static_cast<CDAudioPlayer*>(snd_async_handler_get_callback_private(callback));
    if (player->inCallback.exchange(true)) {
        return;
    }

    snd_pcm_sframes_t avail = snd_pcm_avail_update(player->pcmHandle);
    while (avail >= static_cast<long int>(player->periodSize)) {
        int err = 0, cnt;

        const auto r = player->ringBuffer.read(player->periodData.data(), player->periodSize4);
        if (r > 0) {
            EAGAINALSA(err, cnt, snd_pcm_writei(player->pcmHandle, player->periodData.data(), r / 4));
        }

        if (err < 0) {
            if (player->paused) {
                player->inCallback = false;
                return;
            }
            if (err == -EPIPE) {
                EAGAINALSA(err, cnt, snd_pcm_prepare(player->pcmHandle));
            } else if (err == -ESTRPIPE || err == -EIO) {
                EAGAINALSA(err, cnt, snd_pcm_resume(player->pcmHandle));
                if (err < 0) {
                    EAGAINALSA(err, cnt, snd_pcm_prepare(player->pcmHandle));
                }
            }
            if (err < 0) {
                LOGERROR("Unable to write audio data (callback)");
                if (player->writePipe != -1) {
                    char w = 'e';
                    EINTRWRAP(err, ::write(player->writePipe, &w, 1));
                }
                player->inCallback = false;
                return;
            }
        }
        avail = snd_pcm_avail_update(player->pcmHandle);

        if (!player->finished && player->ringBuffer.readAvailable() < player->periodSize4 * 2) {
            // read new sectors
            const auto audioSector = player->audioSector;
            const auto nsectors = std::min<uint32_t>(RingHalf, (player->endSector - (audioSector + player->startSector)) + 1);
            // LOGINFO("welp, loading sectors %u %u %u (%u)", audioSector, player->startSector, player->endSector, nsectors);
            if (nsectors == 0) {
                player->finished = true;
                if (player->writePipe != -1) {
                    char w = 'f';
                    EINTRWRAP(err, ::write(player->writePipe, &w, 1));
                    err = 0;
                }
                player->inCallback = false;
                return;
            }
            player->audioSector = audioSector + nsectors;

            ASSERT(player->ringBuffer.writeAvailable() >= nsectors * SectorSize);
            auto read = player->readFunction(player->sectorData.data(), nsectors);
            if (read <= 0) {
                // stop
                LOGERROR("Unable to read audio data (callback)");
                if (player->writePipe != -1) {
                    char w = 'e';
                    EINTRWRAP(err, ::write(player->writePipe, &w, 1));
                }
                player->inCallback = false;
                return;
            }
            player->ringBuffer.write(player->sectorData.data(), read * SectorSize);
        }
    }
    player->inCallback = false;
}
