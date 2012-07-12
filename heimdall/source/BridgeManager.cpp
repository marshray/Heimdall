/* Copyright (c) 2010-2011 Benjamin Dobell, Glass Echidna
   Copyright (c) 2012 Marsh Ray

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.*/

#define GTP7510 1

// C Standard Library
#include <assert.h>
#include <stdio.h>

// libusb
#include <libusb.h>

// Heimdall
#include "BeginDumpPacket.h"
#include "BridgeManager.h"
#include "SetupSessionPacket.h"
#include "SetupSessionResponse.h"
#include "DumpPartFileTransferPacket.h"
#include "DumpPartPitFilePacket.h"
#include "DumpResponse.h"
#include "EndModemFileTransferPacket.h"
#include "EndPhoneFileTransferPacket.h"
#include "EndPitFileTransferPacket.h"
#include "EndSessionPacket.h"
#include "FileTransferPacket.h"
#include "FlashPartFileTransferPacket.h"
#include "FlashPartPitFilePacket.h"
#include "InboundPacket.h"
#include "Interface.h"
#include "OutboundPacket.h"
#include "PitFilePacket.h"
#include "PitFileResponse.h"
#include "ReceiveFilePartPacket.h"
#include "ResponsePacket.h"
#include "SendFilePartPacket.h"
#include "SendFilePartResponse.h"

// Future versions of libusb will use usb_interface instead of interface.
#define usb_interface interface

#define CLASS_CDC 0x0A

#include <time.h>
#include <sys/time.h>


#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif



using namespace Heimdall;

const DeviceIdentifier BridgeManager::supportedDevices[BridgeManager::kSupportedDeviceCount] = {
	DeviceIdentifier(BridgeManager::kVidSamsung, BridgeManager::kPidGalaxyS),
	DeviceIdentifier(BridgeManager::kVidSamsung, BridgeManager::kPidGalaxyS2),
	DeviceIdentifier(BridgeManager::kVidSamsung, BridgeManager::kPidDroidCharge)
};

enum
{
	kMaxSequenceLength = 800
};

#if GTP7510

static void LogLibusbResult(int iLibusbErrorValue)
{
	char const * psz = 0;
	switch (iLibusbErrorValue)
	{
	case LIBUSB_SUCCESS: psz = "LIBUSB_SUCCESS"; break;
	case LIBUSB_ERROR_IO: psz = "LIBUSB_ERROR_IO"; break;
	case LIBUSB_ERROR_INVALID_PARAM: psz = "LIBUSB_ERROR_INVALID_PARAM"; break;
	case LIBUSB_ERROR_ACCESS: psz = "LIBUSB_ERROR_ACCESS"; break;
	case LIBUSB_ERROR_NO_DEVICE: psz = "LIBUSB_ERROR_NO_DEVICE"; break;
	case LIBUSB_ERROR_NOT_FOUND: psz = "LIBUSB_ERROR_NOT_FOUND"; break;
	case LIBUSB_ERROR_BUSY: psz = "LIBUSB_ERROR_BUSY"; break;
	case LIBUSB_ERROR_TIMEOUT: psz = "LIBUSB_ERROR_TIMEOUT"; break;
	case LIBUSB_ERROR_OVERFLOW: psz = "LIBUSB_ERROR_OVERFLOW"; break;
	case LIBUSB_ERROR_PIPE: psz = "LIBUSB_ERROR_PIPE"; break;
	case LIBUSB_ERROR_INTERRUPTED: psz = "LIBUSB_ERROR_INTERRUPTED"; break;
	case LIBUSB_ERROR_NO_MEM: psz = "LIBUSB_ERROR_NO_MEM"; break;
	case LIBUSB_ERROR_NOT_SUPPORTED: psz = "LIBUSB_ERROR_NOT_SUPPORTED"; break;
	case LIBUSB_ERROR_OTHER: psz = "LIBUSB_ERROR_OTHER"; break;
	default: psz = "*unknown libusb error code*"; break;
	}
	Interface::Print(psz);
}

static void LogControlTransferResult(int rc)
{
	if (0 <= rc)
		Interface::Print("OK (%d bytes transferred)\n", rc);
	else if (LIBUSB_ERROR_PIPE == rc)
		Interface::Print("EPIPE\n", rc);
	else
	{
		LogLibusbResult(rc);
		//char const * psz = "";
		//switch (rc)
		//{
		//case LIBUSB_ERROR_PIPE: psz = " - control request not supported by device (may be expected)."; break;
		//}
		//Interface::Print("%s\n", psz);
	}
}

bool BridgeManager::ResetInterface()
{
	Interface::Print("Clearing halts.\n");
	{
		//	In theory, we could clear any halt condition on the default control pipe too.
		//	But since we're successfully talking to the device now, that's probably unnecessary.
		//	That condition may require a reset to clear anyway.

		int endpointAddresses[] = { bEndpointAddress_comm, bEndpointAddress_data_in, bEndpointAddress_data_out };
		for (int i = 0; i < sizeof(endpointAddresses)/sizeof(endpointAddresses[0]); ++i)
		{
			int endpointAddress = endpointAddresses[i];
			if (0 <= endpointAddress)
			{
				unsigned char bEndpointAddress = (unsigned char)(endpointAddress & 0xFF);
				Interface::Print("Clearing halt from endpoint address %02X . . . ", bEndpointAddress);

				uint8_t bmRequestType =
					  LIBUSB_ENDPOINT_OUT // host-to-device
					| LIBUSB_REQUEST_TYPE_STANDARD
					| LIBUSB_RECIPIENT_ENDPOINT;
				assert(bmRequestType == 0x02);

				uint8_t bRequest = LIBUSB_REQUEST_CLEAR_FEATURE; // 0x01
				assert(bRequest == 0x01);

				uint16_t wValue = 0x0000; // feature selector ENDPOINT_HALT
				uint16_t wIndex = bEndpointAddress; // endpoint
				unsigned char *data = 0;
				uint16_t length = 0;
				unsigned int timeout = 0; // wait forever
				int rc = libusb_control_transfer(
					deviceHandle, bmRequestType, bRequest, wValue, wIndex, data, length, timeout);
				if (0 <= rc)
					Interface::Print("OK (%d bytes transferred)\n", rc);
				else
				{
					LogLibusbResult(rc);
					char const * psz = "";
					switch (rc)
					{
					case LIBUSB_ERROR_IO:        psz = " - IO error"; break;
					case LIBUSB_ERROR_NOT_FOUND: psz = " - Endpoint doesn't seem to exist."; break;
					case LIBUSB_ERROR_NO_DEVICE: psz = " - Was it disconnected?"; break;
					}
					Interface::Print("%s\n", psz);
					return false;
				}
			}
		}
		Interface::Print("Done clearing halts.\n");
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 89
	Interface::Print("CLEAR_COMM_FEATURE 1 . . . ");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_OUT // host-to-device
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0x21);

		uint8_t bRequest = 0x04; // CLEAR_COMM_FEATURE
		uint16_t wValue = 0x0001; // wFeatureSelector ???
		uint16_t wIndex = 0; // ????
		unsigned char *data = 0;
		uint16_t length = 0;
		unsigned int timeout = 0; // wait forever
		int rc = libusb_control_transfer(
			deviceHandle, bmRequestType, bRequest, wValue, wIndex, data, length, timeout);
		LogControlTransferResult(rc);
		if (!(0 <= rc || rc == LIBUSB_ERROR_PIPE))
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 91
	Interface::Print("GET_COMM_FEATURE . . . ");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_IN // device-to-host
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0xa1);

		uint8_t bRequest = 0x03; // GET_COMM_FEATURE
		uint16_t wValue = 0x0001; // wFeatureSelector ???
		uint16_t wIndex = 0; // ????
		unsigned char data[2] = { 0x00, 0x02 };
		uint16_t length = 2;
		unsigned int timeout = 0; // wait forever
		int rc = libusb_control_transfer(
			deviceHandle, bmRequestType, bRequest, wValue, wIndex, data, length, timeout);
		LogControlTransferResult(rc);
		if (!(0 <= rc || rc == LIBUSB_ERROR_PIPE))
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 93
	Interface::Print("SET_COMM_FEATURE . . .");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_OUT // host-to-device
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0x21);

		uint8_t bRequest = 0x02; // SET_COMM_FEATURE
		uint16_t wValue = 0x0001; // wFeatureSelector ???
		uint16_t wIndex = 0; // ????
		unsigned char data[2] = { 0x02, 0x00 };
		uint16_t length = 2;
		unsigned int timeout = 0; // wait forever
		int rc = libusb_control_transfer(
			deviceHandle, bmRequestType, bRequest, wValue, wIndex, data, length, timeout);
		LogControlTransferResult(rc);
		if (!(0 <= rc || rc == LIBUSB_ERROR_PIPE))
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 95
	Interface::Print("SET_CONTROL_LINE_STATE . . .");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_OUT // host-to-device
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0x21);

		uint8_t bRequest = 34; // SET_CONTROL_LINE_STATE
		uint16_t wValue = 0x0003; // wFeatureSelector ???
		uint16_t wIndex = 0; // ????
		unsigned char * data = 0;
		uint16_t length = 0;
		unsigned int timeout = 0; // wait forever
		int rc = libusb_control_transfer(
			deviceHandle, bmRequestType, bRequest, wValue, wIndex, data, length, timeout);
		LogControlTransferResult(rc);
		if (!(0 <= rc || rc == LIBUSB_ERROR_PIPE))
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 97
	Interface::Print("GET_LINE_CODING . . .");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_IN // device-to-host
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0xa1);

		uint8_t bRequest = 0x21; // GET_LINE_CODING
		uint16_t wValue = 0x0000; // wFeatureSelector ???
		uint16_t wIndex = 0; // ????
		unsigned char data[7] = { 0 };
		uint16_t length = 7;
		unsigned int timeout = 0; // wait forever
		int rc = libusb_control_transfer(
			deviceHandle, bmRequestType, bRequest, wValue, wIndex, data, length, timeout);
		LogControlTransferResult(rc);
		if (!(0 <= rc || rc == LIBUSB_ERROR_PIPE))
			return false;
	}

//? TODO is this optional?
//	Interface::Print("Setting up interface...\n");
//	result = libusb_set_interface_alt_setting(deviceHandle, bInterfaceNumber_data, bAlternateSetting_data);
//	if (result != LIBUSB_SUCCESS)
//	{
//		Interface::PrintError("Setting up interface failed!\n");
//		return (BridgeManager::kInitialiseFailed);
//	}

//	Interface::Print("\n");

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 98
	//	Ensure we're reading from the data_in endpoint.
	bWantOutstanding_bulk_in = true;
	StartAsyncTransfers();

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 100
	Interface::Print("GET_LINE_CODING . . . ");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_IN // device-to-host
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0xa1);

		uint8_t bRequest = 0x21; // GET_LINE_CODING
		uint16_t wValue = 0x0000; // wFeatureSelector ???
		uint16_t wIndex = 0; // ????

		bool ok = SyncTransfer_Control(
			bmRequestType, bRequest, wValue, wIndex,
			0, 0, // length, data
			true ); // bool pipe_error_ok
		if (!ok)
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 102
	Interface::Print("INTERRUPT . . . ");
	{
		//	Ensure we're listening for interrupts on comm
		bWantOutstanding_intr_comm = true;
		StartAsyncTransfers();
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 103
	Interface::Print("sync control request 32 . . . ");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_OUT // host-to-device
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0x21);

		uint8_t bRequest = 32; // ???
		uint16_t wValue = 0x0000; // ????
		uint16_t wIndex = 0; // ????
		unsigned char data[7] = { 0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x00 }; //????
		uint16_t length = sizeof(data);

		bool ok = SyncTransfer_Control(
			bmRequestType, bRequest, wValue, wIndex, length, data,
			true ); // bool pipe_error_ok
		if (!ok)
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 105
	Interface::Print("sync control request 34 0x0003 . . . ");
	{
		uint8_t bmRequestType = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE;
		uint8_t bRequest = 34; // ???
		uint16_t wValue = 0x0003; // ???
		uint16_t wIndex = 0; // ????

		bool ok = SyncTransfer_Control(
			bmRequestType, bRequest, wValue, wIndex,
			0, 0, // length, data
			true ); // bool pipe_error_ok
		if (!ok)
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 105
	Interface::Print("sync control request 34 0x0002 . . . ");
	{
		uint8_t bmRequestType = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE;
		uint8_t bRequest = 34; // ???
		uint16_t wValue = 0x0002; // ???
		uint16_t wIndex = 0; // ????

		bool ok = SyncTransfer_Control(
			bmRequestType, bRequest, wValue, wIndex,
			0, 0, // length, data
			true ); // bool pipe_error_ok
		if (!ok)
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 109 (almost the same as 103)
	Interface::Print("sync control request 32 . . . ");
	{
		uint8_t bmRequestType =
			  LIBUSB_ENDPOINT_OUT // host-to-device
			| LIBUSB_REQUEST_TYPE_CLASS
			| LIBUSB_RECIPIENT_INTERFACE;
		assert(bmRequestType == 0x21);

		uint8_t bRequest = 32; // ???
		uint16_t wValue = 0x0000; // ????
		uint16_t wIndex = 0; // ????
		unsigned char data[7] = { 0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08 }; //????
		uint16_t length = sizeof(data);

		bool ok = SyncTransfer_Control(
			bmRequestType, bRequest, wValue, wIndex, length, data,
			true ); // bool pipe_error_ok
		if (!ok)
			return false;
	}

	//	odin3_1.85_win7_vm_recoveryflash.pcap frame 110 - 111 shows a 536 ms pause
	for (unsigned n = 0; n < 500; ++n)
	{
		timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 0;
		int rc = libusb_handle_events_timeout(libusbContext, &tv);
		if (rc)
		{
			Interface::Print("handle events: ");
			LogLibusbResult(rc);
			Interface::Print("\n");
		}
		::usleep(1000);
	}

	return true;
}

#else // of if GTP7510
#endif // of else of if GTP7510

bool BridgeManager::CheckProtocol(void)
{
	Interface::Print("Checking if protocol is initialised...\n");

#if GTP7510

#if 1

	//? TODO get resumption to actually work
	Interface::Print("... no not really.\n");
	return false;

#elif 0

	if (!ResetInterface())
		return false;

	SetupSessionPacket deviceInfoPacket(SetupSessionPacket::kDeviceInfo);

	if (!SendPacket(&deviceInfoPacket, 3000, false))
	{
		Interface::Print("Protocol is not initialised.\n");
		return (false);
	}

	SetupSessionResponse deviceInfoResponse;

	if (!ReceivePacket(&deviceInfoResponse, 3000, false))
	{
		Interface::Print("Protocol is not initialised.\n");
		return (false);
	}

#elif 0

	//? this is a lame experiment
	{
		Interface::Print("Rebooting device...\n");

		EndSessionPacket *rebootDevicePacket = new EndSessionPacket(EndSessionPacket::kRequestRebootDevice);
		bool success = SendPacket(rebootDevicePacket);
		delete rebootDevicePacket;

		if (!success)
		{
			Interface::PrintError("Failed to send reboot device packet!\n");

			return (false);
		}

		ResponsePacket *rebootDeviceResponse = new ResponsePacket(ResponsePacket::kResponseTypeEndSession);
		success = ReceivePacket(rebootDeviceResponse);
		delete rebootDeviceResponse;

		if (!success)
		{
			Interface::PrintError("Failed to receive reboot confirmation!\n");

			return (false);
		}
		else
		{
			//	Sleep for a while, just polling the device while it reboots.
			pps->ClearReceivedData();
			unsigned char dataBuffer[1];
			pps->ReceiveData(dataBuffer, 1, 1, 10*1000);
			pps->ClearReceivedData();
		}
	}
#elif 0
#endif

#else // of if GTP7510

	SetupSessionPacket deviceInfoPacket(SetupSessionPacket::kDeviceInfo);

	if (!SendPacket(&deviceInfoPacket, 100, false))
	{
		Interface::Print("Protocol is not initialised.\n");
		return (false);
	}

	SetupSessionResponse deviceInfoResponse;

	if (!ReceivePacket(&deviceInfoResponse, 100, false))
	{
		Interface::Print("Protocol is not initialised.\n");
		return (false);
	}

#endif // of else of if GTP7510

	Interface::Print("Protocol is initialised.\n");
	return (true);
}

#if GTP7510

bool BridgeManager::SyncTransfer_Control(
	uint8_t bmRequestType,
	uint8_t bRequest,
	uint16_t wValue,
	uint16_t wIndex,
	unsigned length, unsigned char * data_in,
	bool pipe_error_ok )
{
	unsigned char * data = 0;
	if (length)
	{
		data = new unsigned char [length];
		if (data_in)
			memcpy(data, data_in, length);
		else
			memset(data, 0, length);
	}

	unsigned int timeout = 0; // wait forever
	int rc = libusb_control_transfer(
		deviceHandle, bmRequestType, bRequest, wValue, wIndex, data, length, timeout);
	LogControlTransferResult(rc);
	delete data;
	if (!(0 <= rc || pipe_error_ok && rc == LIBUSB_ERROR_PIPE))
		return false;

	return true;
}

extern "C" void ExtC_OnAsyncTransferComplete_Bulk_In(libusb_transfer * transfer)
{
	BridgeManager * pbm = static_cast<BridgeManager *>(transfer->user_data);
	transfer->user_data = 0;

	pbm->OnAsyncTransferComplete_Bulk_In(transfer);
}

void BridgeManager::OnAsyncTransferComplete_Bulk_In(libusb_transfer * transfer)
{
	//Interface::Print("OnAsyncTransferComplete_Bulk_In received %d bytes\n", transfer->actual_length);

	//	Append the data.
	assert(transfer->actual_length <= buffer_bulk_in_z - buffer_bulk_in_e);
	buffer_bulk_in_e += transfer->actual_length;

	//	libusb should free this for us
	activeTransfer_bulk_in = 0;

	//	Restart the transfer.
	StartAsyncTransfers();
}

extern "C" void ExtC_OnAsyncTransferComplete_Intr_Comm(libusb_transfer * transfer)
{
	BridgeManager * pbm = static_cast<BridgeManager *>(transfer->user_data);
	transfer->user_data = 0;

	pbm->OnAsyncTransferComplete_Intr_Comm(transfer);
}

void BridgeManager::OnAsyncTransferComplete_Intr_Comm(libusb_transfer * transfer)
{
	//Interface::Print("OnAsyncTransferComplete_Intr_Comm!\n");

	// What to do here?
	assert(transfer->actual_length == 0);

	//	libusb should free this for us
	activeTransfer_intr_comm = 0;

	//	Restart the transfer.
	StartAsyncTransfers();
}

int BridgeManager::GetCntBytesAvail_bulk_in()
{
	int cb = 0;

	if (buffer_bulk_in_b)
		cb  = buffer_bulk_in_e - buffer_bulk_in_c;

	return cb;
}

void BridgeManager::StartAsyncTransfers()
{
	for (unsigned b_not_i = 0; b_not_i < 2; ++b_not_i)
	{
		bool                want          = b_not_i ? bWantOutstanding_bulk_in : bWantOutstanding_intr_comm;
		libusb_transfer * & this_transfer = b_not_i ? activeTransfer_bulk_in   : activeTransfer_intr_comm;

		if (want && !this_transfer)
		{
			unsigned char endpoint =
				  b_not_i
				? bEndpointAddress_data_in // 0x81
				: bEndpointAddress_comm;   // 0x82?
			int length = b_not_i ? 4096 : 0;

			//Interface::Print(
			//	"%s (%d bytes) %s . . . ",
			//	b_not_i ? "BULK" : "INTERRUPT",
			//	length,
			//	((endpoint >> 7) & 1) ? "in" : "out" );

			uint8_t * * const buffer_b = b_not_i ? &buffer_bulk_in_b : 0;
			uint8_t * * const buffer_c = b_not_i ? &buffer_bulk_in_c : 0;
			uint8_t * * const buffer_e = b_not_i ? &buffer_bulk_in_e : 0;
			uint8_t * * const buffer_z = b_not_i ? &buffer_bulk_in_z : 0;

			//? TODO find out the max ODIN packet size.
			static int const cnt_bytes_max_packet = 32*1024;
			if (length)
			{
				if (!*buffer_b)
				{
					*buffer_b = new uint8_t[cnt_bytes_max_packet*2];
					*buffer_c = *buffer_b;
					*buffer_e = *buffer_b;
					*buffer_z = *buffer_b + length;
				}

				//	Move unconsumed data to beginning of buffer.
				if (*buffer_c < *buffer_e && *buffer_b < *buffer_c)
				{
					size_t cnt_move = *buffer_e - *buffer_c;
					memmove(*buffer_b, *buffer_c, cnt_move);
					*buffer_c = *buffer_b;
					*buffer_e = *buffer_b + cnt_move;
				}

				//	Possibly grow buffer.
				if (*buffer_z - *buffer_e < length)
				{
					//	Double the size of the allocation.
					size_t cnt_copy = *buffer_e - *buffer_c;
					size_t cnt_alloc = cnt_copy + length + cnt_bytes_max_packet*2;
					uint8_t * b2 = new uint8_t[cnt_alloc];
					if (cnt_copy)
						memcpy(b2, *buffer_c, cnt_copy);
					delete *buffer_b;
					*buffer_b = b2;
					*buffer_c = b2;
					*buffer_e = b2 + cnt_copy;
					*buffer_z = b2 + cnt_alloc;
				}
			}

			libusb_transfer * transfer = libusb_alloc_transfer( // libusb_transfer *
				0 ); // int iso_packets
			if (!transfer) {
				Interface::Print("Error: unable to alloc libusb transfer\n");
				return;
			}

			libusb_transfer_cb_fn callback =
				  b_not_i
				? ExtC_OnAsyncTransferComplete_Bulk_In
				: ExtC_OnAsyncTransferComplete_Intr_Comm;

			uint8_t * buffer = length ? *buffer_e : 0;

			(b_not_i ? libusb_fill_bulk_transfer : libusb_fill_interrupt_transfer)( // void
				transfer,      // struct libusb_transfer * transfer
				deviceHandle,  // libusb_device_handle * dev_handle
				endpoint,      // unsigned char endpoint
				buffer,        // unsigned char * buffer
				length,        // int length
				callback,      // libusb_transfer_cb_fn callback
				this,          // void * user_data
				0 );           // unsigned int timeout
			transfer->flags |= LIBUSB_TRANSFER_FREE_TRANSFER;

			//Interface::Print("Submitting . . . ");
			int rc = libusb_submit_transfer(transfer);
			if (!(0 == rc))
			{
				LogLibusbResult(rc);
				libusb_free_transfer(transfer);
			}
			else
			{
				//Interface::Print("OK\n");
				this_transfer = transfer;
			}
		}
	}
}

#else // of if GTP7510
#endif // of else of if GTP7510

bool BridgeManager::InitialiseProtocol(void)
{
#if GTP7510

	Interface::Print("Initialising protocol.\n");

	if (!ResetInterface())
		return false;

	uint8_t * dataBuffer = new uint8_t[7];

	Interface::Print("Handshaking with Loke...\n");

	int dataTransferred;

	// Send "ODIN"
	strcpy((char *)dataBuffer, "ODIN");

	int result = libusb_bulk_transfer(deviceHandle, bEndpointAddress_data_out, dataBuffer, 4, &dataTransferred, 1000);
	if (result < 0)
	{
		if (verbose)
			Interface::PrintError("Failed to send data: \"%s\"\n", dataBuffer);
		else
			Interface::PrintError("Failed to send data!");

		delete [] dataBuffer;
		return (false);
	}

	if (dataTransferred != 4)
	{
		if (verbose)
			Interface::PrintError("Failed to complete sending of data: \"%s\"\n", dataBuffer);
		else
			Interface::PrintError("Failed to complete sending of data!");

		delete [] dataBuffer;
		return (false);
	}

	// Expect "LOKE"

	memset(dataBuffer, 0, 7);

	dataTransferred = ReceiveData(dataBuffer, 4, 4, 3000);

//	result = libusb_bulk_transfer(deviceHandle, bEndpointAddress_data_in, dataBuffer, 7, &dataTransferred, 1000);
//	if (result < 0)
//	{
//		Interface::PrintError("Failed to receive response!\n");
//
//		delete [] dataBuffer;
//		return (false);
//	}

	if (dataTransferred != 4 || memcmp(dataBuffer, "LOKE", 4) != 0)
	{
		Interface::PrintError("Unexpected communication!\n");

		if (verbose)
			Interface::PrintError("Expected: \"%s\"\nReceived: \"%s\"\n", "LOKE", dataBuffer);

		Interface::PrintError("Handshake failed!\n");

		delete [] dataBuffer;
		return (false);
	}

	return (true);
#else // of if GTP7510
	Interface::Print("Initialising protocol...\n");

	unsigned char *dataBuffer = new unsigned char[7];

	int result = libusb_control_transfer(deviceHandle, LIBUSB_REQUEST_TYPE_CLASS, 0x22, 0x3, 0, nullptr, 0, 1000);

	if (result < 0)
	{
		Interface::PrintError("Failed to initialise protocol!\n");

		delete [] dataBuffer;
		return (false);
	}

	memset(dataBuffer, 0, 7);
	dataBuffer[1] = 0xC2;
	dataBuffer[2] = 0x01;
	dataBuffer[6] = 0x07;

	result = libusb_control_transfer(deviceHandle, LIBUSB_REQUEST_TYPE_CLASS, 0x20, 0x0, 0, dataBuffer, 7, 1000);
	if (result < 0)
	{
		Interface::PrintError("Failed to initialise protocol!\n");

		delete [] dataBuffer;
		return (false);
	}

	result = libusb_control_transfer(deviceHandle, LIBUSB_REQUEST_TYPE_CLASS, 0x22, 0x3, 0, nullptr, 0, 1000);
	if (result < 0)
	{
		Interface::PrintError("Failed to initialise protocol!\n");

		delete [] dataBuffer;
		return (false);
	}

	result = libusb_control_transfer(deviceHandle, LIBUSB_REQUEST_TYPE_CLASS, 0x22, 0x2, 0, nullptr, 0, 1000);
	if (result < 0)
	{
		Interface::PrintError("Failed to initialise protocol!\n");

		delete [] dataBuffer;
		return (false);
	}

	memset(dataBuffer, 0, 7);
	dataBuffer[1] = 0xC2;
	dataBuffer[2] = 0x01;
	dataBuffer[6] = 0x08;

	result = libusb_control_transfer(deviceHandle, LIBUSB_REQUEST_TYPE_CLASS, 0x20, 0x0, 0, dataBuffer, 7, 1000);
	if (result < 0)
	{
		Interface::PrintError("Failed to initialise protocol!\n");

		delete [] dataBuffer;
		return (false);
	}

	result = libusb_control_transfer(deviceHandle, LIBUSB_REQUEST_TYPE_CLASS, 0x22, 0x2, 0, nullptr, 0, 1000);
	if (result < 0)
	{
		Interface::PrintError("Failed to initialise protocol!\n");

		delete [] dataBuffer;
		return (false);
	}

	Interface::Print("Handshaking with Loke...\n");

	int dataTransferred;

	// Send "ODIN"
	strcpy((char *)dataBuffer, "ODIN");

	result = libusb_bulk_transfer(deviceHandle, outEndpoint, dataBuffer, 4, &dataTransferred, 1000);
	if (result < 0)
	{
		if (verbose)
			Interface::PrintError("Failed to send data: \"%s\"\n", dataBuffer);
		else
			Interface::PrintError("Failed to send data!");

		delete [] dataBuffer;
		return (false);
	}

	if (dataTransferred != 4)
	{
		if (verbose)
			Interface::PrintError("Failed to complete sending of data: \"%s\"\n", dataBuffer);
		else
			Interface::PrintError("Failed to complete sending of data!");

		delete [] dataBuffer;
		return (false);
	}

	// Expect "LOKE"
	memset(dataBuffer, 0, 7);

	result = libusb_bulk_transfer(deviceHandle, inEndpoint, dataBuffer, 7, &dataTransferred, 1000);
	if (result < 0)
	{
		Interface::PrintError("Failed to receive response!\n");

		delete [] dataBuffer;
		return (false);;
	}

	if (dataTransferred != 4 || memcmp(dataBuffer, "LOKE", 4) != 0)
	{
		Interface::PrintError("Unexpected communication!\n");

		if (verbose)
			Interface::PrintError("Expected: \"%s\"\nReceived: \"%s\"\n", "LOKE", dataBuffer);

		Interface::PrintError("Handshake failed!\n");

		delete [] dataBuffer;
		return (false);
	}

	return (true);
#endif // of else of if GTP7510
}

BridgeManager::BridgeManager(bool verbose, int communicationDelay)
{
	this->verbose = verbose;
	this->communicationDelay = communicationDelay;

	libusbContext = nullptr;
	deviceHandle = nullptr;
	heimdallDevice = nullptr;

#if GTP7510

	bInterfaceNumber_comm = -1;
	bAlternateSetting_comm = -1;
	bEndpointAddress_comm = -1;

	bInterfaceNumber_data = -1;
	bAlternateSetting_data = -1;
	bEndpointAddress_data_in = -1;
	bEndpointAddress_data_out = -1;

	bWantOutstanding_bulk_in = false;
	activeTransfer_bulk_in = 0;
	buffer_bulk_in_b = 0;
	buffer_bulk_in_c = 0;
	buffer_bulk_in_e = 0;
	buffer_bulk_in_z = 0;
	bWantOutstanding_intr_comm = false;
	activeTransfer_intr_comm = 0;

#else // of if GTP7510

	inEndpoint = -1;
	outEndpoint = -1;
	interfaceIndex = -1;

#endif // of else of if GTP7510

#ifdef OS_LINUX

	detachedDriver = false;

#endif
}

BridgeManager::~BridgeManager()
{
#if GTP7510

	if (activeTransfer_bulk_in)
		libusb_free_transfer(activeTransfer_bulk_in);

	if (activeTransfer_intr_comm)
		libusb_free_transfer(activeTransfer_intr_comm);

	delete buffer_bulk_in_b;

	if (bInterfaceNumber_data >= 0)
		libusb_release_interface(deviceHandle, bInterfaceNumber_data);

	if (bInterfaceNumber_comm >= 0)
		libusb_release_interface(deviceHandle, bInterfaceNumber_comm);

#ifdef OS_LINUX

	if (detachedDriver)
	{
		Interface::Print("Re-attaching kernel driver...\n");

		if (bInterfaceNumber_data >= 0)
			libusb_attach_kernel_driver(deviceHandle, bInterfaceNumber_data);

		if (bInterfaceNumber_comm >= 0)
			libusb_attach_kernel_driver(deviceHandle, bInterfaceNumber_comm);
	}

#endif

#else // of if GTP7510
#endif // of else of if GTP7510

	if (deviceHandle)
		libusb_close(deviceHandle);

	if (heimdallDevice)
		libusb_unref_device(heimdallDevice);

	if (libusbContext)
		libusb_exit(libusbContext);
}

bool BridgeManager::DetectDevice(void)
{
	// Initialise libusb-1.0
	int result = libusb_init(&libusbContext);
	if (result != LIBUSB_SUCCESS)
	{
		Interface::PrintError("Failed to initialise libusb. libusb error: %d\n", result);
		return (false);
	}

	// Get handle to Galaxy S device
	struct libusb_device **devices;
	int deviceCount = libusb_get_device_list(libusbContext, &devices);

	for (int deviceIndex = 0; deviceIndex < deviceCount; deviceIndex++)
	{
		libusb_device_descriptor descriptor;
		libusb_get_device_descriptor(devices[deviceIndex], &descriptor);

		for (int i = 0; i < BridgeManager::kSupportedDeviceCount; i++)
		{
			if (descriptor.idVendor == supportedDevices[i].vendorId && descriptor.idProduct == supportedDevices[i].productId)
			{
				libusb_free_device_list(devices, deviceCount);

				Interface::Print("Device detected\n");
				return (true);
			}
		}
	}

	libusb_free_device_list(devices, deviceCount);

	Interface::PrintDeviceDetectionFailed();
	return (false);
}

int BridgeManager::Initialise(void)
{
#if GTP7510

	int result = 0;

	Interface::Print("Initialising libusb...\n");
	{
		// Initialise libusb-1.0
		result = libusb_init(&libusbContext);
		if (result != LIBUSB_SUCCESS)
		{
			Interface::PrintError("Failed to initialise libusb. libusb error: %d\n", result);
			Interface::Print("Failed to connect to device!");
			return (BridgeManager::kInitialiseFailed);
		}

		int libusbDebugLevel = 3;
		libusb_set_debug(libusbContext, libusbDebugLevel);
	}

	// Get handle to Galaxy S device
	{
		Interface::Print("Detecting device . . . ");

		struct libusb_device **devices;
		int deviceCount = libusb_get_device_list(libusbContext, &devices);

		for (int deviceIndex = 0; deviceIndex < deviceCount; deviceIndex++)
		{
			libusb_device_descriptor descriptor;
			libusb_get_device_descriptor(devices[deviceIndex], &descriptor);

			for (int i = 0; i < BridgeManager::kSupportedDeviceCount; i++)
			{
				if (    descriptor.idVendor == supportedDevices[i].vendorId
				     && descriptor.idProduct == supportedDevices[i].productId )
				{
					heimdallDevice = devices[deviceIndex];
					libusb_ref_device(heimdallDevice);
					break;
				}
			}

			if (heimdallDevice)
				break;
		}

		libusb_free_device_list(devices, deviceCount);

		if (!heimdallDevice)
		{
			Interface::PrintDeviceDetectionFailed();
			return (BridgeManager::kInitialiseDeviceNotDetected);
		}

		Interface::Print("OK\n");
	}

	{
		Interface::Print("Opening device . . . ");
		result = libusb_open(heimdallDevice, &deviceHandle);
		if (result != LIBUSB_SUCCESS)
		{
			Interface::PrintError("Failed to access device. libusb error: %d\n", result);
			return (BridgeManager::kInitialiseFailed);
		}
		Interface::Print("OK\n");
	}

	{
		Interface::Print("Resetting device . . . ");
		result = libusb_reset_device(deviceHandle);
		if (result != LIBUSB_SUCCESS)
		{
			LogLibusbResult(result);
			Interface::PrintError("\n");
			return (BridgeManager::kInitialiseFailed);
		}
		Interface::Print("OK\n");
	}

	libusb_device_descriptor deviceDescriptor;
	{
		Interface::Print("Requesting device description . . . ");
		result = libusb_get_device_descriptor(heimdallDevice, &deviceDescriptor);
		if (result != LIBUSB_SUCCESS)
		{
			LogLibusbResult(result);
			Interface::PrintError(" - Failed to retrieve device description\n");
			return (BridgeManager::kInitialiseFailed);
		}
		Interface::Print("OK\n");
	}

	if (verbose)
	{
		uint8_t stringBuffer[128];

		if (libusb_get_string_descriptor_ascii(deviceHandle, deviceDescriptor.iManufacturer,
			stringBuffer, 128) >= 0)
		{
			Interface::Print("      Manufacturer: \"%s\"\n", stringBuffer);
		}

		if (libusb_get_string_descriptor_ascii(deviceHandle, deviceDescriptor.iProduct,
			stringBuffer, 128) >= 0)
		{
			Interface::Print("           Product: \"%s\"\n", stringBuffer);
		}

		if (libusb_get_string_descriptor_ascii(deviceHandle, deviceDescriptor.iSerialNumber,
			stringBuffer, 128) >= 0)
		{
			Interface::Print("         Serial No: \"%s\"\n", stringBuffer);
		}

		Interface::Print("\n            length: %d\n", deviceDescriptor.bLength);
		Interface::Print("      device class: %d\n", deviceDescriptor.bDeviceClass);
		Interface::Print("               S/N: %d\n", deviceDescriptor.iSerialNumber);
		Interface::Print("           VID:PID: %04X:%04X\n", deviceDescriptor.idVendor, deviceDescriptor.idProduct);
		Interface::Print("         bcdDevice: %04X\n", deviceDescriptor.bcdDevice);
		Interface::Print("   iMan:iProd:iSer: %d:%d:%d\n", deviceDescriptor.iManufacturer, deviceDescriptor.iProduct,
			deviceDescriptor.iSerialNumber);
		Interface::Print("          nb confs: %d\n", deviceDescriptor.bNumConfigurations);
	}

	{
		Interface::Print("Requesting device config . . . ");

		libusb_config_descriptor *configDescriptor;
		result = libusb_get_config_descriptor(heimdallDevice, 0, &configDescriptor);
		if (result != LIBUSB_SUCCESS || !configDescriptor)
		{
			Interface::PrintError("Failed to retrieve config descriptor\n");
			return (BridgeManager::kInitialiseFailed);
		}
		Interface::Print("OK\n");

		if (verbose)
			Interface::Print("\n");

		assert(bInterfaceNumber_comm < 0);
		assert(bAlternateSetting_comm < 0);
		assert(bEndpointAddress_comm < 0);
		assert(bInterfaceNumber_data < 0);
		assert(bAlternateSetting_data < 0);
		assert(bEndpointAddress_data_in < 0);
		assert(bEndpointAddress_data_out < 0);

		Interface::Print("Examining device interfaces.\n");

		//	For each interface.
		for (int i = 0; i < configDescriptor->bNumInterfaces; i++)
		{
			int num_alts = configDescriptor->usb_interface[i].num_altsetting;

			if (verbose)
				Interface::Print("\nInterface at index %d has %d alt settings.", i, num_alts);

			if (num_alts != 1)
				Interface::Print("\nWarning: was expecting just 1 alt setting, interface %d has %d.", i, num_alts);

			//	For each altsetting on interface i.
			for (int j = 0; j < num_alts; j++)
			{
				uint8_t bInterfaceNumber = configDescriptor->usb_interface[i].altsetting[j].bInterfaceNumber;
				uint8_t bAlternateSetting = configDescriptor->usb_interface[i].altsetting[j].bAlternateSetting;
				uint8_t bNumEndpoints = configDescriptor->usb_interface[i].altsetting[j].bNumEndpoints;

				if (verbose)
				{
					Interface::Print("\n    interface[%d].altsetting[%d].bInterfaceNumber = %d", i, j, bInterfaceNumber);
					Interface::Print("\n    interface[%d].altsetting[%d].bAlternateSetting = %d", i, j, bAlternateSetting);
					Interface::Print("\n    interface[%d].altsetting[%d].bNumEndpoints = %d", i, j, bNumEndpoints);
					Interface::Print("\n        Class.SubClass.Protocol: %02X.%02X.%02X\n",
						configDescriptor->usb_interface[i].altsetting[j].bInterfaceClass,
						configDescriptor->usb_interface[i].altsetting[j].bInterfaceSubClass,
						configDescriptor->usb_interface[i].altsetting[j].bInterfaceProtocol);
				}

				int endp_comm = -1;
				int endp_data_in = -1;
				int endp_data_out = -1;

				//	For each endpoint on interface i.
				for (int k = 0; k < configDescriptor->usb_interface[i].altsetting[j].bNumEndpoints; k++)
				{
					const libusb_endpoint_descriptor *endpointDescriptor = &configDescriptor->usb_interface[i].altsetting[j].endpoint[k];

					if (verbose)
					{
						Interface::Print("           endpoint[%d].address: %02X\n", k, endpointDescriptor->bEndpointAddress);
						Interface::Print("               max packet size: %04X\n", endpointDescriptor->wMaxPacketSize);
						Interface::Print("              polling interval: %02X\n", endpointDescriptor->bInterval);
					}

					//	"An Abstract Control Model Communications Device uses a Communications Class interface
					//	for device management. With a Communications Subclass code of Abstract Control."
					//		interface class code 0x02 is "Communications Interface Class"
					//		communications interface class subclass code 0x02 is "Abstract Control Model"
					//		communications interface class control protocol code 0x01 is "ITU-T V.250 AT Commands: V.250 etc"
					if (    configDescriptor->usb_interface[i].altsetting[j].bInterfaceClass == LIBUSB_CLASS_COMM // 0x02
						 && configDescriptor->usb_interface[i].altsetting[j].bInterfaceSubClass == 0x02
						 && configDescriptor->usb_interface[i].altsetting[j].bInterfaceProtocol == 0x01 )
					{
						//? Should we check the endpoint descriptor bmAttributes?

						//	..00.... Usage type: data endpoint
						//	....00.. Synchronization type: no synchronization
						//	......11 Transfer type: interrupt
						uint8_t const bmAttributes_expected = 0x03;
						if (endpointDescriptor->bmAttributes == bmAttributes_expected)
						{
							if (0 <= endp_comm)
								Interface::Print("Warning: Multiple comm endpoints on the same altsetting?!\n");

							endp_comm = endpointDescriptor->bEndpointAddress;
						}
						else
							Interface::Print("Warning: Ignoring unexpected comm endpoint.\n");
					}
					//		interface class code 0x0A is "Data Interface Class"
					//		communications interface class subclass code 0x00 is unused
					//		communications interface class control protocol code 0x00 means
					//			"the device does not use a class-specific protocol on this interface."
					//			However, it may use class-specific protocols on an interface basis."
					else if (
						   configDescriptor->usb_interface[i].altsetting[j].bInterfaceClass == LIBUSB_CLASS_DATA // 0x0A
						//&& configDescriptor->usb_interface[i].altsetting[j].bInterfaceSubClass == 0x00 // technically unused
						&& configDescriptor->usb_interface[i].altsetting[j].bInterfaceProtocol == 0x00 )
					{
						//	..00.... Usage type: data endpoint
						//	....00.. Synchronization type: no synchronization
						//	......10 Transfer type: bulk
						uint8_t const bmAttributes_expected = 0x02;

						if (endpointDescriptor->bEndpointAddress & LIBUSB_ENDPOINT_IN)
						{
							if (endpointDescriptor->bmAttributes == bmAttributes_expected)
							{
								if (0 <= endp_data_in)
									Interface::Print("Warning: Multiple data [in] endpoints on the same interface altsetting?!\n");

								endp_data_in = endpointDescriptor->bEndpointAddress;
							}
							else
								Interface::Print("Warning: Ignoring unexpected data [in] endpoint.\n");
						}
						else // an [out] endpoint
						{
							if (endpointDescriptor->bmAttributes == bmAttributes_expected)
							{
								if (0 <= endp_data_out)
									Interface::Print("Warning: Multiple data [out] endpoints on the same interface altsetting?!\n");

								endp_data_out = endpointDescriptor->bEndpointAddress;
							}
							else
								Interface::Print("Warning: Ignoring unexpected data [out] endpoint.\n");
						}
					}
				}

				//	If we haven't already chosen a comm interface and altSetting, use this if it looks acceptable.
				if (bEndpointAddress_comm < 0 && 0 <= endp_comm )
				{
					bInterfaceNumber_comm = bInterfaceNumber;
					bAlternateSetting_comm = bAlternateSetting;
					bEndpointAddress_comm = endp_comm;
				}

				//	If we haven't already chosen a data interface and altSetting, use this if it looks acceptable.
				if (   bInterfaceNumber_data < 0 && bEndpointAddress_data_in < 0 && bEndpointAddress_data_out < 0
					&& 0 <= endp_data_in && 0 <= endp_data_out )
				{
					bInterfaceNumber_data = bInterfaceNumber;
					bAlternateSetting_data = bAlternateSetting;
					bEndpointAddress_data_in = endp_data_in;
					bEndpointAddress_data_out = endp_data_out;
				}
			}
		}

		Interface::Print("\nDone examining device interfaces.\n");

		libusb_free_config_descriptor(configDescriptor);
	}

	//	Claim the comm and data interfaces.
	{
		int const intfIxs[] = { bInterfaceNumber_comm, bInterfaceNumber_data };
		for (int intfIxsIx= 0; intfIxsIx < sizeof(intfIxs)/sizeof(intfIxs[0]); ++intfIxsIx)
		{
			int intfIx = intfIxs[intfIxsIx];

			Interface::Print("Claiming interface index %d . . . ", intfIx);
			result = libusb_claim_interface(deviceHandle, intfIx);

#ifdef OS_LINUX

			if (result != LIBUSB_SUCCESS) // LIBUSB_ERROR_BUSY seen in practice
			{
				LogLibusbResult(result);
				Interface::Print("\n");

				Interface::Print("Detaching kernel driver . . . ");
				libusb_detach_kernel_driver(deviceHandle, intfIx);
				detachedDriver = true;

				Interface::Print("OK\nClaiming interface again . . .");
				result = libusb_claim_interface(deviceHandle, intfIx);
			}

#endif

			if (result != LIBUSB_SUCCESS)
			{
				LogLibusbResult(result);
				Interface::PrintError("\n");
				return (BridgeManager::kInitialiseFailed);
			}

			Interface::Print("OK\n");
		}
	}

	Interface::PrintError("Checking protocol...\n");
	if (!CheckProtocol())
	{
		Interface::PrintError("... protocol not already initialized.\n");

		if (!InitialiseProtocol())
			return (BridgeManager::kInitialiseFailed);
	}

	Interface::Print("\n");

	return (BridgeManager::kInitialiseSucceeded);


#else // of if GTP7510
	Interface::Print("Initialising connection...\n");

	// Initialise libusb-1.0
	int result = libusb_init(&libusbContext);
	if (result != LIBUSB_SUCCESS)
	{
		Interface::PrintError("Failed to initialise libusb. libusb error: %d\n", result);
		Interface::Print("Failed to connect to device!");
		return (BridgeManager::kInitialiseFailed);
	}

	Interface::Print("Detecting device...\n");

	// Get handle to Galaxy S device
	struct libusb_device **devices;
	int deviceCount = libusb_get_device_list(libusbContext, &devices);

	for (int deviceIndex = 0; deviceIndex < deviceCount; deviceIndex++)
	{
		libusb_device_descriptor descriptor;
		libusb_get_device_descriptor(devices[deviceIndex], &descriptor);

		for (int i = 0; i < BridgeManager::kSupportedDeviceCount; i++)
		{
			if (descriptor.idVendor == supportedDevices[i].vendorId && descriptor.idProduct == supportedDevices[i].productId)
			{
				heimdallDevice = devices[deviceIndex];
				libusb_ref_device(heimdallDevice);
				break;
			}
		}

		if (heimdallDevice)
			break;
	}

	libusb_free_device_list(devices, deviceCount);

	if (!heimdallDevice)
	{
		Interface::PrintDeviceDetectionFailed();
		return (BridgeManager::kInitialiseDeviceNotDetected);
	}

	result = libusb_open(heimdallDevice, &deviceHandle);
	if (result != LIBUSB_SUCCESS)
	{
		Interface::PrintError("Failed to access device. libusb error: %d\n", result);
		return (BridgeManager::kInitialiseFailed);
	}

	libusb_device_descriptor deviceDescriptor;
	result = libusb_get_device_descriptor(heimdallDevice, &deviceDescriptor);
	if (result != LIBUSB_SUCCESS)
	{
		Interface::PrintError("Failed to retrieve device description\n");
		return (BridgeManager::kInitialiseFailed);
	}

	if (verbose)
	{
		uint8_t stringBuffer[128];

		if (libusb_get_string_descriptor_ascii(deviceHandle, deviceDescriptor.iManufacturer,
			stringBuffer, 128) >= 0)
		{
			Interface::Print("      Manufacturer: \"%s\"\n", stringBuffer);
		}

		if (libusb_get_string_descriptor_ascii(deviceHandle, deviceDescriptor.iProduct,
			stringBuffer, 128) >= 0)
		{
			Interface::Print("           Product: \"%s\"\n", stringBuffer);
		}

		if (libusb_get_string_descriptor_ascii(deviceHandle, deviceDescriptor.iSerialNumber,
			stringBuffer, 128) >= 0)
		{
			Interface::Print("         Serial No: \"%s\"\n", stringBuffer);
		}

		Interface::Print("\n            length: %d\n", deviceDescriptor.bLength);
		Interface::Print("      device class: %d\n", deviceDescriptor.bDeviceClass);
		Interface::Print("               S/N: %d\n", deviceDescriptor.iSerialNumber);
		Interface::Print("           VID:PID: %04X:%04X\n", deviceDescriptor.idVendor, deviceDescriptor.idProduct);
		Interface::Print("         bcdDevice: %04X\n", deviceDescriptor.bcdDevice);
		Interface::Print("   iMan:iProd:iSer: %d:%d:%d\n", deviceDescriptor.iManufacturer, deviceDescriptor.iProduct,
			deviceDescriptor.iSerialNumber);
		Interface::Print("          nb confs: %d\n", deviceDescriptor.bNumConfigurations);
	}

	libusb_config_descriptor *configDescriptor;
	result = libusb_get_config_descriptor(heimdallDevice, 0, &configDescriptor);

	if (result != LIBUSB_SUCCESS || !configDescriptor)
	{
		Interface::PrintError("Failed to retrieve config descriptor\n");
		return (BridgeManager::kInitialiseFailed);
	}

	int interfaceIndex = -1;
	int altSettingIndex;

	for (int i = 0; i < configDescriptor->bNumInterfaces; i++)
	{
		for (int j = 0 ; j < configDescriptor->usb_interface[i].num_altsetting; j++)
		{
			if (verbose)
			{
				Interface::Print("\ninterface[%d].altsetting[%d]: num endpoints = %d\n",
					i, j, configDescriptor->usb_interface[i].altsetting[j].bNumEndpoints);
				Interface::Print("   Class.SubClass.Protocol: %02X.%02X.%02X\n",
					configDescriptor->usb_interface[i].altsetting[j].bInterfaceClass,
					configDescriptor->usb_interface[i].altsetting[j].bInterfaceSubClass,
					configDescriptor->usb_interface[i].altsetting[j].bInterfaceProtocol);
			}

			int inEndpointAddress = -1;
			int outEndpointAddress = -1;

			for (int k = 0; k < configDescriptor->usb_interface[i].altsetting[j].bNumEndpoints; k++)
			{
				const libusb_endpoint_descriptor *endpoint = &configDescriptor->usb_interface[i].altsetting[j].endpoint[k];

				if (verbose)
				{
					Interface::Print("       endpoint[%d].address: %02X\n", k, endpoint->bEndpointAddress);
					Interface::Print("           max packet size: %04X\n", endpoint->wMaxPacketSize);
					Interface::Print("          polling interval: %02X\n", endpoint->bInterval);
				}

				if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN)
					inEndpointAddress = endpoint->bEndpointAddress;
				else
					outEndpointAddress = endpoint->bEndpointAddress;
			}

			if (interfaceIndex < 0 && configDescriptor->usb_interface[i].altsetting[j].bNumEndpoints == 2
				&& configDescriptor->usb_interface[i].altsetting[j].bInterfaceClass == CLASS_CDC
				&& inEndpointAddress != -1 && outEndpointAddress != -1)
			{
				interfaceIndex = i;
				altSettingIndex = j;
				inEndpoint = inEndpointAddress;
				outEndpoint = outEndpointAddress;
			}
		}
	}

	libusb_free_config_descriptor(configDescriptor);

	if (result != LIBUSB_SUCCESS)
	{
		Interface::PrintError("Failed to find correct interface configuration\n");
		return (BridgeManager::kInitialiseFailed);
	}

	Interface::Print("Claiming interface...\n");
	result = libusb_claim_interface(deviceHandle, interfaceIndex);

#ifdef OS_LINUX

	if (result != LIBUSB_SUCCESS)
	{
		detachedDriver = true;
		Interface::Print("Attempt failed. Detaching driver...\n");
		libusb_detach_kernel_driver(deviceHandle, interfaceIndex);
		Interface::Print("Claiming interface again...\n");
		result = libusb_claim_interface(deviceHandle, interfaceIndex);
	}

#endif

	if (result != LIBUSB_SUCCESS)
	{
		Interface::PrintError("Claiming interface failed!\n");
		return (BridgeManager::kInitialiseFailed);
	}

	Interface::Print("Setting up interface...\n");
	result = libusb_set_interface_alt_setting(deviceHandle, interfaceIndex, altSettingIndex);

	if (result != LIBUSB_SUCCESS)
	{
		Interface::PrintError("Setting up interface failed!\n");
		return (BridgeManager::kInitialiseFailed);
	}

	Interface::Print("\n");

	if (!CheckProtocol())
	{
		if (!InitialiseProtocol())
			return (BridgeManager::kInitialiseFailed);
	}

	Interface::Print("\n");

	return (BridgeManager::kInitialiseSucceeded);
#endif // of else of if GTP7510
}

bool BridgeManager::BeginSession(void)
{
	Interface::Print("Beginning session...\n");

	SetupSessionPacket beginSessionPacket(SetupSessionPacket::kBeginSession);

	if (!SendPacket(&beginSessionPacket))
	{
		Interface::PrintError("Failed to begin session!\n");
		return (false);
	}

	SetupSessionResponse setupSessionResponse;
	if (!ReceivePacket(&setupSessionResponse))
		return (false);

	int result = setupSessionResponse.GetUnknown();

	// 131072 for Galaxy S II, 0 for other devices.
	if (result != 0 && result != 131072)
	{
		Interface::PrintError("Unexpected device info response!\nExpected: 0\nReceived:%d\n", result);
		return (false);
	}

	// -------------------- KIES DOESN'T DO THIS --------------------

	SetupSessionPacket deviceTypePacket(SetupSessionPacket::kDeviceInfo);

	if (!SendPacket(&deviceTypePacket))
	{
		Interface::PrintError("Failed to request device type!\n");
		return (false);
	}

	if (!ReceivePacket(&setupSessionResponse))
		return (false);

	int deviceType = setupSessionResponse.GetUnknown();

	// TODO: Work out what this value is... it has been either 180 or 0 for Galaxy S phones, 3 on the Galaxy Tab, 190 for SHW-M110S.
	if (deviceType != 180 && deviceType != 0 && deviceType != 3 && deviceType != 190)
	{
		Interface::PrintError("Unexpected device info response!\nExpected: 180, 0 or 3\nReceived:%d\n", deviceType);
		return (false);
	}
	else
	{
		Interface::Print("Session begun with device of type: %d\n\n", result);
	}

	return (true);
}

bool BridgeManager::EndSession(bool reboot)
{
	Interface::Print("Ending session...\n");

	EndSessionPacket *endSessionPacket = new EndSessionPacket(EndSessionPacket::kRequestEndSession);
	bool success = SendPacket(endSessionPacket);
	delete endSessionPacket;

	if (!success)
	{
		Interface::PrintError("Failed to send end session packet!\n");

		return (false);
	}

	ResponsePacket *endSessionResponse = new ResponsePacket(ResponsePacket::kResponseTypeEndSession);
	success = ReceivePacket(endSessionResponse);
	delete endSessionResponse;

	if (!success)
	{
		Interface::PrintError("Failed to receive session end confirmation!\n");

		return (false);
	}

	if (reboot)
	{
		Interface::Print("Rebooting device...\n");

		EndSessionPacket *rebootDevicePacket = new EndSessionPacket(EndSessionPacket::kRequestRebootDevice);
		bool success = SendPacket(rebootDevicePacket);
		delete rebootDevicePacket;

		if (!success)
		{
			Interface::PrintError("Failed to send reboot device packet!\n");

			return (false);
		}

		ResponsePacket *rebootDeviceResponse = new ResponsePacket(ResponsePacket::kResponseTypeEndSession);
		success = ReceivePacket(rebootDeviceResponse);
		delete rebootDeviceResponse;

		if (!success)
		{
			Interface::PrintError("Failed to receive reboot confirmation!\n");

			return (false);
		}
	}

	return (true);
}

bool BridgeManager::SendPacket(OutboundPacket *packet, int timeout, bool retry)
{
	packet->Pack();

#if GTP7510
	//if (verbose)
	//	Interface::Print("Sending packet of %d bytes.\n", packet->GetSize());

	int dataTransferred;
	int result = libusb_bulk_transfer(deviceHandle, bEndpointAddress_data_out, packet->GetData(), packet->GetSize(),
		&dataTransferred, timeout);
#else // of if GTP7510

	int dataTransferred;
	int result = libusb_bulk_transfer(deviceHandle, outEndpoint, packet->GetData(), packet->GetSize(),
		&dataTransferred, timeout);
#endif // of else of if GTP7510

	if (result < 0 && retry)
	{
		// max(250, communicationDelay)
		int retryDelay = (communicationDelay > 250) ? communicationDelay : 250;

		if (verbose)
			Interface::PrintError("libusb error %d whilst sending packet.", result);

		// Retry
		for (int i = 0; i < 5; i++)
		{
			if (verbose)
				Interface::PrintErrorSameLine(" Retrying...\n");

			// Wait longer each retry
			Sleep(retryDelay * (i + 1));

#if GTP7510
			result = libusb_bulk_transfer(deviceHandle, bEndpointAddress_data_out, packet->GetData(), packet->GetSize(),
				&dataTransferred, timeout);
#else // of if GTP7510
			result = libusb_bulk_transfer(deviceHandle, outEndpoint, packet->GetData(), packet->GetSize(),
				&dataTransferred, timeout);
#endif // of else of if GTP7510

			if (result >= 0)
				break;

			if (verbose)
				Interface::PrintError("libusb error %d whilst sending packet.", result);
		}

		if (verbose)
			Interface::PrintErrorSameLine("\n");
	}

	if (communicationDelay != 0)
		Sleep(communicationDelay);

	if (result < 0 || dataTransferred != packet->GetSize())
		return (false);

	return (true);
}

#if GTP7510

int BridgeManager::ReceiveData(unsigned char * dest, int minLength, int maxLength, int timeout)
{
	//	Process libusb events.
	//	Ideally this would be based around a filehandle select/poll loop.
	
	struct timespec ts;
	
  #ifdef __MACH__
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts.tv_sec = mts.tv_sec;
  ts.tv_nsec = mts.tv_nsec;
  #else
  clock_gettime(CLOCK_REALTIME, &ts);
  #endif

	while (GetCntBytesAvail_bulk_in() < minLength)
	{
		//	See if we've waited long enough.
		{
      
    	struct timespec ts_0;

      #ifdef __MACH__
      clock_serv_t cclock;
      mach_timespec_t mts;
      host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
      clock_get_time(cclock, &mts);
      mach_port_deallocate(mach_task_self(), cclock);
      ts_0.tv_sec = mts.tv_sec;
      ts_0.tv_nsec = mts.tv_nsec;
      #else
      clock_gettime(CLOCK_REALTIME, &ts_0);
      #endif
      
      
			long sec = ts.tv_sec - ts_0.tv_sec;
			long msec = sec*1000;
			msec += ts.tv_nsec/1000/1000;
			msec -= ts_0.tv_nsec/1000/1000;
			if (timeout < msec)
			{
				Interface::Print("timeout after %d ms\n", msec);
				break;
			}
		}

		timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 500*1000;

		int rc = libusb_handle_events_timeout(libusbContext, &tv);
		if (LIBUSB_SUCCESS != rc)
		{
			Interface::Print("handle events: ");
			LogLibusbResult(rc);
			Interface::Print("\n");
		}
	}

	int avail = GetCntBytesAvail_bulk_in();
	if (minLength <= avail)
	{
		int cntCopy = std::min<int>(avail, maxLength);
		memcpy(dest, buffer_bulk_in_c, cntCopy);
		buffer_bulk_in_c += cntCopy;
		return cntCopy;
	}
	else
	{
		if (avail)
			Interface::Print("WARNING: partial receive of %d bytes, less than the %d minimum\n", avail, minLength);
		return 0;
	}
}

void BridgeManager::ClearReceivedData()
{
	buffer_bulk_in_c = buffer_bulk_in_e;
}

#else // of if GTP7510
#endif // of else of if GTP7510

bool BridgeManager::ReceivePacket(InboundPacket *packet, int timeout, bool retry)
{
#if GTP7510
	//if (verbose)
	//	Interface::Print("Hoping to receive a packet of %d bytes.\n", packet->GetSize());

	int dataTransferred = 0;

	assert(packet->GetSize());

	int minLength = packet->IsSizeVariable() ? 1 : packet->GetSize();
	int maxLength = packet->GetSize();

	dataTransferred = ReceiveData(packet->GetData(), minLength, maxLength, timeout);

	if (dataTransferred != packet->GetSize() && !packet->IsSizeVariable())
		return (false);

	if (communicationDelay != 0)
		Sleep(communicationDelay);

#else // of if GTP7510

	int dataTransferred;
	int result = libusb_bulk_transfer(deviceHandle, inEndpoint, packet->GetData(), packet->GetSize(),
		&dataTransferred, timeout);

	if (result < 0 && retry)
	{
		// max(250, communicationDelay)
		int retryDelay = (communicationDelay > 250) ? communicationDelay : 250;

		if (verbose)
			Interface::PrintError("libusb error %d whilst receiving packet.", result);

		// Retry
		for (int i = 0; i < 5; i++)
		{
			if (verbose)
				Interface::PrintErrorSameLine(" Retrying...\n");

			// Wait longer each retry
			Sleep(retryDelay * (i + 1));

			result = libusb_bulk_transfer(deviceHandle, inEndpoint, packet->GetData(), packet->GetSize(),
				&dataTransferred, timeout);

			if (result >= 0)
				break;

			if (verbose)
				Interface::PrintError("libusb error %d whilst receiving packet.", result);
		}

		if (verbose)
			Interface::PrintErrorSameLine("\n");
	}

	if (communicationDelay != 0)
		Sleep(communicationDelay);

	if (result < 0 || (dataTransferred != packet->GetSize() && !packet->IsSizeVariable()))
		return (false);

#endif // of else of if GTP7510

	packet->SetReceivedSize(dataTransferred);

	return (packet->Unpack());
}

bool BridgeManager::RequestDeviceInfo(unsigned int request, int *result)
{
	SetupSessionPacket beginSessionPacket(request);
	bool success = SendPacket(&beginSessionPacket);

	if (!success)
	{
		Interface::PrintError("Failed to request device info packet!\n");

		if (verbose)
			Interface::PrintError("Failed request: %u\n", request);

		return (false);
	}

	SetupSessionResponse deviceInfoResponse;
	if (!ReceivePacket(&deviceInfoResponse))
		return (false);

	*result = deviceInfoResponse.GetUnknown();

	return (true);
}

bool BridgeManager::SendPitFile(FILE *file)
{
	fseek(file, 0, SEEK_END);
	long fileSize = ftell(file);
	rewind(file);

	// Start file transfer
	PitFilePacket *pitFilePacket = new PitFilePacket(PitFilePacket::kRequestFlash);
	bool success = SendPacket(pitFilePacket);
	delete pitFilePacket;

	if (!success)
	{
		Interface::PrintError("Failed to initialise PIT file transfer!\n");
		return (false);
	}

	PitFileResponse *pitFileResponse = new PitFileResponse();
	success = ReceivePacket(pitFileResponse);
	delete pitFileResponse;

	if (!success)
	{
		Interface::PrintError("Failed to confirm transfer initialisation!\n");
		return (false);
	}

	// Transfer file size
	FlashPartPitFilePacket *flashPartPitFilePacket = new FlashPartPitFilePacket(fileSize);
	success = SendPacket(flashPartPitFilePacket);
	delete flashPartPitFilePacket;

	if (!success)
	{
		Interface::PrintError("Failed to send PIT file part information!\n");
		return (false);
	}

	pitFileResponse = new PitFileResponse();
	success = ReceivePacket(pitFileResponse);
	delete pitFileResponse;

	if (!success)
	{
		Interface::PrintError("Failed to confirm sending of PIT file part information!\n");
		return (false);
	}

	// Flash pit file
	SendFilePartPacket *sendFilePartPacket = new SendFilePartPacket(file, fileSize);
	success = SendPacket(sendFilePartPacket);
	delete sendFilePartPacket;

	if (!success)
	{
		Interface::PrintError("Failed to send file part packet!\n");
		return (false);
	}

	pitFileResponse = new PitFileResponse();
	success = ReceivePacket(pitFileResponse);
	delete pitFileResponse;

	if (!success)
	{
		Interface::PrintError("Failed to receive PIT file part response!\n");
		return (false);
	}

	// End pit file transfer
	EndPitFileTransferPacket *endPitFileTransferPacket = new EndPitFileTransferPacket(fileSize);
	success = SendPacket(endPitFileTransferPacket);
	delete endPitFileTransferPacket;

	if (!success)
	{
		Interface::PrintError("Failed to send end PIT file transfer packet!\n");
		return (false);
	}

	pitFileResponse = new PitFileResponse();
	success = ReceivePacket(pitFileResponse);
	delete pitFileResponse;

	if (!success)
	{
		Interface::PrintError("Failed to confirm end of PIT file transfer!\n");
		return (false);
	}

	return (true);
}

int BridgeManager::ReceivePitFile(unsigned char **pitBuffer)
{
	*pitBuffer = nullptr;

	bool success;

	// Start file transfer
	PitFilePacket *pitFilePacket = new PitFilePacket(PitFilePacket::kRequestDump);
	success = SendPacket(pitFilePacket);
	delete pitFilePacket;

	if (!success)
	{
		Interface::PrintError("Failed to request receival of PIT file!\n");
		return (0);
	}

	PitFileResponse *pitFileResponse = new PitFileResponse();
	success = ReceivePacket(pitFileResponse);
	int fileSize = pitFileResponse->GetFileSize();
	delete pitFileResponse;

	if (!success)
	{
		Interface::PrintError("Failed to receive PIT file size!\n");
		return (0);
	}

	int transferCount = fileSize / ReceiveFilePartPacket::kDataSize;
	if (fileSize % ReceiveFilePartPacket::kDataSize != 0)
		transferCount++;

	unsigned char *buffer = new unsigned char[fileSize];
	int offset = 0;

	// NOTE: The PIT file appears to always be padded out to exactly 4 kilobytes.
	for (int i = 0; i < transferCount; i++)
	{
		DumpPartPitFilePacket *requestPacket = new DumpPartPitFilePacket(i);
		success = SendPacket(requestPacket);
		delete requestPacket;

		if (!success)
		{
			Interface::PrintError("Failed to request PIT file part #%d!\n", i);
			delete [] buffer;
			return (0);
		}

		ReceiveFilePartPacket *receiveFilePartPacket = new ReceiveFilePartPacket();
		success = ReceivePacket(receiveFilePartPacket);

		if (!success)
		{
			Interface::PrintError("Failed to receive PIT file part #%d!\n", i);
			delete receiveFilePartPacket;
			delete [] buffer;
			return (0);
		}

		// Copy the whole packet data into the buffer.
		memcpy(buffer + offset, receiveFilePartPacket->GetData(), receiveFilePartPacket->GetReceivedSize());
		offset += receiveFilePartPacket->GetReceivedSize();

		delete receiveFilePartPacket;
	}

	// End file transfer
	pitFilePacket = new PitFilePacket(PitFilePacket::kRequestEndTransfer);
	success = SendPacket(pitFilePacket);
	delete pitFilePacket;

	if (!success)
	{
		Interface::PrintError("Failed to send request to end PIT file transfer!\n");
		delete [] buffer;
		return (0);
	}

	pitFileResponse = new PitFileResponse();
	success = ReceivePacket(pitFileResponse);
	delete pitFileResponse;

	if (!success)
	{
		Interface::PrintError("Failed to receive end PIT file transfer verification!\n");
		delete [] buffer;
		return (0);
	}

	*pitBuffer = buffer;
	return (fileSize);
}

bool BridgeManager::SendFile(FILE *file, int destination, int fileIdentifier)
{
	if (destination != EndFileTransferPacket::kDestinationModem && destination != EndFileTransferPacket::kDestinationPhone)
	{
		Interface::PrintError("Attempted to send file to unknown destination!\n");
		return (false);
	}

	if (destination == EndFileTransferPacket::kDestinationModem && fileIdentifier != -1)
	{
		Interface::PrintError("The modem file does not have an identifier!\n");
		return (false);
	}

	FileTransferPacket *flashFileTransferPacket = new FileTransferPacket(FileTransferPacket::kRequestFlash);
	bool success = SendPacket(flashFileTransferPacket);
	delete flashFileTransferPacket;

	if (!success)
	{
		Interface::PrintError("Failed to initialise file transfer!\n");
		return (false);
	}

	fseek(file, 0, SEEK_END);
	long fileSize = ftell(file);
	rewind(file);

	ResponsePacket *fileTransferResponse = new ResponsePacket(ResponsePacket::kResponseTypeFileTransfer);
	success = ReceivePacket(fileTransferResponse);
	delete fileTransferResponse;

	if (!success)
	{
		Interface::PrintError("Failed to confirm transfer initialisation!\n");
		return (false);
	}

	int sequenceCount = fileSize / (kMaxSequenceLength * SendFilePartPacket::kDefaultPacketSize);
	int lastSequenceSize = kMaxSequenceLength;
	int partialPacketLength = fileSize % SendFilePartPacket::kDefaultPacketSize;
	if  (fileSize % (kMaxSequenceLength * SendFilePartPacket::kDefaultPacketSize) != 0)
	{
		sequenceCount++;

		int lastSequenceBytes = fileSize % (kMaxSequenceLength * SendFilePartPacket::kDefaultPacketSize);
		lastSequenceSize = lastSequenceBytes / SendFilePartPacket::kDefaultPacketSize;
		if (partialPacketLength != 0)
			lastSequenceSize++;
	}

	long bytesTransferred = 0;
	int currentPercent;
	int previousPercent = 0;
	Interface::Print("0%%");

	for (int sequenceIndex = 0; sequenceIndex < sequenceCount; sequenceIndex++)
	{
		// Min(lastSequenceSize, 131072)
		bool isLastSequence = sequenceIndex == sequenceCount - 1;
		int sequenceSize = (isLastSequence) ? lastSequenceSize : kMaxSequenceLength;

		FlashPartFileTransferPacket *beginFileTransferPacket = new FlashPartFileTransferPacket(0, 2 * sequenceSize);
		success = SendPacket(beginFileTransferPacket);
		delete beginFileTransferPacket;

		if (!success)
		{
			Interface::PrintErrorSameLine("\n");
			Interface::PrintError("Failed to begin file transfer sequence!\n");
			return (false);
		}

		fileTransferResponse = new ResponsePacket(ResponsePacket::kResponseTypeFileTransfer);
		bool success = ReceivePacket(fileTransferResponse);
		delete fileTransferResponse;

		if (!success)
		{
			Interface::PrintErrorSameLine("\n");
			Interface::PrintError("Failed to confirm beginning of file transfer sequence!\n");
			return (false);
		}

		SendFilePartPacket *sendFilePartPacket;
		SendFilePartResponse *sendFilePartResponse;

		for (int filePartIndex = 0; filePartIndex < sequenceSize; filePartIndex++)
		{
			// Send
			sendFilePartPacket = new SendFilePartPacket(file);
			success = SendPacket(sendFilePartPacket);
			delete sendFilePartPacket;

			if (!success)
			{
				Interface::PrintErrorSameLine("\n");
				Interface::PrintError("Failed to send file part packet!\n");
				return (false);
			}

			// Response
			sendFilePartResponse = new SendFilePartResponse();
			success = ReceivePacket(sendFilePartResponse);
			int receivedPartIndex = sendFilePartResponse->GetPartIndex();

			if (verbose)
			{
				const unsigned char *data = sendFilePartResponse->GetData();
				Interface::Print("File Part #%d... Response: %X  %X  %X  %X  %X  %X  %X  %X \n", filePartIndex,
					data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
			}

			delete sendFilePartResponse;

			if (!success)
			{
				Interface::PrintErrorSameLine("\n");
				Interface::PrintError("Failed to receive file part response!\n");

				for (int retry = 0; retry < 4; retry++)
				{
					Interface::PrintErrorSameLine("\n");
					Interface::PrintError("Retrying...");

					// Send
					sendFilePartPacket = new SendFilePartPacket(file);
					success = SendPacket(sendFilePartPacket);
					delete sendFilePartPacket;

					if (!success)
					{
						Interface::PrintErrorSameLine("\n");
						Interface::PrintError("Failed to send file part packet!\n");
						return (false);
					}

					// Response
					sendFilePartResponse = new SendFilePartResponse();
					success = ReceivePacket(sendFilePartResponse);
					int receivedPartIndex = sendFilePartResponse->GetPartIndex();

					if (verbose)
					{
						const unsigned char *data = sendFilePartResponse->GetData();
						Interface::Print("File Part #%d... Response: %X  %X  %X  %X  %X  %X  %X  %X \n", filePartIndex,
							data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
					}

					delete sendFilePartResponse;

					if (receivedPartIndex != filePartIndex)
					{
						Interface::PrintErrorSameLine("\n");
						Interface::PrintError("Expected file part index: %d Received: %d\n", filePartIndex, receivedPartIndex);
						return (false);
					}

					if (success)
						break;
				}

				if (!success)
					return (false);
			}

			if (receivedPartIndex != filePartIndex)
			{
				Interface::PrintErrorSameLine("\n");
				Interface::PrintError("Expected file part index: %d Received: %d\n", filePartIndex, receivedPartIndex);
				return (false);
			}

			bytesTransferred += SendFilePartPacket::kDefaultPacketSize;
			if (bytesTransferred > fileSize)
				bytesTransferred = fileSize;

			currentPercent = (int)(100.0f * ((float)bytesTransferred / (float)fileSize));

			if (currentPercent != previousPercent)
			{
				if (!verbose)
				{
					if (previousPercent < 10)
						Interface::Print("\b\b%d%%", currentPercent);
					else
						Interface::Print("\b\b\b%d%%", currentPercent);
				}
				else
				{
					Interface::Print("\n%d%%\n", currentPercent);
				}
			}

			previousPercent = currentPercent;
		}

		int lastFullPacketIndex = 2 * ((isLastSequence && partialPacketLength != 0) ? sequenceSize - 1 : sequenceSize);

		if (destination == EndFileTransferPacket::kDestinationPhone)
		{
			EndPhoneFileTransferPacket *endPhoneFileTransferPacket = new EndPhoneFileTransferPacket(
				(isLastSequence) ? partialPacketLength : 0, lastFullPacketIndex, 0, 0, fileIdentifier, isLastSequence);

			success = SendPacket(endPhoneFileTransferPacket, 3000);
			delete endPhoneFileTransferPacket;

			if (!success)
			{
				Interface::PrintErrorSameLine("\n");
				Interface::PrintError("Failed to end phone file transfer sequence!\n");
				return (false);
			}
		}
		else // destination == EndFileTransferPacket::kDestinationModem
		{
			EndModemFileTransferPacket *endModemFileTransferPacket = new EndModemFileTransferPacket(
				(isLastSequence) ? partialPacketLength : 0, lastFullPacketIndex, 0, 0, isLastSequence);

			success = SendPacket(endModemFileTransferPacket, 3000);
			delete endModemFileTransferPacket;

			if (!success)
			{
				Interface::PrintErrorSameLine("\n");
				Interface::PrintError("Failed to end modem file transfer sequence!\n");
				return (false);
			}
		}

		fileTransferResponse = new ResponsePacket(ResponsePacket::kResponseTypeFileTransfer);
		success = ReceivePacket(fileTransferResponse, 30000);
		delete fileTransferResponse;

		if (!success)
		{
			Interface::PrintErrorSameLine("\n");
			Interface::PrintError("Failed to confirm end of file transfer sequence!\n");
			return (false);
		}
	}

	if (!verbose)
		Interface::Print("\n");

	return (true);
}

bool BridgeManager::ReceiveDump(int chipType, int chipId, FILE *file)
{
	bool success;

	// Start file transfer
	BeginDumpPacket *beginDumpPacket = new BeginDumpPacket(chipType, chipId);
	success = SendPacket(beginDumpPacket);
	delete beginDumpPacket;

	if (!success)
	{
		Interface::PrintError("Failed to request dump!\n");
		return (false);
	}

	DumpResponse *dumpResponse = new DumpResponse();
	success = ReceivePacket(dumpResponse);
	unsigned int dumpSize = dumpResponse->GetDumpSize();
	delete dumpResponse;

	if (!success)
	{
		Interface::PrintError("Failed to receive dump size!\n");
		return (false);
	}

	unsigned int transferCount = dumpSize / ReceiveFilePartPacket::kDataSize;
	if (transferCount % ReceiveFilePartPacket::kDataSize != 0)
		transferCount++;

	char *buffer = new char[kDumpBufferSize * ReceiveFilePartPacket::kDataSize];
	int bufferOffset = 0;

	for (unsigned int i = 0; i < transferCount; i++)
	{
		DumpPartFileTransferPacket *dumpPartPacket = new DumpPartFileTransferPacket(i);
		success = SendPacket(dumpPartPacket);
		delete dumpPartPacket;

		if (!success)
		{
			Interface::PrintError("Failed to request dump part #%d!\n", i);
			delete [] buffer;
			return (false);
		}

		ReceiveFilePartPacket *receiveFilePartPacket = new ReceiveFilePartPacket();
		success = ReceivePacket(receiveFilePartPacket);

		if (!success)
		{
			Interface::PrintError("Failed to receive dump part #%d!\n", i);
			continue;
			delete receiveFilePartPacket;
			delete [] buffer;
			return (true);
		}

		if (bufferOffset + receiveFilePartPacket->GetReceivedSize() > kDumpBufferSize * ReceiveFilePartPacket::kDataSize)
		{
			// Write the buffer to the output file
			fwrite(buffer, 1, bufferOffset, file);
			bufferOffset = 0;
		}

		// Copy the packet data into pitFile.
		memcpy(buffer + bufferOffset, receiveFilePartPacket->GetData(), receiveFilePartPacket->GetReceivedSize());
		bufferOffset += receiveFilePartPacket->GetReceivedSize();

		delete receiveFilePartPacket;
	}

	if (bufferOffset != 0)
	{
		// Write the buffer to the output file
		fwrite(buffer, 1, bufferOffset, file);
	}

	delete [] buffer;

	// End file transfer
	FileTransferPacket *fileTransferPacket = new FileTransferPacket(FileTransferPacket::kRequestEnd);
	success = SendPacket(fileTransferPacket);
	delete fileTransferPacket;

	if (!success)
	{
		Interface::PrintError("Failed to send request to end dump transfer!\n");
		return (false);
	}

	ResponsePacket *responsePacket = new ResponsePacket(ResponsePacket::kResponseTypeFileTransfer);
	success = ReceivePacket(responsePacket);
	delete responsePacket;

	if (!success)
	{
		Interface::PrintError("Failed to receive end dump transfer verification!\n");
		return (false);
	}

	return (true);
}
