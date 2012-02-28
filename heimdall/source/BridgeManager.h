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

#ifndef BRIDGEMANAGER_H
#define BRIDGEMANAGER_H

#define GTP7510 1

#if GTP7510

//	I don't think this header is available on Win32
#include <stdint.h>

#else // of if GTP7510
#endif // of else of if GTP7510

// Heimdall
#include "Heimdall.h"

struct libusb_context;
struct libusb_device;
struct libusb_device_handle;
#if GTP7510
struct libusb_transfer;
#else // of if GTP7510
#endif // of else of if GTP7510

namespace Heimdall
{
	class InboundPacket;
	class OutboundPacket;

	class DeviceIdentifier
	{
		public:

			const int vendorId;
			const int productId;

			DeviceIdentifier(int vid, int pid) :
				vendorId(vid),
				productId(pid)
			{
			}
	};

	class BridgeManager
	{
		public:

			enum
			{
				kSupportedDeviceCount		= 3,

				kCommunicationDelayDefault	= 0,
				kDumpBufferSize				= 4096
			};

			enum
			{
				kInitialiseSucceeded = 0,
				kInitialiseFailed,
				kInitialiseDeviceNotDetected
			};

			enum
			{
				kVidSamsung	= 0x04E8
			};

			enum
			{
				kPidGalaxyS		    = 0x6601,
				kPidGalaxyS2        = 0x685D, // and GT-7510 Galaxy Tab 10.1
				kPidDroidCharge     = 0x68C3
			};

		private:

			static const DeviceIdentifier supportedDevices[kSupportedDeviceCount];

			bool verbose;

			libusb_context *libusbContext;
			libusb_device_handle *deviceHandle;
			libusb_device *heimdallDevice;

#if GTP7510

			int bInterfaceNumber_comm;
			int bAlternateSetting_comm;
			int bEndpointAddress_comm;

			int bInterfaceNumber_data;
			int bAlternateSetting_data;
			int bEndpointAddress_data_in;
			int bEndpointAddress_data_out;

			//	True if we want to maintain an outstanding async bulk_in on the data_in endpoint.
			bool want_outstanding_bulk_data_in;

			//	Active outstanding async bulk_in on the data_in endpoint.
			libusb_transfer * transfer_bulk_data_in;
			uint8_t * buffer_bulk_data_in_b; // beginning of allocation
			uint8_t * buffer_bulk_data_in_c; // beginning of unconsumed part
			uint8_t * buffer_bulk_data_in_e; // end of valid data
			uint8_t * buffer_bulk_data_in_z; // end of allocation

			//	True if we want to maintain an outstanding interrupt transfer on the comm endpoint.
			bool want_outstanding_intr_comm;

			//	Active outstanding async interrupt transfer on the comm endpoint.
			libusb_transfer * transfer_intr_comm;

#else // of if GTP7510

			int interfaceIndex;
			int inEndpoint;
			int outEndpoint;

#endif // of else of if GTP7510

			int communicationDelay;

#ifdef OS_LINUX

			bool detachedDriver;

#endif

#if GTP7510

			bool CheckProtocol(void);
			bool ResetInterface();
			bool InitialiseProtocol(void);

			void perhaps_start_async_xfers();
			//bool async_bulk_or_intr(bool b_not_i, uint8_t endpoint, unsigned length, unsigned req);

			int get_cnt_bytes_avail_in();
			int receive_data(uint8_t * dest, int minLength, int maxLength, int timeout);

			void clear_received_data();

#else // of if GTP7510

			bool CheckProtocol(void);
			bool InitialiseProtocol(void);

#endif // of else of if GTP7510

		public:

#if GTP7510

#else // of if GTP7510
#endif // of else of if GTP7510

			BridgeManager(bool verbose, int communicationDelay);
			~BridgeManager();

			bool DetectDevice(void);
			int Initialise(void);

#if GTP7510

			bool sync_control(
				uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
				unsigned length, uint8_t * data, bool pipe_error_ok );
//			bool async_bulk(uint8_t endpoint, unsigned length, unsigned req);
//			bool async_interrupt(uint8_t endpoint, unsigned length, unsigned req);

#else // of if GTP7510
#endif // of else of if GTP7510

			bool BeginSession(void);
			bool EndSession(bool reboot);

			bool SendPacket(OutboundPacket *packet, int timeout = 3000, bool retry = true);
			bool ReceivePacket(InboundPacket *packet, int timeout = 3000, bool retry = true);

			bool RequestDeviceInfo(unsigned int request, int *result);

			bool SendPitFile(FILE *file);
			int ReceivePitFile(unsigned char **pitBuffer);

			bool SendFile(FILE *file, int destination, int fileIdentifier = -1);
			bool ReceiveDump(int chipType, int chipId, FILE *file);

			bool IsVerbose(void) const
			{
				return (verbose);
			}

#if GTP7510
			//	These are public just so they can be called from some extern "C" code.
			void xfer_async_bulk_data_in_complete(libusb_transfer * transfer);
			void xfer_async_intr_comm_complete(libusb_transfer * transfer);
#else // of if GTP7510
#endif // of else of if GTP7510
	};
}

#endif
