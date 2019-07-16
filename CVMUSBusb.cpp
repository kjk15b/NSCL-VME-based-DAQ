/*
    This software is Copyright by the Board of Trustees of Michigan
    State University (c) Copyright 2005.

    You may use this software under the terms of the GNU public license
    (GPL).  The terms of this license are described at:

     http://www.gnu.org/licenses/gpl.txt

     Author:
             Ron Fox
	     NSCL
	     Michigan State University
	     East Lansing, MI 48824-1321
*/

// Implementation of the CVMUSB class.

#include "CVMUSBusb.h"
#include "CVMUSBReadoutList.h"
#include <os.h>

#include <usb.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <iterator>
#include <stdexcept>

#include <pthread.h>

#include <thread>
#include <chrono>
using namespace std;

// Constants:

// Identifying marks for the VM-usb:

static const short USB_WIENER_VENDOR_ID(0x16dc);
static const short USB_VMUSB_PRODUCT_ID(0xb);

// Bulk transfer endpoints

static const int ENDPOINT_OUT(2);
static const int ENDPOINT_IN(0x86);

// Timeouts:

static const int DEFAULT_TIMEOUT(2000);	// ms.

// Retries for flushing the fifo/stopping data taking:

static const int DRAIN_RETRIES(5);    // Retries.




// Bits in the list target address word:

static const uint16_t TAVcsID0(1); // Bit mask of Stack id bit 0.
static const uint16_t TAVcsSel(2); // Bit mask to select list dnload
static const uint16_t TAVcsWrite(4); // Write bitmask.
static const uint16_t TAVcsIMMED(8); // Target the VCS immediately.
static const uint16_t TAVcsID1(0x10);
static const uint16_t TAVcsID2(0x20);
static const uint16_t TAVcsID12MASK(0x30); // Mask for top 2 id bits
static const uint16_t TAVcsID12SHIFT(4);

//   The following flag determines if enumerate needs to init the libusb:

static bool usbInitialized(false);

void print_stack(const char* beg, const char* end, size_t unitWidth)
{
  using namespace std;

  cout << "Stack dump:" << endl;
  auto it = beg;
  size_t datum=0;
  while (it < end) {
    std::copy(it, it+unitWidth, reinterpret_cast<char*>(&datum));
    cout << dec << setfill(' ') << setw(3) << distance(beg, it)/unitWidth;
    cout << " : 0x";
    cout << hex << setfill('0') << setw(2*unitWidth) << datum << endl;

    //reset value
    datum = 0;
    advance(it, unitWidth);
  }

  // dump any remainder

  cout << dec << setfill(' ');
  cout << "--------" << std::endl;
}
 
/////////////////////////////////////////////////////////////////////
/*!
  Enumerate the Wiener/JTec VM-USB devices.
  This function returns a vector of usb_device descriptor pointers
  for each Wiener/JTec device on the bus.  The assumption is that
  some other luck function has initialized the libusb.
  It is perfectly ok for there to be no VM-USB device on the USB 
  subsystem.  In that case an empty vector is returned.
*/
vector<struct usb_device*>
CVMUSBusb::enumerate()
{
  if(!usbInitialized) {
    usb_init();
    usbInitialized = true;
  }
  usb_find_busses();		// re-enumerate the busses
  usb_find_devices();		// re-enumerate the devices.
  
  // Now we are ready to start the search:
  
  vector<struct usb_device*> devices;	// Result vector.
  struct usb_bus* pBus = usb_get_busses();

  while(pBus) {
    struct usb_device* pDevice = pBus->devices;
    while(pDevice) {
      usb_device_descriptor* pDesc = &(pDevice->descriptor);
      if ((pDesc->idVendor  == USB_WIENER_VENDOR_ID)    &&
	  (pDesc->idProduct == USB_VMUSB_PRODUCT_ID)) {
	devices.push_back(pDevice);
      }
      
      pDevice = pDevice->next;
    }
    
    pBus = pBus->next;
  }
  
  return devices;
}

/**
 * Return the serial number of a usb device.  This involves:
 * - Opening the device.
 * - Doing a simple string fetch on the SerialString
 * - closing the device.
 * - Converting that to an std::string which is then returned to the caller.
 *
 * @param dev - The usb_device* from which we want the serial number string.
 *
 * @return std::string
 * @retval The serial number string of the device.  For VM-USB's this is a
 *         string of the form VMnnnn where nnnn is an integer.
 *
 * @throw std::string exception if any of the USB functions fails indicating
 *        why.
 */
string
CVMUSBusb::serialNo(struct usb_device* dev)
{
  usb_dev_handle* pDevice = usb_open(dev);

  if (pDevice) {
    char szSerialNo[256];	// actual string is only 6chars + null.
    int nBytes = usb_get_string_simple(pDevice, dev->descriptor.iSerialNumber,
				       szSerialNo, sizeof(szSerialNo));
    usb_close(pDevice);

    if (nBytes > 0) {
      return std::string(szSerialNo);
    } else {
      throw std::string("usb_get_string_simple failed in CVMUSBusb::serialNo");
    }
				       
  } else {
    throw std::string("usb_open failed in CVMUSBusb::serialNo");
  }

}
////////////////////////////////////////////////////////////////////
/*!
  Construct the CVMUSB object.  This involves storing the
  device descriptor we are given, opening the device and
  claiming it.  Any errors are signalled via const char* exceptions.
  \param vmUsbDevice   : usb_device*
      Pointer to a USB device descriptor that we want to open.

  \bug
      At this point we take the caller's word that this is a VM-USB.
      Future implementations should verify the vendor and product
      id in the device structure, throwing an appropriate exception
      if there is aproblem.

*/
CVMUSBusb::CVMUSBusb(struct usb_device* device) :
    m_handle(0),
    m_device(device),
    m_timeout(DEFAULT_TIMEOUT)
{
  m_serial = serialNo(device);                  // Set the desired serial number.
  CMutexAttr  attr;
  attr.setType(PTHREAD_MUTEX_RECURSIVE_NP);
  m_pMutex  = new CMutex(attr);
  
  openVMUsb();
}
////////////////////////////////////////////////////////////////
/*!
    Destruction of the interface involves releasing the claimed
    interface, followed by closing the device.
*/
CVMUSBusb::~CVMUSBusb()
{
    usb_release_interface(m_handle, 0);
    usb_close(m_handle);
    
    delete m_pMutex;
    
    Os::usleep(5000);
}

/**
 * reconnect
 *
 * re open the VM-USB.
 * this is done by closing the device and then invoking
 * openVMUSBUsb which has code common to us and
 * the construtor.
 *   If we can read the firmware register in the VMUSB we assume we don't need
 *   to reconnect.
 *   
 *   @return bool - true if necessary.false if not
 */
bool
CVMUSBusb::reconnect()
{
  try {
    int fwid = readFirmwareID();
    return false;                      // Success so don't need to reconnect.
  }
  catch (...) {
    usb_release_interface(m_handle, 0);
    usb_close(m_handle);
    Os::usleep(1000);			// Let this all happen
    openVMUsb();
    return true;
  }
}

/*!
*  writeActionRegister
*
    Writing a value to the action register.  This is really the only
    special case for this code.  The action register is the only
    item that cannot be handled by creating a local list and
    then executing it immediately.
    Action register I/O requires a special list, see section 4.2, 4.3
    of the Wiener VM-USB manual for more information
    \param value : uint16_t
       The register value to write.
 */
void CVMUSBusb::writeActionRegister(uint16_t data) 
{
    CriticalSection s(*m_pMutex);
    char outPacket[100];


  // Build up the output packet:

  char* pOut = outPacket;

  pOut = static_cast<char*>(addToPacket16(pOut, 5)); // Select Register block for transfer.
  pOut = static_cast<char*>(addToPacket16(pOut, 10)); // Select action register wthin block.
  pOut = static_cast<char*>(addToPacket16(pOut, data));

  // This operation is write only.

  int outSize = pOut - outPacket;
  int status = usb_bulk_write(m_handle, ENDPOINT_OUT, 
      outPacket, outSize, DEFAULT_TIMEOUT);
  if (status < 0) {
    string message = "Error in usb_bulk_write, writing action register ";
    message == strerror(-status);
    throw message;
  }
  if (status != outSize) {
    throw "usb_bulk_write wrote different size than expected";
  }

  //print_stack(outPacket, outPacket+outSize, sizeof(uint16_t));

}

//////////////////////////////////////////////////////////////////////////
/////////////////////////// List operations  ////////////////////////////
/////////////////////////////////////////////////////////////////////////
  
/*!
    Execute a list immediately.  It is the caller's responsibility
    to ensure that no data taking is in progress and that data taking
    has run down (the last buffer was received).  
    The list is transformed into an out packet to the VMUSB and
    transaction is called to execute it and to get the response back.
    \param list  : CVMUSBReadoutList&
       A reference to the list of operations to execute.
    \param pReadBuffer : void*
       A pointer to the buffer that will receive the reply data.
    \param readBufferSize : size_t
       number of bytes of data available to the pReadBuffer.
    \param bytesRead : size_t*
       Return value to hold the number of bytes actually read.
 
    \return int
    \retval  0    - All went well.
    \retval -1    - The usb_bulk_write failed.
    \retval -2    - The usb_bulk_read failed.

    In case of failure, the reason for failure is stored in the
    errno global variable.
*/
int
CVMUSBusb::executeList(CVMUSBReadoutList&     list,
		   void*                  pReadoutBuffer,
		   size_t                 readBufferSize,
		   size_t*                bytesRead)
{
  size_t outSize;
  uint16_t* outPacket = listToOutPacket(TAVcsWrite | TAVcsIMMED,
					list, &outSize);
    
    // Now we can execute the transaction:
    
  int status = transaction(outPacket, outSize,
			   pReadoutBuffer, readBufferSize);
  
  
  
  delete []outPacket;
  if(status >= 0) {
    *bytesRead = status;
  } 
  else {
    *bytesRead = 0;
  }
  return  status;
  
}



/*!
   Load a list into the VM-USB for later execution.
   It is the callers responsibility to:
   -  keep track of the lists and their  storage requirements, so that 
      they are not loaded on top of or overlapping
      each other, or so that the available list memory is not exceeded.
   - Ensure that the list number is a valid value (0-7).
   - The listOffset is valid and that there is room in the list memory
     following it for the entire list being loaded.
   This code just load the list, it does not attach it to any specific trigger.
   that is done via register operations performed after all the required lists
   are in place.
    
   \param listNumber : uint8_t  
      Number of the list to load. 
   \param list       : CVMUSBReadoutList
      The constructed list.
   \param listOffset : off_t
      The offset in list memory at which the list is loaded.
      Question for the Wiener/Jtec guys... is this offset a byte or long
      offset... I'm betting it's a longword offset.
*/
int
CVMUSBusb::loadList(uint8_t  listNumber, CVMUSBReadoutList& list, off_t listOffset)
{
  // Need to construct the TA field, straightforward except for the list number
  // which is splattered all over creation.
  
  uint16_t ta = TAVcsSel | TAVcsWrite;
  if (listNumber & 1)  ta |= TAVcsID0;
  if (listNumber & 2)  ta |= TAVcsID1; // Probably the simplest way for this
  if (listNumber & 4)  ta |= TAVcsID2; // few bits.

  size_t   packetSize;
  uint16_t* outPacket = listToOutPacket(ta, list, &packetSize, listOffset);
  int status = usb_bulk_write(m_handle, ENDPOINT_OUT,
			      reinterpret_cast<char*>(outPacket),
			      packetSize, DEFAULT_TIMEOUT);
  if (status < 0) {
    errno = -status;
    status= -1;
  }


  delete []outPacket;
  return (status >= 0) ? 0 : status;


  
}
/*!
  Execute a bulk read for the user.  The user will need to do this
  when the VMUSB is in autonomous data taking mode to read buffers of data
  it has available.
  \param data : void*
     Pointer to user data buffer for the read.
  \param buffersSize : size_t
     size of 'data' in bytes.
  \param transferCount : size_t*
     Number of bytes actually transferred on success.
  \param timeout : int [2000]
     Timeout for the read in ms.
 
  \return int
  \retval 0   Success, transferCount has number of bytes transferred.
  \retval -1  Read failed, errno has the reason. transferCount will be 0.

*/
int 
CVMUSBusb::usbRead(void* data, size_t bufferSize, size_t* transferCount, int timeout)
{
  CriticalSection s(*m_pMutex);
  int status = usb_bulk_read(m_handle, ENDPOINT_IN,
			     static_cast<char*>(data), bufferSize,
			     timeout);
  if (status >= 0) {
    *transferCount = status;
    status = 0;
  } 
  else {
    errno = -status;
    status= -1;
    *transferCount = 0;
  }
  return status;
}

/*! 
   Set a new transaction timeout.  The transaction timeout is used for
   all usb transactions but usbRead where the user has full control.
   \param ms : int
      New timeout in milliseconds.
*/
void
CVMUSBusb::setDefaultTimeout(int ms)
{
  m_timeout = ms;
}


////////////////////////////////////////////////////////////////////////
/////////////////////////////// Utility methods ////////////////////////
////////////////////////////////////////////////////////////////////////
/*
   Utility function to perform a 'symmetric' transaction.
   Most operations on the VM-USB are 'symmetric' USB operations.
   This means that a usb_bulk_write will be done followed by a
   usb_bulk_read to return the results/status of the operation requested
   by the write. Depending on the transaction and the amount of data
   expected to be received, there will be one or more calls to usb_bulk_read.
   If a failure occurs after the first read, the data from all subsequent reads
   that succeeded is returned to the user. It is not an error to timeout on
   any read operation after the first.

   Parametrers:
   void*   writePacket   - Pointer to the packet to write.
   size_t  writeSize     - Number of bytes to write from writePacket.
   
   void*   readPacket    - Pointer to storage for the read.
   size_t  readSize      - Number of bytes to attempt to read.


   Returns:
     > 0 the actual number of bytes read into the readPacket...
         and all should be considered to have gone well.
     -1  The write failed with the reason in errno.
     -2  The read failed with the reason in errno.

   NOTE:  The m_timeout is used for both write and read timeouts. To change
   the value of m_timeout, use setDefaultTimeout().

*/
int
CVMUSBusb::transaction(void* writePacket, size_t writeSize,
		    void* readPacket,  size_t readSize)
{ 
  char buf[8192];
  size_t bytesToTransfer;

  //print_stack(reinterpret_cast<char*>(writePacket), 
  //    reinterpret_cast<char*>(writePacket)+writeSize, sizeof(uint16_t));

    CriticalSection s(*m_pMutex);
    int status = usb_bulk_write(m_handle, ENDPOINT_OUT,
		                        		static_cast<char*>(writePacket), writeSize, 
                                DEFAULT_TIMEOUT);
    if (status < 0) {
      errno = -status;
      return -1;		// Write failed!!
    } 
    
    status    = usb_bulk_read(m_handle, ENDPOINT_IN,
			      buf, sizeof(buf), m_timeout);
    if (status < 0) {
      if ((status == EINTR) || (status == EAGAIN)) {
	status = 0;                 // can try again.
      } else {
	errno = -status;
	return -2;
      }
    } 

    bytesToTransfer =  std::min(static_cast<size_t>(status ), readSize);
    
    long bytesRead = bytesToTransfer;
    auto pReadCursor = reinterpret_cast<char*>(readPacket);
    // Copy the newly read data into the output buffer
    pReadCursor = std::copy(
        buf,
	buf + bytesToTransfer, pReadCursor);

    int nAttempts = 0;
    int maxAttempts = readSize/std::min(int(sizeof(buf)),getBufferSize()*2);


    if (bytesRead < readSize) {
        // looks like there might be a bug that causes the first read to
        // return 0 bytes. Adjust to try at least 1 extra time if needed.
        maxAttempts += 1;

//        std::cout << "Read " << bytesRead<< " bytes " << std::endl;
//        std::cout << "need to try again " << maxAttempts << " times"
//                  << " for all " << readSize << " bytes requested" << std::endl;
    }
    // if in events buffering mode, then getBufferSize() returns -1. This
    // short circuits the while loop below because the maxAttempts will be negative.

    // iteratively read until we have the data we desire
    while ((bytesRead < readSize) && (nAttempts < maxAttempts)) {
      size_t bytesLeft = readSize - bytesRead;
      status = usb_bulk_read(m_handle, ENDPOINT_IN, buf,
			     sizeof(buf), m_timeout);
      if (status < 0) {
          if ( status != -ETIMEDOUT) {
	    if ( (status == EAGAIN) || (status == EINTR)) {
	      status = 0;                               // can try again.
	    } else {
              errno = -status;
              return -2;
	    }
          } else {
              return bytesRead;
          }
      }

//      std::cout << "updating after a successful read of " << status << " bytes" << std::endl;
      bytesToTransfer = std::min(static_cast<size_t>(status), bytesLeft);
      pReadCursor = std::copy(
           buf,
	   buf+bytesToTransfer, pReadCursor);

      bytesRead += bytesToTransfer;

      nAttempts++;
    }

    return bytesRead;
}

/*
   Reading a register is just creating the appropriate CVMUSBReadoutList
   and executing it immediatly.
*/
uint32_t
CVMUSBusb::readRegister(unsigned int address)
{
    CVMUSBReadoutList list;
    uint32_t          data;
    size_t            count;
    list.addRegisterRead(address);

    int status = executeList(list,
			     &data,
			     sizeof(data),
			     &count);
    if (status == -1) {
	char message[100];
	sprintf(message, "CVMUSBusb::readRegister USB write failed: %s",
		strerror(errno));
	throw string(message);
    }
    if (status == -2) {
	char message[100];
	sprintf(message, "CVMUSBusb::readRegister USB read failed %s",
		strerror(errno));
	throw string(message);

    }

    return data;

}
/*
  
   Writing a register is just creating the appropriate list and
   executing it immediately:
*/
void
CVMUSBusb::writeRegister(unsigned int address, uint32_t data)
{
  CVMUSBReadoutList list;
  uint16_t          rdstatus;
  size_t            rdcount;
  list.addRegisterWrite(address, data);

  int status = executeList(list,
      &rdstatus, 
      sizeof(rdstatus),
      &rdcount);

  if (status == -1) {
    char message[100];
    sprintf(message, "CVMUSBusb::writeRegister USB write failed: %s",
        strerror(errno));
    throw string(message);
  }
  if (status == -2) {
    char message[100];
    sprintf(message, "CVMUSBusb::writeRegister USB read failed %s",
        strerror(errno));
    throw string(message);

  }
}

/**
 * openVMUsb
 *
 *   Open the VM-USB.  This contains code that is 
 *   common to both the constructor and reconnect.
 *
 *   We assume that m_serial is set to the
 *   desired VM-USB serial number.
 *
 *   @throw std::string on errors.
 */
void
CVMUSBusb::openVMUsb()
{
    enumerateAndIdentify();
    m_handle  = usb_open(m_device);
    if (!m_handle) {
        throw "CVMUSBusb::CVMUSBusb  - unable to open the device";
    }

    // Resetting the device causes the usb device to lose its enumeration.
    // It must be found again and then reopened.
    resetVMUSB();

    enumerateAndIdentify();
    
    m_handle  = usb_open(m_device);
    if (!m_handle) {
        throw "CVMUSBusb::CVMUSBusb  - unable to open the device";
    }

    // Now claim the interface.. again this could in theory fail.. but.
    usb_set_configuration(m_handle, 1);
    int status = usb_claim_interface(m_handle,
                                     0);
    if (status == -EBUSY) {
        throw "CVMUSBusb::CVMUSBusb - some other process has already claimed";

    }

    if (status == -ENOMEM) {
        throw "CVMUSBusb::CVMUSBusb - claim failed for lack of memory";
    }
    // Errors we don't know about:

    if (status < 0) {
        std::string msg("Failed to claim the interface: ");
        msg += strerror(-status);
        throw msg;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // reset the module
    writeActionRegister(ActionRegister::clear);

    // Turn off DAQ mode and flush any data that might be trapped in the VMUSB
    // FIFOS.  To write the action register may require at least one read of the FIFO.
    //

    int retriesLeft = DRAIN_RETRIES;
    uint8_t buffer[1024*13*2];  // Biggest possible VM-USB buffer.
    size_t  bytesRead;

    while (retriesLeft) {
        try {
            usbRead(buffer, sizeof(buffer), &bytesRead, 1);
            writeActionRegister(0);     // Turn off data taking.
            break;                      // done if success.
        } catch (...) {
            retriesLeft--;
        }
    }
    if (!retriesLeft) {
        std::cerr << "** Warning - not able to stop data taking VM-USB may need to be power cycled\n";
    }
    
    while(usbRead(buffer, sizeof(buffer), &bytesRead) == 0) {
        fprintf(stderr, "Flushing VMUSB Buffer\n");
    }
    
    // Now set the irq mask so that all bits are set..that:
    // - is the only way to ensure the m_irqMask value matches the register.
    // - ensures m_irqMask actually gets set:

    writeIrqMask(0x7f);

    // Read the state of the module
    initializeShadowRegisters();
}


void CVMUSBusb::initializeShadowRegisters()
{
    readGlobalMode();
    readDAQSettings();
    readLEDSource();
    readDeviceSource();
    readDGG_A();
    readDGG_B();
    readDGG_Extended();
    for (int i=1; i<5; ++i) readVector(i);
    readIrqMask();
    readBulkXferSetup();
    readEventsPerBuffer();
}


/*!
 * \brief Reset the already opened VMUSB
 *
 * The reset operation destroys the enumeration so that the caller must
 * reenumerate the devices.
 *
 * \throws std::runtime_error if reset operation failed.
 */
void CVMUSBusb::resetVMUSB()
{
  int status = usb_reset(m_handle);
  if (status < 0) {
    throw std::runtime_error("CVMUSB::resetVMUSB() failed to reset the device");
  }
}


/*!
 * \brief Enumerate and locate the device by serial number
 *
 * It is expected that the serial number of the device has already been
 * specified. This will enumerate the USB busses and then locate the
 * device on it with a matching serial number. If found, the device is
 * stored by the class for later use.
 *
 * \throws std::string if no device with a matching serial number is found
 */
void CVMUSBusb::enumerateAndIdentify()
{
  // Since we might be re-opening the device we're going to
  // assume only the serial number is right and re-enumerate

  std::vector<struct usb_device*> devices = enumerate();
  m_device = 0;
  for (int i = 0; i < devices.size(); i++) {
    if (serialNo(devices[i]) == m_serial) {
      m_device = devices[i];
      break;
    }
  }
  if (!m_device) {
    std::string msg = "The VM-USB with serial number ";
    msg += m_serial;
    msg += " could not be enumerated";
    throw msg;
  }
}
