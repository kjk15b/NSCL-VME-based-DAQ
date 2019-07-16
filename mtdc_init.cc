/* 
 * This program initializes and starts the pulser on the MTDC32
 * The program is still in debugging phase
 * 
 * Kolby Kiesling
 * kjk15b@acu.edu
 * 07 / 02 / 2019
 * 
 * This program mimics the reset.vmescript provided with the templates for MVMVE
 * This program resets the MTDC and fetches the hardware ID & Firmware
 */

#include "CVMUSB.h"
#include "CVMUSBusb.h"
#include <cstdio>
#include "CVMUSBReadoutList.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include "vmeClass.h"
//#include <arpa/inet.h>
//#include <stdio.h>
#define MTDC 0x06060000
#define MQDC 0x01060000
#define ADDR_R 0x0D
#define ADDR_W 0x0E

int main() {
    vme VME;
    
    std::vector<struct usb_device*> devices = CVMUSB::enumerate(); // attempt to connect to VMUSB
    std::cout << "--------------------\n" << "Found " << devices.size() << " CVMUSB device(s)\n" << "--------------------" << std::endl;
    if (devices.size() <= 0) {
      return -1; // not found any devices
    
    }
    else{
      
      std::cout << CVMUSB::serialNo(devices[0]) << "\n--------------------" << std::endl; // VM0327 is the serial No.
  
      CVMUSBusb cvm (devices[0]);
      CVMUSBReadoutList list;
      CVMUSBReadoutList testList;
      unsigned long datumA, datumB; // dummy variables for debugging
      static const uint8_t listNumber=2;
      uint16_t data_buffer=0;
      uint32_t r_data=0;
      
      
      if(VME.testStack (&cvm, &testList)) {
      //cvm.loadList (listNumber, testList, sizeof(testList));
      
      VME.moduleReset (MTDC, &cvm); // soft power cycle the modules
      VME.moduleReset (MQDC, &cvm);
      
      VME.moduleInit (MTDC, &cvm); // initialize the Mesytec modules interface
      VME.moduleInit (MQDC, &cvm);
      
      VME.mvmeInit (MTDC, &cvm); // initialize the Mesytec modules for VM USB interface
      VME.mvmeInit (MQDC, &cvm);
      
      VME.vmUSBInit (&cvm);
      
      bool data_search = false;
      int data_count = 0;
      
      while (!data_search) {
	  data_buffer = VME.pollBuffer (&cvm, MQDC);
	  if (data_buffer > 0) {
	    printf("Found Data...");
	    //VME.testMask(MQDC, &cvm, testList);
	    cvm.vmeRead32(MQDC, ADDR_R, &r_data);
	    printf("\nData collected:\t\t0x%0x\n",r_data);
	    printf("Module ID:\t0x%0x\n", ((r_data&0x00FF0000) / 0x10000));
	    data_count ++;
	    if (data_count == 4) data_search = true;
	  }
	  usleep(100000);
	}

	VME.daqStop (&cvm);
	list.dump (std::cout);
	VME.cycleClear (&list);
	VME.moduleReset (MTDC, &cvm);
	VME.moduleReset (MQDC, &cvm);
	
      }
      else {
	return -1;
      }
    }
    return 0;
}
