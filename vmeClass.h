/*
 * This file defines classes and routines to abstract the VME DAQ
 * Comments and a README will be implemented later to better explain ways
 * to utilize this program
 * 
 *
 * Kolby Kiesling
 * kjk15b@acu.edu
 * 07 / 03 / 2019
 * 
 */

#ifndef vmeClass_H
#define vmeClass_H

#include "CVMUSB.h"
#include "CVMUSBusb.h"
#include <cstdio>
#include "CVMUSBReadoutList.h"
#include <iostream>
#include <cstring>
#include <unistd.h>


class vme // class for streamlining interfacing with VME modules
{
public:
  
  virtual int vmUSBInit (CVMUSBusb* cvm);
  
  virtual int moduleReset (uint32_t module_addr, CVMUSBusb* cvm);
  
  virtual int mvmeInit (uint32_t module_addr, CVMUSBusb* cvm);
  
  virtual int moduleInit (uint32_t module_addr, CVMUSBusb* cvm);
  
  virtual int daqStart (CVMUSBusb* cvm);
  
  virtual int daqInit (CVMUSBusb* cvm);
  
  virtual int daqStop (CVMUSBusb* cvm);
  
  virtual int cycleStart (uint32_t module_addr, CVMUSBReadoutList* list);
  
  virtual int cycleReadout (CVMUSBReadoutList* list);
  
  virtual int cycleReset (CVMUSBReadoutList* list);
  
  virtual int cycleClear (CVMUSBReadoutList* list);
  
  virtual int cycleExecute (CVMUSBusb* cvm, CVMUSBReadoutList& list, unsigned long* datumA, unsigned long* b);
  
  virtual int pollBuffer (CVMUSBusb* cvm, uint32_t module_addr);
  
  virtual int registerDump (CVMUSBusb* cvm);
  
  virtual bool buildStack (CVMUSBusb* cvm, CVMUSBReadoutList* list);  
  
  virtual bool testStack (CVMUSBusb* cvm, CVMUSBReadoutList* list);
  
  virtual int testMask (uint32_t module_addr, CVMUSBusb* cvm, CVMUSBReadoutList& list);
  
  
};

#endif