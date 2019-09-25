/*
 * This program holds all of the functions utilized for the VM USB and the DAQ
 *
 *
 * Kolby Kiesling
 * kjk15b@acu.edu
 * 07 / 08 / 2019
 *
 *
 */

#include "CVMUSB.h"
#include "CVMUSBusb.h"
#include <cstdio>
#include "CVMUSBReadoutList.h"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include "vmeClass.h"

/*
 * I am lazy a lot of stuff is redundant on the VME bus, so I am going to make a lot of
 * definitions to call later in the program, please refer back to here for later aid.
 * All addresses are pulled from "Interfacing Mesytec VME Modules" Page 11.
 * This is an early design, I am copying the setup scripts provided from MVME, 
 * refer to the init-##-Module Init.vmescripts for help.
 * 
 * You will notice my syntax is close to what is in the user manuals, I tried to keep things
 * looking similar to help out...
 * 
 * Some registers are never interfaced with.
 * I should note that these are mainly for interfacing with the Mesytec VME devices, below I will append
 * stuff for the VM-USB specifically, this should hopefully help out with debugging in the future.
 */
// These commands are for reading and writing to the VME bus, need certain priviledges to perform specific tasks
#define ADDR_R 0x0D // supervisory read, I do not know if this changes anything
#define ADDR_W 0x0E // limited read command, probably good to keep like this right now
#define MBLT_ADDR_R 0x0F // Supervisory BLT read, for stack execution
// Mesytec Addresses
#define soft_reset 0x6008
#define firmware_revision 0x600E
#define marking_type 0x6038 // event counter, timestamp, extended timestamp
#define irq_level 0x6010
#define irq_vector 0x6012 // probably should not change from 0
#define IRQ_source 0x601C // 0 for event threshold and 1 for data threshold
#define irq_event_threshold 0x601E // should always be 1
#define multi_event 0x6036 // 0->single event, 0x3-> multi event, 0xb->transmits # events specified
#define Max_transfer_data 0x601A // one event is transmitted, should always be 1
#define cblt_mcst_control 0x6020 // Setup Multicast, DO NOT TOUCH
#define cblt_address 0x6024 // set 8 high bits of Multicast address, DO NOT TOUCH
#define output_format 0x6044 // 0->standard, 1->timestamp
#define bank_operation 0x6040 // 0->bank connected, 1->independent
#define tdc_resolution 0x6042 // timing resolution, refer to user manuals
#define first_hit 0x6052 // 0bRT-> R: bank0, T: bank1, 1->transmit first hit, 0->transmit all hits
#define bank0_win_start 0x6050 // leave at 16368 unless you know what you are doing
#define bank1_win_start 0x6052 // see above
#define bank0_win_width 0x6054 // ns of bin width (32 normally)
#define bank1_win_width 0x6056
#define bank0_trig_source 0x6058 // 0x001 leaves in split bank mode
#define bank1_trig_source 0x605A
#define Negative_edge 0x6060 // DO NOT CHANGE
#define bank0_input_thr 0x6078 // leave at 105 unless you know what you are doing
#define bank1_input_thr 0x607A
#define ECL_term 0x6062 // to turn terminators on (APPLT TO MQDC ONLY)
#define ECL_trig1_osc 0x6064
#define Trig_select 0x6068
#define NIM_trig1_osc 0x606A
#define NIM_busy 0x606E
#define pulser_status 0x6070
#define ts_sources 0x6096 // VME source for frequency (DO NOT CHANGE)
#define ts_divisor 0x6098
#define stop_ctr 0x60AE
#define ECL_gate1_osc 0x6064 // MQDC32 stuff now plenty of overlap in registers
#define ECL_fc_res 0x6066 // a lot of this stuff just straight up should not be touched
#define Gate_select 0x6068 // unless of course you know what you are doing
#define NIM_gat1_osc 0x606A
#define NIM_fc_reset 0x606C
#define pulser_dac 0x6072
#define chn0 0x4000 // channels used for setting thresholds on the MQDC32
#define chn1 0x4002 // Again I am lazy so I made them all definitions, this is so people
#define chn2 0x4004 // can actually come back and half-read what I wrote instead of seeing
#define chn3 0x4006 // a lot of cryptic hexadecimal addresses
#define chn4 0x4008
#define chn5 0x400A
#define chn6 0x400C
#define chn7 0x400E
#define chn8 0x4010
#define chn9 0x4012
#define chn10 0x4014
#define chn11 0x4016
#define chn12 0x4018
#define chn13 0x401A
#define chn14 0x401C
#define chn15 0x401E
#define chn16 0x4020
#define chn17 0x4022
#define chn18 0x4024
#define chn19 0x4026
#define chn20 0x4028
#define chn21 0x402A
#define chn22 0x402C
#define chn23 0x402E
#define chn24 0x4030
#define chn25 0x4032
#define chn26 0x4034
#define chn27 0x4036
#define chn28 0x4038
#define chn29 0x403A
#define chn30 0x403C
#define chn31 0x403E
#define ignore_thresholds 0x604C
#define offset_bank0 0x6044
#define offset_bank1 0x6046
#define limit_bank0 0x6050
#define limit_bank1 0x6052
#define trig_delay0 0x6054
#define trig_delay1 0x6056
#define input_coupling 0x6060
#define skip_oorange 0x604A
#define start_acq 0x603A // I am still copying the syntax of what they are declared as in the manual for reference.
#define reset_ctr_ab 0x6090
#define FIFO_reset 0x603C
#define readout_reset 0x6034
#define base_addr 0xbb000000
#define module_id 0x6004
#define data_reg 0x6030
#define fifo_threshold 0x6018
#define MTDC 0x06060000
#define MQDC 0x01060000

/*
 * VM-USB registers and settings. Hopefully when this is all finished, I will have left all of my notepads
 * somewhere near the DAQ for future reference. For all of the VM-USB stuff refer to the gray Gemline pad.
 * That has the process I went through for coming up with all of the register statuses for the VME. This
 * also goes in order if you are reading the VM-USB manual as well.
 */
#define GLOBAL_SETTINGS 0x0099 // Aligning data and mixing in IRQ data, (We want IRQ)
#define DAQ_SETTINGS 0x000000EF // Sets the delay length for data acquisition, randomly chosen...
#define LED_SETTINGS 0x13111010 // TY: USBFIFOOut not empty, R: Event Trigger, G: Stack not empty, BY: VME BERR
#define USR_DEV_SETTINGS 0x00001110 // Setup the NIM outputs for the VME, O1-> Busy (stack processing), O2 -> Data sent to buffer
#define DDG_SETTINGS 0x007F000F // Some easy delays and gates, feel free to change
#define SCALAR_RESET 0x00880000 // reset the scalar readouts after execution of a stack
#define EVENTSBUFF_SETTINGS 0x001 // Set VM-USB to handle one event at a time
#define ISV_SETTINGS 0x218F218F // Set the IRQ to look for stack labled 2
#define USB_SETTINGS 0x00000502 // Set timeout (5sec) and packet transmit from buffer (2)
#define AR_IRQ 0xFF00 // Action registers access
#define AR_DAQ_START 0x0001
#define AR_DAQ_STOP 0x0000
#define firmwareReg 0x00
#define gmodeReg 0x04
#define daqReg 0x08
#define ledReg 0x0C
#define usrDevReg 0x10
#define ddgAReg 0x14
#define ddgBReg 0x18
#define ddgExtReg 0x38
#define scalar_A 0x1C
#define scalar_B 0x20
#define eventBuffReg 0x24
#define ISVReg 0x28
#define USBReg 0x3C
#define EVENT_MARKER 0xBDE7
#define TIMEOUT 200


/*
 * vme::moduleReset
 * This function performs a soft powercycle on a module and
 * reads back the firmware and hardware ID.
 */
int
vme::moduleReset (uint32_t module_addr, CVMUSBusb* cvm) {
    static uint16_t w_data=1;
    static uint16_t r_data=0;
    cvm->vmeWrite16(module_addr|soft_reset, ADDR_W, w_data);
    printf("\n--------------------\nSoft Reset Starting\n--------------------\n");
    usleep(400); // sleep for a short while to be sure the write was successful, this is only noted for powercycling
    cvm->vmeRead16(module_addr|soft_reset, ADDR_R, &r_data);
    printf("Module ID:\t\t0x%0x\n", r_data);
    cvm->vmeRead16(module_addr|firmware_revision, ADDR_R, &r_data);
    printf("Firmware Revision:\t0x%0x\n", r_data);
    printf("\n--------------------\nSoft Reset Finished\n--------------------\n");
    return 0;
}


/*
 * vme::vmUSBInit
 * This function initializes the VM USB for DAQ mode. This process is copied from the method used
 * with the MVME, feel free to change if you know what you are doing. Mostly everything is zeros.
 */

int
vme::vmUSBInit (CVMUSBusb* cvm) {
    unsigned int reg[10]={gmodeReg, daqReg, ledReg, usrDevReg, ddgAReg, ddgBReg, ddgExtReg, eventBuffReg, ISVReg, USBReg};
    uint32_t data[10]={GLOBAL_SETTINGS, DAQ_SETTINGS, LED_SETTINGS, USR_DEV_SETTINGS, DDG_SETTINGS, DDG_SETTINGS, DDG_SETTINGS, EVENTSBUFF_SETTINGS, ISV_SETTINGS, USB_SETTINGS};
    printf("\n--------------------\nInitializing VM-USB\n--------------------\n");
    for (int i=0;i<10;++i) {
      cvm->writeRegister(reg[i], data[i]);
      printf(".\t");
    }
    static uint16_t r_data=0;
    r_data = cvm->readRegister(0x28);
    printf("\n--------------------\nVector Zero: %02x\n--------------------\n", r_data);
    printf("\n--------------------\nFinished initializing VM-USB\n--------------------\n");
    
    vme::registerDump(cvm);
    return 0;
}


/*
 * vme::mvmeInit 
 * This function initializes the Mesytec VME devices to communicate through the VME interface.
 * Be sure to call this after a power cycle, appeared to be standard practice with NSCL & MVME
 */
int
vme::mvmeInit (uint32_t module_addr, CVMUSBusb* cvm) {
    static uint16_t reg[9] = {irq_level, irq_vector, IRQ_source, irq_event_threshold, marking_type, multi_event, Max_transfer_data, cblt_mcst_control, cblt_address};
    static uint16_t reg_data[9] = {1, 0, 0, 1, 0x1, 0x0, 0, 0x80, 0xBB};
    printf("\n--------------------\nStarting VME Interfacing\n--------------------\n");
    for (int i=0;i<9;++i) {
      cvm->vmeWrite16(module_addr|reg[i], ADDR_W, reg_data[i]); 
      printf(".\t");
      usleep(200);
    }
    printf("\n--------------------\nVME Interfacing Finished\n--------------------\n");
    return 0;
}


/*
 * vme::moduleInit
 * This function initializes the Mesytec VME devices internally. See MVME and technical notes for more clarity.
 * These parameters are all standard an have not been changed, IRQ is the only thing changed which is why it is in a separate function.
 */
int
vme::moduleInit (uint32_t module_addr, CVMUSBusb* cvm) {
  static uint16_t mqdc_reg[53]={ECL_term, ECL_gate1_osc, ECL_fc_res, Gate_select, NIM_gat1_osc, NIM_fc_reset, NIM_busy, pulser_status, pulser_dac, ts_sources, ts_divisor, chn0, chn1, chn2, chn3, chn4, chn5, chn6, chn7, chn8, chn9, chn10, chn11, chn12, chn13, chn14, chn15, chn16, chn17, chn18, chn19, chn20, chn21, chn22, chn23, chn24, chn25, chn26, chn27, chn28, chn29, chn30, chn31, ignore_thresholds, bank_operation, offset_bank0, offset_bank1, limit_bank0, limit_bank1, trig_delay0, trig_delay1, input_coupling, skip_oorange};
  static uint16_t mqdc_data[53]={0b11000, 0, 1, 0, 0, 0, 0, 5 /*5PULSER*/, 32, 0b00, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 130, 130, 255, 255, 0, 0, 0b000, 0};

  static uint16_t mtdc_reg[22]={output_format, tdc_resolution, first_hit, bank0_win_start, bank1_win_start, bank0_win_width, bank1_win_width, bank0_trig_source, bank1_trig_source, Negative_edge, bank0_input_thr, bank1_input_thr, ECL_term, ECL_trig1_osc, Trig_select, NIM_trig1_osc, NIM_busy, pulser_status, ts_sources, ts_divisor, stop_ctr};
  static uint16_t mtdc_data[22]={0, 0, 3, 0b11, 16368, 16368, 32, 32, 0x001, 0x002, 0b00, 105, 105, 0b000, 0, 0, 0, 0, 3 /*3PULSER*/, 0b00, 1, 0b00};

  static uint16_t r_data=0;
  cvm->vmeRead16(module_addr|firmware_revision, ADDR_R, &r_data);

  printf("\n--------------------\nStarting initialization process\n--------------------\n");
  
  if (r_data == 0x204) {
    for (int i=0;i<53;++i) {
      cvm->vmeWrite16(module_addr|mqdc_reg[i], ADDR_W, mqdc_data[i]);
      usleep(200);
      printf(".\t");
    }
  }
  
  if (r_data == 0x206) {
    for (int i=0;i<22;++i) {
      cvm->vmeWrite16(module_addr|mtdc_reg[i], ADDR_W, mtdc_data[i]);
      usleep(200);
      printf(".\t");
    }
  }
  
  printf("\n--------------------\nFinished initialization process\n--------------------\n");
}


/*
 * vme::daqInit
 * This function writes to a write only register.
 * The action register at bit 0 starts acquisition, refer to page 16 in the Wiener VM-USB manual for more help.
 */
int
vme::daqInit (CVMUSBusb* cvm) {
  static uint16_t s_data[5]={0, 3, 1, 1, 1};
  static uint16_t reg_addr[5]={start_acq, reset_ctr_ab, FIFO_reset, start_acq, readout_reset};
  printf("\n--------------------\nInitializing Data Acquisition\n--------------------\n");
  
  for (int i=0;i<5;++i) {
    cvm->vmeWrite16(base_addr|reg_addr[i], ADDR_W, s_data[i]);
    printf(".\t");
  }
  return 0;
}



/*
 * 
 * 
 * 
 */
int 
vme::daqStart (CVMUSBusb* cvm) {
  printf("\n--------------------\nStarting Data Acquisition\n--------------------\n");
  cvm->writeActionRegister(AR_IRQ|AR_DAQ_STOP); // stop DAQ and hold IRQ
  cvm->writeActionRegister(AR_IRQ|AR_DAQ_START); // start DAQ and hold IRQ  
}
/*
 * vme::daqStop
 * This function writes to a write only register
 * Writing a 0 to bit 0 in the action register halts the acquisition process.
 */
int
vme::daqStop (CVMUSBusb* cvm) {
  printf("\n--------------------\nStopping Data Acquisition\n--------------------\n");
  cvm->writeActionRegister(AR_IRQ|AR_DAQ_STOP);
  return 0;
}


/*
 * vme::cycleStart
 * This function adds the reading FIFO to the stack to be executed later.
 * Cycle functions are being broken up so it is easier to keep track of what is happening.
 */
int
vme::cycleStart (uint32_t module_addr, CVMUSBReadoutList* list) {
  char buffer[64];
  memset(buffer, 0, sizeof(buffer));
  list->addFifoRead32(module_addr, MBLT_ADDR_R, sizeof(buffer));
  printf("\n--------------------\nAdding to stack\n--------------------\n");
  return 0;
}


/*
 * vme::cycleReadout
 * This function is primarily for debugging. To show the size of the stack, mostly to acknowledge that
 * data is being properly written to the stack.
 */
int 
vme::cycleReadout (CVMUSBReadoutList* list) {
  printf("\n--------------------\nStack Size: %d\n--------------------\n", list->size());
  return 0;
}


/*
 * vme::cycleReset
 * This function adds a reset write to the Mesytec modules to the stack. This should ALWAYS be added to the stack last.
 * Execution of this should come before end of cycle and after acquisition.
 */
int 
vme::cycleReset (CVMUSBReadoutList* list) {
  static uint16_t data=1;
  list->addWrite16(base_addr|readout_reset, ADDR_W, data);
  printf("\n--------------------\nAdding to stack\n--------------------\n");
  return 0;
}


/*
 * vme::cycleClear
 * This function clears the stack and returns the stack size. This is intended for both debugging and 
 * end of event cycles. To always be cautious and clear out stacks, and build stacks from scratch.
 */
int 
vme::cycleClear (CVMUSBReadoutList* list) {
  printf("\n--------------------\nClearing Stack\n--------------------\n");
  list->clear();
  printf("\n--------------------\nStack Size: %d\n--------------------\n", list->size());
  return 0;
}


/*
 * vme::cycleExecute
 * This function executes the stack built to read back data from the VM USB.
 * In heavy debugging mode right now.
 */
int 
vme::cycleExecute (CVMUSBusb* cvm, CVMUSBReadoutList& list, unsigned long* datumA, unsigned long* datumB) {
  printf("\n--------------------\nExecuting stack\n--------------------\n");
  char buffer[2048];
  memset(buffer, 0, sizeof(buffer));
  unsigned long nBytesRead=0;
  unsigned int md_id[2048]={0};
  unsigned int chn_id[2048]={0};
  unsigned int md_sttng[2048]={0};
  
  cvm->executeList(list, buffer, sizeof(buffer), &nBytesRead);
  printf("\n--------------------\n");
  for (int i=0;i<nBytesRead;++i) {
    printf("%02x | ", buffer[i]);
    md_id[i] = (buffer[i] & 0x00FF0000) / 0x10000;
    chn_id[i] = (buffer[i] & 0x003F0000) / 0x010000;
    md_sttng[i] = (buffer[i] & 0x0000FC00) / 0x400;
  }
  for (int i=0;i<nBytesRead;++i) {
    printf("\nIndex: %d\tID: 0x%0x\tChannel: 0x%0x\tSettings: 0x%0x\n",i, md_id[i], chn_id[i], md_sttng[i]);
  }
  printf("\n--------------------\n");
  return 0;
}


/*
 * vme::pollIRQ 
 * Used mainly for debugging, may get dropped altogether...
 * 
 */
int 
vme::pollBuffer (CVMUSBusb* cvm, uint32_t module_addr) {
  printf("\n--------------------\nPolling Data Buffer\n--------------------\n");
  static uint16_t r_data=0;
  printf("Register Polling:\t%02x", module_addr);
  cvm->vmeRead16(module_addr|data_reg, ADDR_R, &r_data);
  printf("\n--------------------\nData Buffer: %d\n--------------------\n", r_data);
  return r_data;
}


/*
 * vme::registerDump
 * This routine dumps the core registers of the VME to the console, for debugging purposes
 * really.
 */
int 
vme::registerDump (CVMUSBusb* cvm) {
  printf("\n--------------------\nDumping Core Registers\n--------------------\n");
  unsigned int reg[11]={0, gmodeReg, daqReg, ledReg, usrDevReg, ddgAReg, ddgBReg, ddgExtReg, eventBuffReg, ISVReg, USBReg};
  uint32_t r_data=0;
  for (int i=0;i<11;i++) {
    r_data = cvm->readRegister(reg[i]);
    printf("Register: %d\t\tData: %20x\n", reg[i], r_data);
  }
}


/*
 * vme::buildStack
 * This function builds the stack, change it as you see necessary. These things are here in order for how I want to read
 * out data. You want to always end too with resetting the modules. Debugging is done by checking stack size, everything returns void for CVMUSBReadoutList
 */
bool
vme::buildStack (CVMUSBusb* cvm, CVMUSBReadoutList* list) {
  printf("\n--------------------\nBuilding Stack\n--------------------\n");
  char buffer[128];
  static unsigned int status=0;
  static unsigned int stk_size=0;
  static uint16_t data=1;
  memset(buffer, 0, sizeof(buffer));
  list->addMarker(EVENT_MARKER); // add event marker
  cvm->setDefaultTimeout(TIMEOUT); // add default timeout to transaction (200ms)
  list->addFifoRead16(MTDC, MBLT_ADDR_R, sizeof(buffer)); // read from MTDC
  list->addFifoRead16(MQDC, MBLT_ADDR_R, sizeof(buffer)); // read from MQDC
  list->addRegisterRead(scalar_A); // read from out scalar A
  list->addRegisterRead(scalar_B); // read from out scalar B
  list->addWrite16(base_addr|readout_reset, ADDR_W, data); // reset Mesytec modules
  list->addRegisterWrite(usrDevReg, USR_DEV_SETTINGS|SCALAR_RESET); // reset scalars
  printf("\n--------------------\nStack Size: %d\n--------------------\n", list->size());
  return true;
}


bool
vme::testStack (CVMUSBusb* cvm, CVMUSBReadoutList* list) {
  printf("\n--------------------\nBuilding Stack\n--------------------\n");
  char buffer[128];
  static unsigned int status=0;
  static unsigned int stk_size=0;
  static uint16_t data=1;
  memset(buffer, 0, sizeof(buffer));
  list->addMarker(EVENT_MARKER); // add event marker
  cvm->setDefaultTimeout(TIMEOUT); // add default timeout to transaction (200ms)
  list->addFifoRead16(MQDC, MBLT_ADDR_R, sizeof(buffer)); // read from MQDC
  list->addWrite16(base_addr|readout_reset, ADDR_W, data); // reset Mesytec modules
  printf("\n--------------------\nStack Size: %d\n--------------------\n", list->size());
  return true;
}

/*
 * 
 * 
 * 
 */
int 
vme::testMask (uint32_t module_addr, CVMUSBusb* cvm, CVMUSBReadoutList& list) {
  printf("\n--------------------\nExecuting stack\n--------------------\n");
  char buffer[2048];
  memset(buffer, 0, sizeof(buffer));
  unsigned long nBytesRead=0;
  unsigned int md_id[2048]={0};
  unsigned int chn_id[2048]={0};
  unsigned int md_sttng[2048]={0};
  
  cvm->executeList(list, buffer, sizeof(buffer), &nBytesRead);
  printf("\n--------------------\n");
  for (int i=0;i<nBytesRead;++i) {
    printf("%02x | ", buffer[i]);
    md_id[i] = (buffer[i] & 0x00FF0000) / 0x10000;
    chn_id[i] = (buffer[i] & 0x003F0000) / 0x010000;
    md_sttng[i] = (buffer[i] & 0x0000FC00) / 0x400;
  }
  for (int i=0;i<nBytesRead;++i) {
    printf("\nIndex: %d\tID: 0x%0x\tChannel: 0x%0x\tSettings: 0x%0x\n",i, md_id[i], chn_id[i], md_sttng[i]);
  }
  printf("\n--------------------\n");
  return 0;
}
