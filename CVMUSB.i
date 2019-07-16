/*
    This software is Copyright by the Board of Trustees of Michigan
    State University (c) Copyright 2014.

    You may use this software under the terms of the GNU public license
    (GPL).  The terms of this license are described at:

     http://www.gnu.org/licenses/gpl.txt

     Author:
       NSCL DAQ Development Team
	     NSCL
	     Michigan State University
	     East Lansing, MI 48824-1321
*/

%module CVMUSB
%include <stdint.i>
%include <std_string.i>

%ignore CVMUSB::executeList(CVMUSBReadoutList& list, void* pReadBuffer, size_t readBufferSize, size_t* bytesRead);
%ignore CVMUSBusb::executeList(CVMUSBReadoutList& list, void* pReadBuffer, size_t readBufferSize, size_t* bytesRead);

%{
  #include <CVMUSB.h> 
  #include <CVMUSBusb.h> 
  #include <CMockVMUSB.h> 

  class CTCLApplication;
  CTCLApplication *gpTCLApplication = 0;
%}

%include "CVMUSB.h"
%include "CVMUSBusb.h"
%include "CMockVMUSB.h"

