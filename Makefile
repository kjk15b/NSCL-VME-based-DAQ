
all: libCVMUSBusb_minimal.a mtdc_init

%.o: %.cpp
	g++ -g -O2 -std=c++11 -I. -c $^


libCVMUSBusb_minimal.a: CMutex.o  CSemaphore.o  CVMUSB.o  CVMUSBReadoutList.o  CVMUSBusb.o ErrnoException.o Exception.o os.o vmClass.o
	ar rc $@ $^

clean:
	rm -f libCVMUSBusb_minimal.a *.o mtdc_init


mtdc_init: mtdc_init.cc libCVMUSBusb_minimal.a
	g++ -std=c++11 $@.cc -g -O -o $@  -lpthread -lcrypt -fpermissive -lusb -I. libCVMUSBusb_minimal.a libCVMUSBusb_minimal.a
	
