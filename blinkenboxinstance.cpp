#include "blinkenboxinstance.h"

#define PAGE_SIZE 256
#define USER_PAGE0_RAM_OFFSET RAMEND-(PAGE_SIZE*2)
#define USER_PAGE1_RAM_OFFSET RAMEND-(PAGE_SIZE)

BlinkenBoxInstance::BlinkenBoxInstance() 
{
  int i;
  for (i=0; i < 16; i++) {
    MEMORY_[i] = 0;
  }
  //map user memory pages
  MEMORY_[0] = (byte*)USER_PAGE0_RAM_OFFSET;
  MEMORY_[1] = (byte*)USER_PAGE1_RAM_OFFSET;

  //initialize the internal registers
  SP_ = 0x01FF;
  PC_ = 0x0000; 

}

BlinkenBoxInstance::~BlinkenBoxInstance() 
{
}

void BlinkenBoxInstance::run()
{
}

void *BlinkenBoxInstance::access_memory(int address)
{
  char page = (address & 0xFF00) >> 8;
  if (page == 0xF) {  // system page
    return access_system_memory(address & 0xFF);
  }
  
  if (MEMORY_[page] == 0) // unmapped page
    return NULL;
    
  return MEMORY_[page] + (address & 0xFF);  // return a pointer into mapped memory page
}

void *BlinkenBoxInstance::access_system_memory(int offset)
{
  // F0..FF map to the registers
  if (offset >= 0xF0) {
    return &R_[(offset - 0xF0)];
  }
  
  switch (offset) {
    case 0xE0: //SPL
      return (byte*)(&SP_)+1;
    case 0xE1: //SPH
      return (byte*)(&SP_);
  }  

}


