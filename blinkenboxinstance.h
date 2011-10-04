#ifndef BLINKENBOXINSTANCE_H
#define BLINKENBOXINSTANCE_H

#include <WProgram.h>

class BlinkenBoxInstance {

  public:
    BlinkenBoxInstance();
    ~BlinkenBoxInstance();
    
    void run();
    
  private:
    int SP_;
    int PC_;
    char R_[16];   // 16 8bit registers
    char SREG_;
    
    byte *MEMORY_[16];  // 16 possible memory pages
    
    
    

    
    void *access_memory(int address);
    void *access_system_memory(int offset);
    
};

#endif


