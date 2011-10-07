#ifndef BLINKENBOXINSTANCE_H
#define BLINKENBOXINSTANCE_H

#include <WProgram.h>


#define BIT_SET(w, b)   (w) |= 1<<(b)
#define BIT_CLR(w, b)   (w) &= ~(1<<(b))
#define BIT_VAL(w, b)   (((w) & (1<<(b))) >> (b))

#define BIT_UP(w, b, v)   if ((v)) BIT_SET((w),(b)); else BIT_CLR((w),(b))

#define SREG_C   0
#define SREG_Z   1
#define SREG_N   2
#define SREG_V   3
#define SREG_S   4


#define REG_Z    ((REG_[0xF] << 8) | REG_[0xE])


// set SREG_V if a  two's complement overflow resulted from the operation; cleared otherwise.
#define UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(Rd, K, R)         BIT_UP(SREG_, SREG_V, \
    (BIT_VAL((Rd), 7) & ~BIT_VAL((K), 7) & ~BIT_VAL((R), 7)) \
  | (~BIT_VAL((Rd), 7) & BIT_VAL((K), 7) & BIT_VAL((R), 7)))
// N xor C (for N and C after the shift)
#define UPDATE_SREG_OVERFLOW_NXORC()      BIT_SET(SREG_, SREG_S, BIT_VAL(SREG_, SREG_N) ^ BIT_VAL(SREG_, SREG_C))
// set SREG_C if the absolute value of K is larger than the absolute value of Rd; cleared othwerise.
#define UPDATE_SREG_CARRY(Rd, K, R)             BIT_UP(SREG_, SREG_C, \
    (~BIT_VAL((Rd), 7) & BIT_VAL((K), 7)) \
  | (BIT_VAL((K), 7) & BIT_VAL((R), 7)) \
  | (BIT_VAL((R), 7) & BIT_VAL((Rd), 7)))
//set if the result is 0; cleared otherwise.
#define UPDATE_SREG_ZERO(R)     BIT_UP(SREG_, SREG_Z, (R) == 0)
// set if MSB of the result is set; cleared otherwise.
#define UPDATE_SREG_NEG(R)      BIT_UP(SREG_, SREG_N, BIT_VAL((R),7))
// N xor V, for signed tests.
#define UPDATE_SREG_SIGN()      BIT_SET(SREG_, SREG_S, BIT_VAL(SREG_, SREG_N) ^ BIT_VAL(SREG_, SREG_V))

//previous value remains unchanged when the result is zero; cleared otherwise.
#define UPDATE_SREG_ZERO_IFCHANGED(R)      if ((R) != 0) BIT_CLR(SREG_, SREG_Z)


class BlinkenBoxInstance {

  public:
    BlinkenBoxInstance();
    ~BlinkenBoxInstance();
    
    void run();
    
  private:
    bool running_;

    int SP_;
    int PC_;
    char R_[16];        // 16x 8bit registers
    char SREG_;         // special register
    byte *MEMORY_[16];  // 16x possible memory pages
    
    
    

    
    void *access_memory(int address);
    void *access_system_memory(int offset);
    byte *fetch_instruction_byte();
    void push_stack(byte value);
    byte pop_stack();

    void run_one_instruction();

    void invalid_opcode(byte opcode);
    void segfault(int address);




};

#endif


