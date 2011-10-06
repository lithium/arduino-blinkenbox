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

    void run_one_instruction();

    void invalid_opcode(byte opcode);
    void segfault(int address);




    void opcode_cpi(byte d, byte k);
    void opcode_sbci(byte d, byte k);
    void opcode_subi(byte d, byte k);
    void opcode_ori(byte d, byte k);
    void opcode_andi(byte d, byte k);
    void opcode_lds(byte d, byte k);
    void opcode_ldi(byte d, byte k);
    void opcode_sts(byte d, byte k);
    void opcode_sbr(byte d, byte k);
    void opcode_halt();
    void opcode_nop();
    void opcode_ret();
    void opcode_icall();
    void opcode_ijmp();
    void opcode_mov(byte d, byte r);
    void opcode_ld(byte d);
    void opcode_st(byte d);
    void opcode_push(byte d);
    void opcode_pop(byte d);
    void opcode_lsl(byte d);
    void opcode_lsr(byte d);
    void opcode_rol(byte d);
    void opcode_ror(byte d);
    void opcode_asr(byte d);
    void opcode_bclr(byte r);
    void opcode_bset(byte r);
    void opcode_cpse(byte d, byte r);
    void opcode_cp(byte d, byte r);
    void opcode_cpc(byte d, byte r);
    void opcode_rcall(byte d);
    void opcode_call(byte d);
    void opcode_rjmp(byte d);
    void opcode_jmp(byte d);
    void opcode_brcs(byte d);
    void opcode_breq(byte d);
    void opcode_brmi(byte d);
    void opcode_brlt(byte d);
    void opcode_brcc(byte d);
    void opcode_brne(byte d);
    void opcode_brpl(byte d);
    void opcode_brge(byte d);
    void opcode_sbrc(byte d, byte r);
    void opcode_sbrs(byte d, byte r);
    void opcode_dec(byte d);
    void opcode_inc(byte d);
    void opcode_mul(byte d, byte r);
    void opcode_muls(byte d, byte r);
    void opcode_mulsu(byte d, byte r);
    void opcode_sbc(byte d, byte r);
    void opcode_add(byte d, byte r);
    void opcode_sub(byte d, byte r);
    void opcode_adc(byte d, byte r);
    void opcode_and(byte d, byte r);
    void opcode_eor(byte d, byte r);
    void opcode_or(byte d, byte r);
    void opcode_neg(byte d);
    void opcode_com(byte d);
    
};

#endif


