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
    running_ = 1;
    while (running_) {
        run_one_instruction();
        // scale speed?
    }
}



void BlinkenBoxInstance::run_one_instruction()
{
    // fetch one instruction and adjust PC
    byte *instruction = fetch_instruction_byte();

    if (instruction == 0) {
        return segfault(PC_);
    }

    //
    // some opcodes are only 4 bits wide, test for those first
    // 0xA*, 0xB*, 0xC*, 0xE*, 0x3*, 0x4*, 0x5*, 0x6*, 0x7*
    byte opcode = (*instruction) & 0xF0;
    if (opcode >= 0xA0 && opcode <= 0xC0 && opcode >= 0x30 && opcode <= 0x70) {
        byte d = (*instruction) & 0x0F;
        byte *Rd = &REG_[d];
        byte *k = fetch_instruction_byte();
        if (k == 0) { 
            return segfault(PC_);
        }

        switch (opcode) {
            case 0x30: //CPI 
            {
                byte result = *Rd - *k;
                BIT_SET(SREG_, SREG_N, BIT_VAL(*Rd, 7));
                BIT_SET(SREG_, SREG_Z, (result == 0));
                BIT_SET(SREG_, SREG_C, (*k > *Rd));
                return;
            }

            case 0x40: //SBCI
            {
                *Rd -= *k - BIT_VAL(SREG_, SREG_C);
                BIT_SET(SREG_, SREG_N, BIT_VAL(*Rd, 7));
                if (*Rd != 0) BIT_SET(SREG_, SREG_Z, 0);
                BIT_SET(SREG_, SREG_C, (*k + BIT_VAL(SREG_,SREG_C) > *Rd));
                return;
            }

            case 0x50: //SUBI
            {
                *Rd -= *k;
                BIT_SET(SREG_, SREG_N, BIT_VAL(*Rd, 7));
                BIT_SET(SREG_, SREG_Z, (*Rd == 0));
                BIT_SET(SREG_, SREG_C, (*k > *Rd));
                return;
            }

            case 0x60: //ORI
            {
                *Rd |= *k;
                BIT_SET(SREG_, SREG_N, BIT_VAL(*Rd, 7));
                BIT_SET(SREG_, SREG_Z, (*Rd == 0));
                return;
            }


            case 0x70: //ANDI
            {
                *Rd &= *k;
                BIT_SET(SREG_, SREG_N, BIT_VAL(*Rd, 7));
                BIT_SET(SREG_, SREG_Z, (*Rd == 0));
                return;
            }

            case 0xA0: //LDS
            {
                byte *v = access_memory( (REG_[0xF] << 8) | (*k) );
                *Rd = *v;
                return;
            }

            case 0xB0: //LDI
            {
                *Rd = *k;
                return;
            }

            case 0xC0: //STS
            {
                byte *v = access_memory( (REG_[0xF] << 8) | (*k) );
                *v = *Rd;
                return;
            }


            default:
                return invalid_opcode(*instruction);
        }
    }


    // these instructions all have no operand byte
    switch (*instruction) {
        case 0x00: // HALT
        {
            running_ = 0;
            return;
        }

        case 0xFF: // NOP
        {
            return;
        }

        case 0xDD: // RET
        {
            return;
        }

        case 0xD9: // ICALL
        {
            return;
        }

        case 0xDB: // IJMP
        {
            return;
        }
    }

    // remaining instructions all have an operand byte
    byte *operand = fetch_instruction_byte();
    byte d = (*operand & 0xF0) >> 4;
    byte r = (*operand & 0x0F);

    switch (*instruction) {
        case 0x2C: // MOV
            return opcode_mov(d, r);
        case 0x80: // LD
            return opcode_ld(d);
        case 0x82: // ST
            return opcode_st(d);
        case 0x05: // PUSH
            return opcode_push(d);
        case 0x01: // POP
            return opcode_pop(d);

        case 0x09: // LSL
            return opcode_lsl(d);
        case 0x1D: // LSR
            return opcode_lsr(d);
        case 0x98: // ROL
            return opcode_rol(d);
        case 0x99: // ROR
            return opcode_ror(d);
        case 0x11: // ASR
            return opcode_asr(d);
        case 0x06: // BCLR
            return opcode_bclr(r & 0x7);
        case 0x07: // BSET
            return opcode_bset(r & 0x7);

        case 0x10: // CPSE
            return opcode_cpse(d, r);
        case 0x14: // CP
            return opcode_cp(d, r);
        case 0x04: // CPC
            return opcode_cpc(d, r);
        case 0xD0: // RCALL
            return opcode_rcall(*operand);
        case 0xD1: // CALL
        {
            push_stack(PC_ & 0xFF);
            push_stack((PC_ & 0xFF00) >> 8);
            PC_ = *k;
            return;
        }

        case 0xD3: // RJMP
            return opcode_rjmp(*operand);
        case 0xD7: // JMP
            return opcode_jmp(*operand);
        case 0xF0: // BRCS
            return opcode_brcs(*operand);
        case 0xF1: // BREQ
            return opcode_breq(*operand);
        case 0xF2: // BRMI
            return opcode_brmi(*operand);
        case 0xF4: // BRLT
            return opcode_brlt(*operand);
        case 0xF8: // BRCC
            return opcode_brcc(*operand);
        case 0xF9: // BRNE
            return opcode_brne(*operand);
        case 0xFA: // BRPL
            return opcode_brpl(*operand);
        case 0xFC: // BRGE
            return opcode_brge(*operand);
        case 0xFD: // SBRC
            return opcode_sbrc(d, r & 0x7);
        case 0xFE: // SBRS
            return opcode_sbrs(d, r & 0x7);

        case 0x96: // DEC
            return opcode_dec(d);
        case 0x97: // INC
            return opcode_inc(d);
        case 0x9A: // MUL
            return opcode_mul(d, r);
        case 0x02: // MULS
            return opcode_muls(d, r);
        case 0x03: // MULSU
            return opcode_mulsu(d, r);
        case 0x08: // SBC
            return opcode_sbc(d, r);
        case 0x0C: // ADD
            return opcode_add(d, r);
        case 0x18: // SUB
            return opcode_sub(d, r);
        case 0x1C: // ADC
            return opcode_adc(d, r);

        case 0x20: // AND
            return opcode_and(d, r);
        case 0x24: // EOR
            return opcode_eor(d, r);
        case 0x28: // OR
            return opcode_or(d, r);
        case 0x91: // NEG
            return opcode_neg(d);
        case 0x94: // COM
            return opcode_com(d);
    }

    return invalid_opcode(*instruction);
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

byte *BlinkenBoxInstance::fetch_instruction_byte()
{
    byte *instruction = (byte *)access_memory(PC_);

    if (instruction == 0) {
        return 0;
    }
    PC_ += 1;
    return instruction;
}



void BlinkenBoxInstance::invalid_opcode(byte instruction)
{
}
void BlinkenBoxInstance::segfault(int address)
{
}



/*
 * opcode functions
 *
 */
void BlinkenBoxInstance::opcode_cpi(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_sbci(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_subi(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_ori(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_andi(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_lds(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_ldi(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_sts(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_sbr(byte d, byte k)
{
}

void BlinkenBoxInstance::opcode_halt()
{
}

void BlinkenBoxInstance::opcode_nop()
{
}

void BlinkenBoxInstance::opcode_ret()
{
}

void BlinkenBoxInstance::opcode_icall()
{
}

void BlinkenBoxInstance::opcode_ijmp()
{
}

void BlinkenBoxInstance::opcode_mov(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_ld(byte d)
{
}

void BlinkenBoxInstance::opcode_st(byte d)
{
}

void BlinkenBoxInstance::opcode_push(byte d)
{
}

void BlinkenBoxInstance::opcode_pop(byte d)
{
}

void BlinkenBoxInstance::opcode_lsl(byte d)
{
}

void BlinkenBoxInstance::opcode_lsr(byte d)
{
}

void BlinkenBoxInstance::opcode_rol(byte d)
{
}

void BlinkenBoxInstance::opcode_ror(byte d)
{
}

void BlinkenBoxInstance::opcode_asr(byte d)
{
}

void BlinkenBoxInstance::opcode_bclr(byte r)
{
}

void BlinkenBoxInstance::opcode_bset(byte r)
{
}

void BlinkenBoxInstance::opcode_cpse(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_cp(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_cpc(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_rcall(byte d)
{
}

void BlinkenBoxInstance::opcode_call(byte d)
{
}

void BlinkenBoxInstance::opcode_rjmp(byte d)
{
}

void BlinkenBoxInstance::opcode_jmp(byte d)
{
}

void BlinkenBoxInstance::opcode_brcs(byte d)
{
}

void BlinkenBoxInstance::opcode_breq(byte d)
{
}

void BlinkenBoxInstance::opcode_brmi(byte d)
{
}

void BlinkenBoxInstance::opcode_brlt(byte d)
{
}

void BlinkenBoxInstance::opcode_brcc(byte d)
{
}

void BlinkenBoxInstance::opcode_brne(byte d)
{
}

void BlinkenBoxInstance::opcode_brpl(byte d)
{
}

void BlinkenBoxInstance::opcode_brge(byte d)
{
}

void BlinkenBoxInstance::opcode_sbrc(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_sbrs(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_dec(byte d)
{
}

void BlinkenBoxInstance::opcode_inc(byte d)
{
}

void BlinkenBoxInstance::opcode_mul(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_muls(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_mulsu(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_sbc(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_add(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_sub(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_adc(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_and(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_eor(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_or(byte d, byte r)
{
}

void BlinkenBoxInstance::opcode_neg(byte d)
{
}

void BlinkenBoxInstance::opcode_com(byte d)
{
}

