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
#ifdef ARDUINO
  MEMORY_[0] = (byte*)USER_PAGE0_RAM_OFFSET;
  MEMORY_[1] = (byte*)USER_PAGE1_RAM_OFFSET;
#else
  MEMORY_[0] = (byte*)malloc(256);
  MEMORY_[1] = (byte*)malloc(256);
#endif

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

    // test for opcodes with 4 bit instructions, 4 bit register, and 8 bit constant operand
    // 0x40 .. 0xBF
    byte opcode = (*instruction) & 0xF0;
    if (opcode >= 0x40 && opcode <= 0xBF) {
        byte d = (byte)(*instruction) & 0x0F;
        byte *Rd = &REG_[d];
        byte *k = (byte*)fetch_instruction_byte();
        if (k == 0) { 
            return segfault(PC_);
        }

        switch (opcode) {
            case 0x40: //LDI
            {
                *Rd = *k;
                return;
            }

            case 0x50: //LDD
            {
                int addr = REG_Z + *k;
                byte *mem = (byte*)access_memory(addr);
                if (!mem)
                    return segfault(addr);
                *Rd = *mem;
                return;
            }

            case 0x60: //STD
            {
                int addr = REG_Z + *k;
                byte *mem = (byte*)access_memory(addr);
                if (!mem)
                    return segfault(addr);
                *mem = *Rd;
                return;
            }

            case 0x70: //ORI
            {
                *Rd |= *k;

                UPDATE_SREG_ZERO(*Rd);
                UPDATE_SREG_NEG(*Rd);
                BIT_CLR(SREG_, SREG_V);
                UPDATE_SREG_SIGN();
                return;
            }

            case 0x80: //ANDI
            {
                *Rd &= *k;

                UPDATE_SREG_ZERO(*Rd);
                UPDATE_SREG_NEG(*Rd);
                BIT_CLR(SREG_, SREG_V);
                UPDATE_SREG_SIGN();
                return;
            }

            case 0x90: //SUBI
            {
                byte old = *Rd;
                *Rd -= *k;

                UPDATE_SREG_CARRY(old, *k, *Rd);
                UPDATE_SREG_ZERO(*Rd);
                UPDATE_SREG_NEG(*Rd);
                UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(old, *k, *Rd);
                UPDATE_SREG_SIGN();
                return;
            }

            case 0xA0: //SBCI
            {
                byte old = *Rd;
                *Rd -= *k - BIT_VAL(SREG_, SREG_C);

                UPDATE_SREG_CARRY(old, *k, *Rd);
                UPDATE_SREG_ZERO_IFCHANGED(*Rd);
                UPDATE_SREG_NEG(*Rd);
                UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(old, *k, *Rd);
                UPDATE_SREG_SIGN();
                return;
            }

            case 0xB0: //CPI 
            {
                byte result = *Rd - *k;

                UPDATE_SREG_CARRY(*Rd, *k, result);
                UPDATE_SREG_ZERO(result);
                UPDATE_SREG_NEG(result);
                UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(*Rd, *k, result);
                UPDATE_SREG_SIGN();
                return;
            }

            default:
                return invalid_opcode(*instruction);
        }
    }


    // these instructions all have no operand byte
    switch (*instruction) {
        case 0x0A: // HALT
        {
            running_ = 0;
            return;
        }

        case 0x00: // NOP
        {
            return;
        }

        case 0x01: // ICALL
        {
            push_stack((PC_ & 0xFF00) >> 8);
            push_stack(PC_ & 0x00FF);
            PC_ = REG_Z;
            return;
        }

        case 0x02: // IJMP
        {
            PC_ = REG_Z;
            return;
        }

        case 0x03: // RET
        {
            byte l = pop_stack();
            byte h = pop_stack();
            PC_ = (h<<8)|l;
            return;
        }

    }

    // remaining instructions all have an operand byte
    byte *operand = fetch_instruction_byte();
    byte d = (*operand & 0xF0) >> 4;
    byte r = (*operand & 0x0F);
    byte *Rd = &REG_[d];

    switch (*instruction) {
        case 0x10: // LSL
        {
            byte old = *Rd;
            *Rd = *Rd << 1;

            BIT_UP(SREG_, SREG_C, BIT_VAL(old, 7));
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_NXORC();
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x11: // LSR
        {
            byte old = *Rd;
            *Rd = *Rd >> 1;

            BIT_UP(SREG_, SREG_C, BIT_VAL(old, 0));
            UPDATE_SREG_ZERO(*Rd);
            BIT_CLR(SREG_, SREG_N);
            UPDATE_SREG_OVERFLOW_NXORC();
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x12: // ROL
        {
            byte old = *Rd;
            *Rd = (*Rd << 1) | BIT_VAL(SREG_, SREG_C);

            BIT_UP(SREG_, SREG_C, BIT_VAL(old, 7));
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_NXORC();
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x13: // ROR
        {
            byte old = *Rd;
            *Rd = (BIT_VAL(SREG_, SREG_C) << 8) | (*Rd >> 1);

            BIT_UP(SREG_, SREG_C, BIT_VAL(old, 0));
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_NXORC();
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x14: // ASR
        {
            byte old = *Rd;
            *Rd = (old & 0x80) | (*Rd >> 1);

            BIT_UP(SREG_, SREG_C, BIT_VAL(old, 0));
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_NXORC();
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x15: // AND
        {
            *Rd &= REG_[r];

            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            BIT_CLR(SREG_, SREG_V);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x16: // EOR
        {
            *Rd ^= REG_[r];

            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            BIT_CLR(SREG_, SREG_V);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x17: // OR 
        {
            *Rd |= REG_[r];

            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            BIT_CLR(SREG_, SREG_V);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x18: // NEG
        {
            *Rd = ~(*Rd) + 1; // twos complement

            BIT_UP(SREG_, SREG_C, *Rd != 0);
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            BIT_UP(SREG_, SREG_V, *Rd == 0x80);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x19: // COM
        {
            *Rd = ~(*Rd);

            BIT_SET(SREG_, SREG_C);
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            BIT_CLR(SREG_, SREG_V);
            UPDATE_SREG_SIGN();
            return;
        }


        case 0x20: // ADD
        {
            byte old = *Rd;
            *Rd += REG_[r];

            UPDATE_SREG_CARRY(old, REG_[r], *Rd);
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(old, REG_[r], *Rd);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x21: // ADC
        {
            byte old = *Rd;
            *Rd += REG_[r] + BIT_VAL(SREG_, SREG_C);

            UPDATE_SREG_CARRY(old, REG_[r], *Rd);
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(old, REG_[r], *Rd);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x22: // SUB
        {
            byte old = *Rd;
            *Rd -= REG_[r];

            UPDATE_SREG_CARRY(old, REG_[r], *Rd);
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(old, REG_[r], *Rd);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x23: // SBC
        {
            byte old = *Rd;
            *Rd -= REG_[r] - BIT_VAL(SREG_, SREG_C);

            UPDATE_SREG_CARRY(old, REG_[r], *Rd);
            UPDATE_SREG_ZERO(*Rd);
            UPDATE_SREG_NEG(*Rd);
            UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(old, REG_[r], *Rd);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0x24: // MUL
        {
            unsigned int result = (unsigned byte)(*Rd) * (unsigned byte)(REG_[r]);
            REG_[0] = result & 0x00FF;
            REG_[1] = (result & 0xFF00) >> 8;

            BIT_UP(SREG_, SREG_C, BIT_VAL(result, 15));
            UPDATE_SREG_ZERO(result);
            return;
        }

        case 0x25: // MULS
        {
            signed int result = (signed byte)(*Rd) * (signed byte)(REG_[r]);
            REG_[0] = result & 0x00FF;
            REG_[1] = (result & 0xFF00) >> 8;

            BIT_UP(SREG_, SREG_C, BIT_VAL(result, 15));
            UPDATE_SREG_ZERO(result);
            return;
        }

        case 0x26: // MULSU
        {
            signed int result = (signed byte)(*Rd) * (unsigned byte)(REG_[r]);
            REG_[0] = result & 0x00FF;
            REG_[1] = (result & 0xFF00) >> 8;

            BIT_UP(SREG_, SREG_C, BIT_VAL(result, 15));
            UPDATE_SREG_ZERO(result);
            return;
        }



        case 0x30: // MOV
        {
            *Rd = REG_[r];

            return;
        }

        case 0x31: // LD 
        {
            byte *mem = (byte*)access_memory(REG_Z);
            if (mem == 0) {
                return segfault(REG_Z);
            }
            *Rd = *mem;

            return;
        }

        case 0x33: // ST 
        {
            byte *mem = (byte*)access_memory(REG_Z);
            if (mem == 0) {
                return segfault(REG_Z);
            }
            *mem = *Rd;
            return;
        }


        case 0x35: // PUSH
        {
            push_stack(*Rd);
            return;
        }

        case 0x36: // POP
        {
            *Rd = pop_stack();
            return;
        }



        case 0xC0: // CP
        {
            byte result = *Rd - REG_[r];

            UPDATE_SREG_CARRY(*Rd, REG_[r], result);
            UPDATE_SREG_ZERO(result);
            UPDATE_SREG_NEG(result);
            UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(*Rd, REG_[r], result);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0xC1: // CPC
        {
            byte result = *Rd - REG_[r] - BIT_VAL(SREG_, SREG_C);

            UPDATE_SREG_CARRY(*Rd, REG_[r], result);
            UPDATE_SREG_ZERO(result);
            UPDATE_SREG_NEG(result);
            UPDATE_SREG_OVERFLOW_TWOSCOMPLEMENT(*Rd, REG_[r], result);
            UPDATE_SREG_SIGN();
            return;
        }

        case 0xC2: // CPSE
        {
            if (*Rd == REG_[r]) {
                skip_next_instruction();
            }
            return;
        }

        case 0xC3: // SBRC
        {
            if (BIT_VAL(*Rd, r & 0x7) == 0) {
                skip_next_instruction();
            }
            return;
        }

        case 0xC4: // SBRS
        {
            if (BIT_VAL(*Rd, r & 0x7) == 1) {
                skip_next_instruction();
            }
            return;
        }


        case 0xD0: // RCALL
        {
            push_stack((PC_ & 0xFF00) >> 8);
            push_stack(PC_ & 0x00FF);
            PC_ += (signed byte)r;
            return;
        }

        case 0xD1: // RJUMP
        {
            PC_ += (signed byte)r;
            return;
        }

        case 0xD2: // CALL
        {
            push_stack((PC_ & 0xFF00) >> 8);
            push_stack(PC_ & 0x00FF);
            PC_ = REG_Z + (signed byte)r;
            return;
        }

        case 0xD3: // JUMP
        {
            PC_ = REG_Z + (signed byte)r;
            return;
        }

    }

    // BR** has the bit number encoded in 0x0F
    if ((*instruction & 0xF0) == 0xE0) {
        byte bitno = *instruction & 0x0F;
        byte val = BIT_VAL(SREG_, bitno % 8);
        if (bitno > 7) { // bit cleared test, so invert test
            val = !val;
        }
        if (val) {
            PC_ += (signed byte)r;
        }
    }

    // SE* and CL* have the bit number encoded in 0x0F
    if ((*instruction & 0xF0) == 0xF0) {
        byte bitno = *instruction & 0x0F;
        BIT_UP(SREG_, bitno % 8, bitno > 7 ? 0 : 1);
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
    return &REG_[(offset - 0xF0)];
  }
  
  switch (offset) {
    case 0xE0: //SPL
      return (byte*)(&SP_)+1;
    case 0xE1: //SPH
      return (byte*)(&SP_);
    //TODO: finish mapping system memory page
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

void BlinkenBoxInstance::skip_next_instruction()
{
    byte *instruction = (byte *)access_memory(PC_);
    if (instruction == 0) {
        return; // segfault
    }
    // all instructions except 0* have an operand byte

    if ((*instruction & 0xF0) == 0) {
        PC_ += 1;
    }
    PC_ += 1;
    
}


void BlinkenBoxInstance::push_stack(byte value)
{
// FIXME: err... silently segfault on stack access?
    byte *loc = (byte*)access_memory(SP_);
    if (loc != 0) {
        *loc = value;
        SP_ -= 1;
    }
}

byte BlinkenBoxInstance::pop_stack()
{
// FIXME: err... return 0 when segfault on stack access?
    byte value = 0;
    byte *loc = (byte*)access_memory(SP_);
    if (loc != 0) {
        value = *loc;
        SP_ += 1;
    }
    return value;
}



void BlinkenBoxInstance::invalid_opcode(byte instruction)
{
}
void BlinkenBoxInstance::segfault(int address)
{
}


