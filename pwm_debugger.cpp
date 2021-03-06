/*
    PWM debugger - fetch registers from Raspberry Pi PWM

    Copyright (c) 2020, Marcin Kondej
    All rights reserved.

    See https://github.com/markondej/fm_transmitter

    Redistribution and use in source and binary forms, with or without modification, are
    permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be
    used to endorse or promote products derived from this software without specific
    prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
    TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "transmitter.hpp"
#include "mailbox.h"
#include "peripherals.hpp"
#include <bcm_host.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>
#include <sched.h>

static void print_hdr( void )
{
    printf("       ctl,    status,   dmaConf,     rsrv0, chn1Range,  chn1Data,    fifoIn,     rsrv1,   chn2Rng,  chn2Data\n");
}

static void print_regs( const PWMRegisters & regs )
{
    printf("0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x\n",
        regs.ctl,
        regs.status,
        regs.dmaConf,
        regs.reserved0,
        regs.chn1Range,
        regs.chn1Data,
        regs.fifoIn,
        regs.reserved1,
        regs.chn2Range,
        regs.chn2Data);
}

static PWMRegisters readRegs( volatile PWMRegisters * pwm )
{
    PWMRegisters regs;
    regs.ctl = pwm->ctl;
    regs.status = pwm->status;
    regs.dmaConf = pwm->dmaConf;
    regs.reserved0 = pwm->reserved0;
    regs.chn1Range = pwm->chn1Range;
    regs.chn1Data = pwm->chn1Data;
    regs.fifoIn = pwm->fifoIn;
    regs.reserved1 = pwm->reserved1;
    regs.chn2Range = pwm->chn2Range;
    regs.chn2Data = pwm->chn2Data;
    return regs;
}

int main( void )
{
    print_hdr();
    Peripherals & peripherals = Peripherals::GetInstance();

    volatile PWMRegisters *pwm = reinterpret_cast<PWMRegisters *>(peripherals.GetVirtualAddress(PWM_BASE_OFFSET));

    PWMRegisters crntRegs = readRegs( pwm );
    crntRegs.reserved1++;

    while(1) {
        PWMRegisters newRegs = readRegs( pwm );
        if( !memcmp( &newRegs, &crntRegs, sizeof(PWMRegisters) ) ) {
            sched_yield();
        } else {
            print_regs(newRegs);
            crntRegs = newRegs;
        }
    }

return 0;
}


