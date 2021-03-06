/*
    Timer Debugger - fetch registers from Raspberry Pi Timer

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
    printf(" ctlStatus,       low,      high,        c0,        c1,        c2,        c3\n");
}

static void print_regs( const TimerRegisters & regs )
{
    printf("0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x\n",
        regs.ctlStatus,
        regs.low,
        regs.high,
        regs.c0,
        regs.c1,
        regs.c2,
        regs.c3);
}

static TimerRegisters readRegs( volatile TimerRegisters * timer )
{
    TimerRegisters regs;
    regs.ctlStatus = timer->ctlStatus;
    regs.low = timer->low;
    regs.high = timer->high;
    regs.c0 = timer->c0;
    regs.c1 = timer->c1;
    regs.c2 = timer->c2;
    regs.c3 = timer->c3;
    return regs;
}

int main( void )
{
    print_hdr();
    Peripherals & peripherals = Peripherals::GetInstance();

    volatile TimerRegisters *timer = reinterpret_cast<TimerRegisters *>(peripherals.GetVirtualAddress(TIMER_BASE_OFFSET));

    TimerRegisters crntRegs = readRegs( timer);
    crntRegs.ctlStatus -= 1;

    while(1) {
        TimerRegisters newRegs = readRegs( timer );
        if( !memcmp( &newRegs, &crntRegs, sizeof(TimerRegisters) ) ) {
            sched_yield();
        } else {
            print_regs(newRegs);
            crntRegs = newRegs;
        }
    }

return 0;
}

