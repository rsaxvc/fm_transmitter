/*
    Hardware registers for Raspberry Pi

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

#ifndef REGISTERS_HPP
#define REGISTERS_HPP

#include <stdint.h>
#include <bcm_host.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdexcept>
#include <thread>

#define PERIPHERALS_PHYS_BASE 0x7e000000
#define BCM2835_PERI_VIRT_BASE 0x20000000
#define BCM2838_PERI_VIRT_BASE 0xfe000000
#define DMA0_BASE_OFFSET 0x00007000
#define DMA15_BASE_OFFSET 0x00e05000
#define CLK0_BASE_OFFSET 0x00101070
#define CLK1_BASE_OFFSET 0x00101078
#define PWMCLK_BASE_OFFSET 0x001010a0
#define GPIO_BASE_OFFSET 0x00200000
#define PWM_BASE_OFFSET 0x0020c000
#define TIMER_BASE_OFFSET 0x00003000

#define BCM2835_MEM_FLAG 0x0c
#define BCM2838_MEM_FLAG 0x04

#define BCM2835_PLLD_FREQ 500
#define BCM2838_PLLD_FREQ 750

#define BUFFER_TIME 1000000
#define PWM_WRITES_PER_SAMPLE 10
#define PWM_CHANNEL_RANGE 32
#define PAGE_SIZE 4096

struct TimerRegisters {
    uint32_t ctlStatus;
    uint32_t low;
    uint32_t high;
    uint32_t c0;
    uint32_t c1;
    uint32_t c2;
    uint32_t c3;
};

struct ClockRegisters {
    uint32_t ctl;
    uint32_t div;
};

struct PWMRegisters {
    uint32_t ctl;
    uint32_t status;
    uint32_t dmaConf;
    uint32_t reserved0;
    uint32_t chn1Range;
    uint32_t chn1Data;
    uint32_t fifoIn;
    uint32_t reserved1;
    uint32_t chn2Range;
    uint32_t chn2Data;
};

struct DMAControllBlock {
    uint32_t transferInfo;
    uint32_t srcAddress;
    uint32_t dstAddress;
    uint32_t transferLen;
    uint32_t stride;
    uint32_t nextCbAddress;
    uint32_t reserved0;
    uint32_t reserved1;
};

struct DMARegisters {
    uint32_t ctlStatus;
    uint32_t cbAddress;
    uint32_t transferInfo;
    uint32_t srcAddress;
    uint32_t dstAddress;
    uint32_t transferLen;
    uint32_t stride;
    uint32_t nextCbAddress;
    uint32_t debug;
};

class Peripherals
{
    public:
        virtual ~Peripherals() {
            munmap(peripherals, GetSize());
        }
        Peripherals(const Peripherals &) = delete;
        Peripherals(Peripherals &&) = delete;
        Peripherals &operator=(const Peripherals &) = delete;
        static Peripherals &GetInstance() {
            static Peripherals instance;
            return instance;
        }
        inline uint32_t GetPhysicalAddress(volatile void *object) const {
            return PERIPHERALS_PHYS_BASE + (reinterpret_cast<uint32_t>(object) - reinterpret_cast<uint32_t>(peripherals));
        }
        inline uint32_t GetVirtualAddress(uint32_t offset) const {
            return reinterpret_cast<uint32_t>(peripherals) + offset;
        }
        inline static uint32_t GetVirtualBaseAddress() {
            return (bcm_host_get_peripheral_size() == BCM2838_PERI_VIRT_BASE) ? BCM2838_PERI_VIRT_BASE : bcm_host_get_peripheral_address();
        }
        inline static float GetClockFrequency() {
            return (Peripherals::GetVirtualBaseAddress() == BCM2838_PERI_VIRT_BASE) ? BCM2838_PLLD_FREQ : BCM2835_PLLD_FREQ;
        }
    private:
        Peripherals() {
            int memFd;
            if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
                throw std::runtime_error("Cannot open /dev/mem file (permission denied)");
            }

            peripherals = mmap(nullptr, GetSize(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, GetVirtualBaseAddress());
            close(memFd);
            if (peripherals == MAP_FAILED) {
                throw std::runtime_error("Cannot obtain access to peripherals (mmap error)");
            }
        }
        unsigned GetSize() {
            unsigned size = bcm_host_get_peripheral_size();
            if (size == BCM2838_PERI_VIRT_BASE) {
                size = 0x01000000;
            }
            return size;
        }

        void *peripherals;
};


class Device
{
    public:
        Device() {
            peripherals = &Peripherals::GetInstance();
        }
        Device(const Device &) = delete;
        Device(Device &&) = delete;
        Device &operator=(const Device &) = delete;
    protected:
        Peripherals *peripherals;
};

class ClockDevice : public Device
{
    public:
        ClockDevice(uint32_t clockAddress, unsigned divisor) {
            clock = reinterpret_cast<ClockRegisters *>(peripherals->GetVirtualAddress(clockAddress));
            clock->ctl = (0x5a << 24) | 0x06;
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            clock->div = (0x5a << 24) | (0xffffff & divisor);
            clock->ctl = (0x5a << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;
        }
        virtual ~ClockDevice() {
            clock->ctl = (0x5a << 24) | 0x06;
        }
    protected:
        volatile ClockRegisters *clock;
};

class ClockOutput : public ClockDevice
{
    public:
#ifndef GPIO21
        ClockOutput(unsigned divisor) : ClockDevice(CLK0_BASE_OFFSET, divisor) {
            output = reinterpret_cast<uint32_t *>(peripherals->GetVirtualAddress(GPIO_BASE_OFFSET));
            *output = (*output & 0xffff8fff) | (0x04 << 12);
#else
        ClockOutput(unsigned divisor) : ClockDevice(CLK1_BASE_OFFSET, divisor) {
            output = reinterpret_cast<uint32_t *>(peripherals->GetVirtualAddress(GPIO_BASE_OFFSET + 0x08));
            *output = (*output & 0xffffffc7) | (0x02 << 3);
#endif
        }
        virtual ~ClockOutput() {
#ifndef GPIO21
            *output = (*output & 0xffff8fff) | (0x01 << 12);
#else
            *output = (*output & 0xffffffc7) | (0x02 << 3);
#endif
        }
        inline void SetDivisor(unsigned divisor) {
            clock->div = (0x5a << 24) | (0xffffff & divisor);
        }
        inline volatile uint32_t &GetDivisor() {
            return clock->div;
        }
    private:
        volatile uint32_t *output;
};

class PWMController : public ClockDevice
{
    public:
        PWMController(unsigned sampleRate) : ClockDevice(PWMCLK_BASE_OFFSET, static_cast<unsigned>(Peripherals::GetClockFrequency() * 1000000.f * (0x01 << 12) / (PWM_WRITES_PER_SAMPLE * PWM_CHANNEL_RANGE * sampleRate))) {
            pwm = reinterpret_cast<PWMRegisters *>(peripherals->GetVirtualAddress(PWM_BASE_OFFSET));
            pwm->ctl = 0x00000000;
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            pwm->status = 0x01fc;
            pwm->ctl = (0x01 << 6);
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            pwm->chn1Range = PWM_CHANNEL_RANGE;
            pwm->dmaConf = (0x01 << 31) | 0x0707;
            pwm->ctl = (0x01 << 5) | (0x01 << 2) | 0x01;
        }
        virtual ~PWMController() {
            pwm->ctl = 0x00000000;
        }
        inline volatile uint32_t &GetFifoIn() {
            return pwm->fifoIn;
        }
    private:
        volatile PWMRegisters *pwm;
};

class DMAController : public Device
{
    public:
        DMAController(uint32_t controllBlockAddress, unsigned dmaChannel) {
            dma = reinterpret_cast<DMARegisters *>(peripherals->GetVirtualAddress((dmaChannel < 15) ? DMA0_BASE_OFFSET + dmaChannel * 0x100 : DMA15_BASE_OFFSET));
            dma->ctlStatus = (0x01 << 31); //Reset the channel
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            dma->ctlStatus = (0x01 << 2) | (0x01 << 1); //Write 1s to clear INT and END flags
            dma->cbAddress = controllBlockAddress;
            dma->ctlStatus = (0xff << 16) | 0x01;//Configure priority to highest
        }
        virtual ~DMAController() {
            dma->ctlStatus = (0x01 << 31);//Reset the controller
        }
        inline void SetControllBlockAddress(uint32_t address) {
            dma->cbAddress = address;
        }
        inline volatile uint32_t &GetControllBlockAddress() {
            return dma->cbAddress;
        }
    private:
        volatile DMARegisters *dma;
};

#endif
