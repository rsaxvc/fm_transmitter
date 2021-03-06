/*
    FM Transmitter - use Raspberry Pi as FM transmitter

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

class AllocatedMemory
{
    public:
        AllocatedMemory(unsigned size) {
            mBoxFd = mbox_open();
            memSize = size;
            if (memSize % PAGE_SIZE) {
                memSize = (memSize / PAGE_SIZE + 1) * PAGE_SIZE;
            }
            memHandle = mem_alloc(mBoxFd, size, PAGE_SIZE, (Peripherals::GetVirtualBaseAddress() == BCM2835_PERI_VIRT_BASE) ? BCM2835_MEM_FLAG : BCM2838_MEM_FLAG);
            if (!memHandle) {
                mbox_close(mBoxFd);
                memSize = 0;
                throw std::runtime_error("Cannot allocate memory (" + std::to_string(size) + "bytes");
            }
            memAddress = mem_lock(mBoxFd, memHandle);
            memAllocated = mapmem(memAddress & ~0xc0000000, memSize);
        }
        virtual ~AllocatedMemory() {
            unmapmem(memAllocated, memSize);
            mem_unlock(mBoxFd, memHandle);
            mem_free(mBoxFd, memHandle);
            mbox_close(mBoxFd);
            memSize = 0;
        }
        AllocatedMemory(const AllocatedMemory &) = delete;
        AllocatedMemory(AllocatedMemory &&) = delete;
        AllocatedMemory &operator=(const AllocatedMemory &) = delete;
        inline uint32_t GetPhysicalAddress(volatile void *object) const {
            return (memSize) ? memAddress + (reinterpret_cast<uint32_t>(object) - reinterpret_cast<uint32_t>(memAllocated)) : 0x00000000;
        }
        inline uint32_t GetAddress() const {
            return reinterpret_cast<uint32_t>(memAllocated);
        }
    private:
        unsigned memSize, memHandle;
        uint32_t memAddress;
        void *memAllocated;
        int mBoxFd;
};

bool Transmitter::transmitting = false;

Transmitter::Transmitter()
    : output(nullptr), stopped(true)
{
}

Transmitter::~Transmitter() {
    if (output != nullptr) {
        delete output;
    }
}

void Transmitter::Transmit(WaveReader &reader, float frequency, float bandwidth, unsigned dmaChannel, bool preserveCarrier)
{
    if (transmitting) {
        throw std::runtime_error("Cannot transmit, transmitter already in use");
    }
    transmitting = true;
    stopped = false;

    WaveHeader header = reader.GetHeader();
    unsigned bufferSize = static_cast<unsigned>(static_cast<unsigned long long>(header.sampleRate) * BUFFER_TIME / 1000000);

    unsigned clockDivisor = static_cast<unsigned>(round(Peripherals::GetClockFrequency() * (0x01 << 12) / frequency));
    unsigned divisorRange = clockDivisor - static_cast<unsigned>(round(Peripherals::GetClockFrequency() * (0x01 << 12) / (frequency + 0.0005f * bandwidth)));

    if (output == nullptr) {
        output = new ClockOutput(clockDivisor);
    }

    auto finally = [&]() {
        if (!preserveCarrier) {
            delete output;
            output = nullptr;
        }
        transmitting = false;
    };
    try {
        if (dmaChannel != 0xff) {
            TransmitViaDma(reader, *output, header.sampleRate, bufferSize, clockDivisor, divisorRange, dmaChannel);
        } else {
            TransmitViaCpu(reader, *output, header.sampleRate, bufferSize, clockDivisor, divisorRange);
        }
    } catch (...) {
        finally();
        throw;
    }
    finally();
}

void Transmitter::Stop()
{
    stopped = true;
}

void Transmitter::TransmitViaCpu(WaveReader &reader, ClockOutput &output, unsigned sampleRate, unsigned bufferSize, unsigned clockDivisor, unsigned divisorRange)
{
    std::vector<Sample> samples = reader.GetSamples(bufferSize, stopped);
    if (samples.empty()) {
        return;
    }

    unsigned sampleOffset = 0;
    bool eof = samples.size() < bufferSize;
    std::thread transmitterThread(Transmitter::TransmitterThread, this, &output, sampleRate, clockDivisor, divisorRange, &sampleOffset, &samples);

    std::this_thread::sleep_for(std::chrono::microseconds(BUFFER_TIME / 2));

    auto finally = [&]() {
		stopped = true;
        transmitterThread.join();
        samples.clear();
    };
    try {
        while (!eof && !stopped) {
            {
                std::lock_guard<std::mutex> lock(access);
                if (samples.empty()) {
                    if (!reader.SetSampleOffset(sampleOffset + bufferSize)) {
                        break;
                    }
                    samples = reader.GetSamples(bufferSize, stopped);
                    if (samples.empty()) {
                        break;
                    }
                    eof = samples.size() < bufferSize;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(BUFFER_TIME / 2));
        }
    } catch (...) {
        finally();
        throw;
    }
    finally();
}

void Transmitter::TransmitViaDma(WaveReader &reader, ClockOutput &output, unsigned sampleRate, unsigned bufferSize, unsigned clockDivisor, unsigned divisorRange, unsigned dmaChannel)
{
    if (dmaChannel > 15) {
        throw std::runtime_error("DMA channel number out of range (0 - 15)");
    }

    AllocatedMemory allocated(sizeof(uint32_t) * (bufferSize) + sizeof(DMAControllBlock) * (2 * bufferSize) + sizeof(uint32_t));

    std::vector<Sample> samples = reader.GetSamples(bufferSize, stopped);
    if (samples.empty()) {
        return;
    }

    bool eof = false;
    if (samples.size() < bufferSize) {
        bufferSize = samples.size();
        eof = true;
    }

    PWMController pwm(sampleRate);
    Peripherals &peripherals = Peripherals::GetInstance();

    unsigned i, cbOffset = 0;

    volatile DMAControllBlock *dmaCb = reinterpret_cast<DMAControllBlock *>(allocated.GetAddress());
    volatile uint32_t *clkDiv = reinterpret_cast<uint32_t *>(reinterpret_cast<uint32_t>(dmaCb) + 2 * sizeof(DMAControllBlock) * bufferSize);
    volatile uint32_t *pwmFifoData = reinterpret_cast<uint32_t *>(reinterpret_cast<uint32_t>(clkDiv) + sizeof(uint32_t) * bufferSize);
    for (i = 0; i < bufferSize; i++) {
        float value = samples[i].GetMonoValue();
        clkDiv[i] = (0x5a << 24) | (0xffffff & (clockDivisor - static_cast<int>(round(value * divisorRange))));
        dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x01 << 3);
        dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&clkDiv[i]);
        dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&output.GetDivisor());
        dmaCb[cbOffset].transferLen = sizeof(uint32_t);
        dmaCb[cbOffset].stride = 0;
        dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
        cbOffset++;

        dmaCb[cbOffset].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
        dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(pwmFifoData);
        dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&pwm.GetFifoIn());
        dmaCb[cbOffset].transferLen = sizeof(uint32_t) * PWM_WRITES_PER_SAMPLE;
        dmaCb[cbOffset].stride = 0;
        dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress((i < bufferSize - 1) ? &dmaCb[cbOffset + 1] : dmaCb);
        cbOffset++;
    }
    *pwmFifoData = 0x00000000;

    DMAController dma(allocated.GetPhysicalAddress(dmaCb), dmaChannel);

    std::this_thread::sleep_for(std::chrono::microseconds(BUFFER_TIME / 4));

    auto finally = [&]() {
        dmaCb[(cbOffset < 2 * bufferSize) ? cbOffset : 0].nextCbAddress = 0x00000000;
        while (dma.GetControllBlockAddress() != 0x00000000) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
        stopped = true;
        samples.clear();
    };
    try {
        while (!eof && !stopped) {
            samples = reader.GetSamples(bufferSize, stopped);
            if (!samples.size()) {
                break;
            }
            cbOffset = 0;
            eof = samples.size() < bufferSize;
            for (i = 0; i < samples.size(); i++) {
                float value = samples[i].GetMonoValue();
                while (i == ((dma.GetControllBlockAddress() - allocated.GetPhysicalAddress(dmaCb)) / (2 * sizeof(DMAControllBlock)))) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1000));
                }
                clkDiv[i] = (0x5a << 24) | (0xffffff & (clockDivisor - static_cast<int>(round(value * divisorRange))));
                cbOffset += 2;
            }
        }
    } catch (...) {
        finally();
        throw;
    }
    finally();
}

void Transmitter::TransmitterThread(Transmitter *instance, ClockOutput *output, unsigned sampleRate, unsigned clockDivisor, unsigned divisorRange, unsigned *sampleOffset, std::vector<Sample> *samples)
{
    Peripherals &peripherals = Peripherals::GetInstance();

    volatile TimerRegisters *timer = reinterpret_cast<TimerRegisters *>(peripherals.GetVirtualAddress(TIMER_BASE_OFFSET));
    uint64_t current = *(reinterpret_cast<volatile uint64_t *>(&timer->low));
    uint64_t playbackStart = current;

    while (true) {
        std::vector<Sample> loadedSamples;
        while (true) {
            {
                std::lock_guard<std::mutex> lock(instance->access);
                if (instance->stopped) {
                    return;
                }
                loadedSamples = std::move(*samples);
                current = *(reinterpret_cast<volatile uint64_t *>(&timer->low));
                if (!loadedSamples.empty()) {
                    *sampleOffset = (current - playbackStart) * sampleRate / 1000000;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        };

        uint64_t start = current;
        unsigned offset = (current - start) * sampleRate / 1000000;

        while (true) {
            if (offset >= loadedSamples.size()) {
                break;
            }
            unsigned prevOffset = offset;
            float value = loadedSamples[offset].GetMonoValue();
            instance->output->SetDivisor(clockDivisor - static_cast<int>(round(value * divisorRange)));
            while (offset == prevOffset) {
                std::this_thread::sleep_for(std::chrono::microseconds(1)); // asm("nop");
                current = *(reinterpret_cast<volatile uint64_t *>(&timer->low));;
                offset = (current - start) * sampleRate / 1000000;
            }
        }
    }
}
