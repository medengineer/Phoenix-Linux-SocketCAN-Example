/*
 * Copyright (c) 2018-2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdint.h>
#include <unordered_map>
#include <map>
#include <vector>
#include <memory>
#include <set>
#include <thread>
#include <atomic>
#include <utility>

#include "rev/CANSparkMaxFrames.h"

namespace rev {
namespace detail {

struct CANScanIdentifier {
    frc_deviceType_t deviceTypeId;
    frc_manufacturer_t manufacturerId;
    uint16_t canId;
    uint32_t uniqueId;
    std::string name;
    CANScanIdentifier(uint32_t arbid, std::string name, uint32_t uniqueId = 0) {
        frc_frameID_t frame;
        frame.raw = arbid;
        this->name = name;
        this->uniqueId = uniqueId;
        this->deviceTypeId = frame.fields.deviceType;
        this->manufacturerId = frame.fields.manufacturer;
        this->canId = frame.fields.deviceNumber;
    }

    std::string Name() {
        if (name.empty()) {
            return std::string(GetFRCManufacturerText(manufacturerId)) + " " + std::string(GetFRCDeviceTypeText(deviceTypeId));
        } else {
            return name;
        }
    }
};
  

inline bool operator<(const CANScanIdentifier& lhs, const CANScanIdentifier& rhs) {
        return std::tie(lhs.deviceTypeId, lhs.manufacturerId, lhs.canId, lhs.uniqueId) < 
        std::tie(rhs.deviceTypeId, rhs.manufacturerId, rhs.canId, rhs.uniqueId);
}

inline bool operator>(const CANScanIdentifier& lhs, const CANScanIdentifier& rhs) {
        return std::tie(lhs.deviceTypeId, lhs.manufacturerId, lhs.canId, lhs.uniqueId) >
                std::tie(rhs.deviceTypeId, rhs.manufacturerId, rhs.canId, rhs.uniqueId);
}

inline bool operator==(const CANScanIdentifier& lhs, const CANScanIdentifier& rhs) {
        return std::tie(lhs.deviceTypeId, lhs.manufacturerId, lhs.canId, lhs.uniqueId) ==
                std::tie(rhs.deviceTypeId, rhs.manufacturerId, rhs.canId, rhs.uniqueId);
}

class CANBusScanner {

public:
    // Make sure the buffer size is large enough to capture all needed messages in
    // the thread interval time
    CANBusScanner(int buffersize = 256, int threadIntervalMs = 10);
    ~CANBusScanner();

    bool Start();
    void Stop();
    bool Running();
    std::string LastError();
    std::vector<CANScanIdentifier> CANBusScan();
    void RegisterDevice(std::string name, std::vector<uint32_t> validIds, int32_t maxFramePeriodMs = 100);

private:
    class CANScanElement {
        uint64_t lastSeen;
        uint64_t timeout;
    public:
        CANScanElement(uint64_t timeoutMs = 1000);
        void UpdateLastSeen();
        bool IsActive() const;
    };

    class CANScanCollection {
    public:
        CANScanCollection(std::string name, uint32_t arbId, uint64_t timeoutMs)
                : name(name), 
                  timeout(timeoutMs),
                  arbId(arbId) { }

        std::vector<int> ActiveDevices() const {
            std::vector<int> result;
            for (auto const& dev : devices) {
                if (dev.second.IsActive()) {
                    result.push_back(dev.first);
                }
            }
            return result;
        }

        void AddOrUpdateDevice(int id) {
            if (devices.find(id) != devices.end()) {
                devices[id] = CANScanElement(timeout);
            }
            devices[id].UpdateLastSeen();
        }

        std::string Name() const { return name; }
        uint32_t ArbId() const { return arbId; }
    private:
        std::string name;
        uint64_t timeout;
        uint32_t arbId;
        std::map<int, CANScanElement> devices;
    };

    // Data structure is a map of all arbIds that point to a single shared collection
    // and a vector of all collections (same pointer). This allows a fast lookup per
    // arbID recieved and a way to iterate through by registered device
    std::map< uint32_t, std::shared_ptr<CANScanCollection> > m_registeredDevices;
    std::vector< std::shared_ptr<CANScanCollection> > m_registeredList;

    int m_streamBufferSize;
    uint32_t m_streamHandle;
    int m_threadInterval;

    std::thread m_thread;
    std::atomic_bool m_stopThread;
    std::atomic_bool m_running;
    std::string m_lastError;

    void run();
};

} // namespace detail
} // namespace rev
