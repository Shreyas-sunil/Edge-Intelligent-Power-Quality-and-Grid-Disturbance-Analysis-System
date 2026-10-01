// Non-volatile configuration storage interface.
//   Pico : last 4 KB flash sector (platform/pico/flash_store.cpp)
//   Host : file (platform/host/host_io.cpp)
#pragma once
#include "system/config.h"

namespace pq {

class ConfigStore {
public:
    virtual ~ConfigStore() = default;
    virtual bool load(StoredConfig& out) = 0;       // false if empty / unreadable
    virtual bool save(const StoredConfig& in) = 0;
    virtual bool erase() = 0;
};

}  // namespace pq
