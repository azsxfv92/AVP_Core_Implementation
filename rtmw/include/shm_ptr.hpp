#pragma once

#include <cstddef>
#include <memory>
#include <sys/mman.h>

namespace rtmw{
    
    struct ShmDeleter {
        std::size_t size;
        void operator()(void* p) const {
            if(p != nullptr){ 
                munmap(p, size);
            }
        }
    };

    using ShmPtr = std::unique_ptr<void, ShmDeleter>;
}