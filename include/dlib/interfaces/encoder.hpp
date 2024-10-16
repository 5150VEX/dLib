#pragma once
#include <cstdint>

namespace dlib {
    class Encoder {
        public:
            virtual void reset() = 0;
            virtual double get_displacement() = 0;
            virtual const uint32_t get_update_rate() const = 0;
    };
}