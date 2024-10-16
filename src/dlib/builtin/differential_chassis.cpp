#include "dlib/builtin/differential_chassis.hpp"
#include "dlib/dlib.hpp"

namespace dlib {
    Motors& DifferentialChassis::get_motors() {
        return this->motors;
    }

    const Specs DifferentialChassis::get_specs() const {
        return this->specs;
    }

    void DifferentialChassis::move(int32_t voltage) {
        dlib::move(*this, voltage);
    }

    void DifferentialChassis::turn(int32_t voltage) {
        dlib::turn(*this, voltage);
    }

    void DifferentialChassis::arcade(int32_t power, int32_t turn) {
        dlib::arcade(*this, power, turn);
    }
}