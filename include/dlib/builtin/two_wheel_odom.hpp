#pragma once
#include "api.h"
#include "dlib/interfaces/odom.hpp"

namespace dlib {

class TwoWheelOdom {
    private:
        double previous_forward = 0;
        double previous_theta = 0;
        Position position = Position();
        
    public:
        TwoWheelOdom() {};

        void update(double forward, double theta);
        Position get_position();
};

}