#pragma once

namespace dlib {
    struct Position {
        double x = 0;
        double y = 0;
        double heading = 0;
    };

    class Odom {
        public:
            virtual void update(double forward, double theta) = 0;
            virtual const Position get_position() = 0;
    };
}