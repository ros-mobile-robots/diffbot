#ifndef DIFFBOT_WHEEL_COMMAND_H
#define DIFFBOT_WHEEL_COMMAND_H

namespace diffbot
{
    /// The target angular velocities of both wheels, in rad/s
    struct WheelCommand
    {
        float left_rad_s;
        float right_rad_s;
    };
}

#endif // DIFFBOT_WHEEL_COMMAND_H
