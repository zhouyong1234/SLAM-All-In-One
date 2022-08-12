/* 
 * scout_params.hpp
 * 
 * Created on: Sep 27, 2019 15:08
 * Description: 
 * 
 * Copyright (c) 2020 Ruixiang Du (rdu)
 */

#ifndef SCOUT_PARAMS_HPP
#define SCOUT_PARAMS_HPP

namespace westonrobot
{
    struct ScoutParams
    {
        /* Scout Parameters */
        static constexpr double max_steer_angle = 30.0; // in degree

        static constexpr double track = 0.58306;      // in meter (left & right wheel distance)
        static constexpr double wheelbase = 0.498;    // in meter (front & rear wheel distance)
        static constexpr double wheel_radius = 0.165; // in meter

        // from user manual v1.2.8 P18
        // max linear velocity: 1.5 m/s
        // max angular velocity: 0.7853 rad/s
        static constexpr double max_linear_speed = 1.5;     // in m/s
        static constexpr double max_angular_speed = 0.7853; // in rad/s
        static constexpr double max_speed_cmd = 10.0;       // in rad/s
    };
} // namespace westonrobot

#endif /* SCOUT_PARAMS_HPP */
