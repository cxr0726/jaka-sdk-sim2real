#pragma once
#include <chrono>
namespace rclcpp
{
    struct Duration
    {
        Duration()=default;
        template<class Rep,class Period> explicit Duration(std::chrono::duration<Rep,Period> value)
            :seconds(std::chrono::duration<double>(value).count()){}
        double seconds=0.;
    };
}
