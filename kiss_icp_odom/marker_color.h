//
// Created by jimmy on 23-8-17.
//

#pragma once
#include <std_msgs/msg/color_rgba.hpp>


inline std_msgs::msg::ColorRGBA get_loop_closure_color(){
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 0.5;
    return color;
}

inline std_msgs::msg::ColorRGBA get_key_color(){
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 0.5;
    return color;
}


inline std_msgs::msg::ColorRGBA get_gps_color(){
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 0.5;
    return color;
}


inline std_msgs::msg::ColorRGBA get_loop_residual_color(){
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 0.5;
    return color;
}
