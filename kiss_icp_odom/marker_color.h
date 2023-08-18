//
// Created by jimmy on 23-8-17.
//

#pragma once
#include <std_msgs/msg/color_rgba.hpp>

inline std_msgs::msg::ColorRGBA get_color_from_hex(uint32_t hex){
    std_msgs::msg::ColorRGBA color;
    color.r = ((hex >> 16) & 0xFF) / 255.0;
    color.g = ((hex >> 8) & 0xFF) / 255.0;
    color.b = ((hex >> 0) & 0xFF) / 255.0;
    color.a = 1.0;
    return color;
}

inline std_msgs::msg::ColorRGBA get_loop_closure_color(){
    return get_color_from_hex(0xBA55D3);
}

inline std_msgs::msg::ColorRGBA get_key_color(){
    return get_color_from_hex(0x00FF00);
}


inline std_msgs::msg::ColorRGBA get_gps_color(){
    return get_color_from_hex(0x00FFFF);
}


inline std_msgs::msg::ColorRGBA get_loop_residual_color(){
    return get_color_from_hex(0xFFD700);
}
