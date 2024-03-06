/*
 *  Camera.cpp
 *
 *  Created on: Jan 1, 2024
 *
 *  Author: onlydcx, G4T1PR0
 */

#ifndef _APP_MODULE_CAMERA_HPP_
#define _APP_MODULE_CAMERA_HPP_

#include <McuAbstractionLayer/baseMcuAbstractionLayer.hpp>

#define YELLOW 0
#define BLUE 1

class camera {
   public:
    camera(MAL* mcu);
    void init();
    void update();
    bool AttackColor;

    struct camera_object {
        uint16_t ball_angle;
        uint16_t yellow_angle;
        uint16_t blue_angle;
        uint16_t ball_distance;
        uint16_t yellow_distance;
        uint16_t blue_distance;
        uint8_t enable;
    };

    struct camera_parsed_object : public camera_object {
        camera_parsed_object& operator=(const camera_object& other) {
            this->ball_angle = other.ball_angle;
            this->yellow_angle = other.yellow_angle;
            this->blue_angle = other.blue_angle;
            this->ball_distance = other.ball_distance;
            this->yellow_distance = other.yellow_distance;
            this->blue_distance = other.blue_distance;
            this->enable = other.enable;
            this->isBallDetected = this->enable & 0x01;
            this->isYellowDetected = this->enable >> 1 & 0x01;
            this->isBlueDetected = this->enable >> 2 & 0x01;
            return *this;
        }
        bool isBallDetected;
        bool isYellowDetected;
        bool isBlueDetected;
    };

    camera_parsed_object data;

   private:
    MAL* _mcu;
    void _read_by_header();
    void _read_via_buffer();

    int _rx_mode = 0;
    unsigned int _rx_data_index;

    union {
        uint8_t rx_buffer[sizeof(camera_object)];
        camera_object parsed_data;
    } camera_rx_data_parser;
};

#endif /* _APP_MODULE_CAMERA_HPP_ */