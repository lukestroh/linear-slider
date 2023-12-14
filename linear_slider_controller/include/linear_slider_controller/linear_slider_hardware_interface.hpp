#ifndef __LINEAR_SLIDER_CONTROLLER__HARDWARE_INTERFACE_H__
#define __LINEAR_SLIDER_CONTROLLER__HARDWARE_INTERFACE_H__

#include <hardware_interface/system_interface.hpp>

namespace hardware_interface
{
    class LinearSliderHardwareInterface : public SystemInterface
    {
        public:
            LinearSliderHardwareInterface() {}
            ~LinearSliderHardwareInterface() {}

    };
}