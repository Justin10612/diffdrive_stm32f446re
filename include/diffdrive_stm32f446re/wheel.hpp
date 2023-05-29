#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

#include <string>
#include <cmath>

class Wheel
{
    public:

    std::string name = "";  // wheel name
    int enc = 0;    // encoder counts
    double cmd = 0; // command velocity
    double pos = 0; // feedback position
    double vel = 0; // feedback velocity
    // double eff = 0;
    // double velSetPt = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
        setup(wheel_name, counts_per_rev);
    }
    
    void setup(const std::string &wheel_name, int counts_per_rev){
        name = wheel_name;
        rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calcEncAngle(){
        return enc*rads_per_count;
    }


};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H