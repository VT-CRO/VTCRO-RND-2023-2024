/////////////////////////////////////////////////////////////
// Author: Jayson De La Vega   R&D Team VT CRO
// filename: SmoothServo.cpp
// Last Modified: 3/24/2024
// Description:  This file contains class definitions for
//               a servo class with smoother movement
/////////////////////////////////////////////////////////////
#include "SmoothServo.h"
#include "arduino_freertos.h"

SmoothServo::SmoothServo(int pin)
{
    servo.attach(pin);

    yd = 0;
    y = 0;
    xp = 0;
}

SmoothServo::~SmoothServo()
{
    servo.detach();
}

void SmoothServo::setConstants(double damping, double resonant, double r)
{
    k1 = damping / (PI * resonant);
    k2 = 1 / ((2 * PI * resonant) * (2 * PI * resonant));
    k3 = r * damping / (2 * PI * resonant);
}

void SmoothServo::updatePosition(unsigned long &lastUpdate)
{
    unsigned long dt = millis() - lastUpdate;
    if (dt >= 100) {
        lastUpdate = millis();

        double xd = (goalAngle - xp) / dt;
        xp = goalAngle;

        y = y + dt * yd;
        yd = yd + dt * (goalAngle + k3 * xd - y - k1 * yd) / k2;

        servo.write(y);
    }
}

void SmoothServo::setGoalPosition(int angle)
{
    goalAngle = angle;
    if (goalAngle > 180)
        goalAngle = 180;
    if (goalAngle < 0)
        goalAngle = 0;
}
