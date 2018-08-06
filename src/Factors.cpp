#include <motors_elmo_ds402/Factors.hpp>

using namespace std;
using namespace motors_elmo_ds402;

double Factors::rawToEncoder(int64_t encoder) const
{
    return static_cast<double>(encoder) *
        positionNumerator / positionDenominator;
}

int64_t Factors::rawFromEncoder(double encoder) const
{
    return static_cast<double>(encoder) *
        positionDenominator / positionNumerator;
}

long Factors::rawFromCurrent(double current) const
{
    return static_cast<double>(current) / ratedCurrent * 1000;
}

long Factors::rawFromTorque(double torque) const
{
    return static_cast<double>(torque) / ratedTorque * 1000;
}

double Factors::rawToCurrent(long raw) const
{
    return static_cast<double>(raw) / 1000 * ratedCurrent;
}

double Factors::rawToTorque(long raw) const
{
    return static_cast<double>(raw) / 1000 * ratedTorque;
}

void Factors::update()
{
    positionNumerator =
        encoderScaleFactor *
        feedLength *
        encoderRevolutions *
        gearDrivingShaftRevolutions;
    positionDenominator =
        feedDrivingShaftRevolutions *
        encoderTicks *
        gearMotorShaftRevolutions;
}
