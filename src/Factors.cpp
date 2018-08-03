#include <motors_elmo_ds402/Factors.hpp>

using namespace std;
using namespace motors_elmo_ds402;

double Factors::scaleEncoderValue(int64_t encoder) const
{
    double in_turns = static_cast<double>(encoder) *
        positionNumerator / positionDenominator;
    return in_turns;
}

double Factors::currentToUserTorque(long current) const
{
    return static_cast<double>(current) / 1000 * ratedTorque;
}

double Factors::currentToUser(long current) const
{
    return static_cast<double>(current) / 1000 * ratedCurrent;
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
