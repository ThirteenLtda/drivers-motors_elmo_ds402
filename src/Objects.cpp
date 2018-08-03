#include <motors_elmo_ds402/Objects.hpp>

using namespace motors_elmo_ds402;

StatusWord::State parseState(uint8_t byte)
{
    switch(byte & 0x4F)
    {
        case 0x00: return StatusWord::NOT_READY_TO_SWITCH_ON;
        case 0x40: return StatusWord::SWITCH_ON_DISABLED;
        case 0x0F: return StatusWord::FAULT_REACTION_ACTIVE;
        case 0x08: return StatusWord::FAULT;
    }

    switch(byte & 0x6F)
    {
        case 0x21: return StatusWord::READY_TO_SWITCH_ON;
        case 0x23: return StatusWord::SWITCH_ON;
        case 0x27: return StatusWord::OPERATION_ENABLED;
        case 0x07: return StatusWord::QUICK_STOP_ACTIVE;
    }

    throw StatusWord::UnknownState("received an unknown value for the state");
}

namespace motors_elmo_ds402
{
    template<>
    uint16_t encode<ControlWord, uint16_t>(ControlWord const& value)
    {
        uint16_t word = 0;
        switch(value.transition)
        {
            case ControlWord::SHUTDOWN:
                word = 0x06;
                break;
            case ControlWord::SWITCH_ON:
                word = 0x07;
                break;
            case ControlWord::ENABLE_OPERATION:
                word = 0x0F;
                break;
            case ControlWord::DISABLE_VOLTAGE:
                word = 0x00;
                break;
            case ControlWord::QUICK_STOP:
                word = 0x02;
                break;
            case ControlWord::DISABLE_OPERATION:
                word = 0x07;
                break;
            case ControlWord::FAULT_RESET:
                word = 0x80;
                break;
        }

        if (value.enable_halt)
            word |= 0x100;

        return word;
    }

    template<>
    StatusWord parse<StatusWord, uint16_t>(uint16_t word)
    {
        StatusWord::State state = parseState(word & 0x6F);
        bool voltageEnabled = (word & 0x0010);
        bool warning        = (word & 0x0080);
        bool targetReached  = (word & 0x0400);
        bool internalLimitActive = (word & 0x0800);
        return StatusWord { state, voltageEnabled, warning,
            targetReached, internalLimitActive };
    }
}