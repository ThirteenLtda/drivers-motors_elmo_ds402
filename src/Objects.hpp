#ifndef MOTORS_ELMO_DS402_OBJECTS
#define MOTORS_ELMO_DS402_OBJECTS

#include <cstdint>
#include <stdexcept>

namespace motors_elmo_ds402
{
    enum UPDATES
    {
        UPDATE_HEARTBEAT      = 0x00000001,
        UPDATE_PDO_CONFIG     = 0x00000002,
        UPDATE_STATUS_WORD    = 0x00000004,
        UPDATE_FACTORS        = 0x00000008,
        UPDATE_JOINT_POSITION = 0x00000010,
        UPDATE_JOINT_VELOCITY = 0x00000020,
        UPDATE_JOINT_CURRENT  = 0x00000040,
        UPDATE_JOINT_STATE    = UPDATE_JOINT_POSITION |
            UPDATE_JOINT_VELOCITY |
            UPDATE_JOINT_CURRENT,
        UPDATE_JOINT_LIMITS   = 0x00000080,
        UPDATE_OPERATION_MODE = 0x00000100
    };

    enum OPERATION_MODES
    {
        OPERATION_MODE_NONE = 0,
        OPERATION_MODE_PROFILED_POSITION = 1,
        OPERATION_MODE_VELOCITY = 2,
        OPERATION_MODE_PROFILED_VELOCITY = 3,
        OPERATION_MODE_PROFILED_TORQUE = 4,
        OPERATION_MODE_HOMING = 6,
        OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION = 8,
        OPERATION_MODE_CYCLIC_SYNCHRONOUS_VELOCITY = 9,
        OPERATION_MODE_CYCLIC_SYNCHRONOUS_TORQUE = 10
    };

    template<typename T, typename Raw> T parse(Raw value);
    template<typename T, typename Raw> Raw encode(T const& value);

    #define CANOPEN_DEFINE_OBJECT_COMMON(object_id, object_sub_id, type) \
        static const int OBJECT_ID = object_id; \
        static const int OBJECT_SUB_ID = object_sub_id; \
        typedef type OBJECT_TYPE;

    #define CANOPEN_DEFINE_RO_OBJECT(object_id, object_sub_id, name, type, update_id) \
        struct name {\
            static const int OBJECT_ID = object_id; \
            static const int OBJECT_SUB_ID = object_sub_id; \
            typedef type OBJECT_TYPE; \
            static const uint64_t UPDATE_ID = update_id; \
        }; \
        template<> name parse<name, type>(type value);
    #define CANOPEN_DEFINE_WO_OBJECT(object_id, object_sub_id, name, type) \
        struct name {\
            static const int OBJECT_ID = object_id; \
            static const int OBJECT_SUB_ID = object_sub_id; \
            typedef type OBJECT_TYPE; \
        }; \
        template<> type encode(name const& value);
    #define CANOPEN_DEFINE_RW_OBJECT(object_id, object_sub_id, name, type, update_id) \
        struct name {\
            static const int OBJECT_ID = object_id; \
            static const int OBJECT_SUB_ID = object_sub_id; \
            typedef type OBJECT_TYPE; \
            static const uint64_t UPDATE_ID = update_id; \
        }; \
        template<> name parse<name, type>(type value); \
        template<> type encode(name const& value);

    CANOPEN_DEFINE_RO_OBJECT(0x1000, 0, DeviceType,                    std::uint32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x1001, 0, ErrorRegister,                 std::uint8_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x1002, 0, ManufacturerStatusRegister,    std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x1016, 2, ConsumerHeartbeatTime,         std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x1017, 0, ProducerHeartbeatTime,         std::uint32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x1018, 4, IdentityObject,                std::uint32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x2041, 0, TimestampUsec,                 std::uint32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x2081, 5, ExtendedErrorCode,             std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x2082, 0, CANControllerStatus,           std::uint32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x2085, 0, ExtraStatusRegister,           std::int16_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x2086, 0, STOStatusRegister,             std::uint32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x2087, 0, PALVersion,                    std::uint16_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x2206, 0, DCSupply5V,                    std::uint16_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x22A3, 3, Temperature,                   std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x2E06, 0, TorqueWindow,                  std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x2E07, 0, TorqueWindowTime,              std::uint16_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x603f, 0, ErrorCode,                     std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6040, 0, ControlWordRegister,           std::uint16_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x6041, 0, StatusWordRegister,            std::uint16_t, UPDATE_STATUS_WORD);
    CANOPEN_DEFINE_RW_OBJECT(0x605A, 0, QuickStopOptionCode,           std::int16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x605B, 0, ShutdownOptionCode,            std::int16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x605C, 0, DisableOperationOptionCode,    std::int16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x605D, 0, HaltOptionCode,                std::int16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x605E, 0, FaultReactionOptionCode,       std::int16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6060, 0, ModesOfOperation,              std::int8_t, UPDATE_OPERATION_MODE);
    CANOPEN_DEFINE_RO_OBJECT(0x6062, 0, PositionDemandValue,           std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x6063, 0, PositionActualInternalValue,   std::int32_t, UPDATE_JOINT_POSITION);
    CANOPEN_DEFINE_RW_OBJECT(0x6065, 0, FollowingErrorWindow,          std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6066, 0, FollowingErrorTimeout,         std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6067, 0, PositionWindow,                std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6068, 0, PositionWindowTimeout,         std::uint32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x6069, 0, VelocitySensorActualValue,     std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x606B, 0, VelocityDemandValue,           std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x606C, 0, VelocityActualValue,           std::int32_t, UPDATE_JOINT_VELOCITY);
    CANOPEN_DEFINE_RW_OBJECT(0x606D, 0, VelocityWindow,                std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x606E, 0, VelocityWindowTime,            std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x606F, 0, VelocityThreshold,             std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6070, 0, VelocityThresholdTime,         std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6071, 0, TargetTorque,                  std::int16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6072, 0, MaxTorque,                     std::uint16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6073, 0, MaxCurrent,                    std::uint16_t, UPDATE_JOINT_LIMITS);
    CANOPEN_DEFINE_RO_OBJECT(0x6074, 0, TorqueDemand,                  std::int16_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x6075, 0, MotorRatedCurrent,             std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RO_OBJECT(0x6076, 0, MotorRatedTorque,              std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RO_OBJECT(0x6077, 0, TorqueActualValue,             std::int16_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x6078, 0, CurrentActualValue,            std::int16_t, UPDATE_JOINT_CURRENT);
    CANOPEN_DEFINE_RO_OBJECT(0x6079, 0, DCLinkCircuitVoltage,          std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x607A, 0, TargetPosition,                std::int32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x607B, 1, PositionRangeLimitMin,         std::int32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x607B, 2, PositionRangeLimitMax,         std::int32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x607D, 1, SoftwarePositionLimitMin,      std::int32_t, UPDATE_JOINT_LIMITS);
    CANOPEN_DEFINE_RW_OBJECT(0x607D, 2, SoftwarePositionLimitMax,      std::int32_t, UPDATE_JOINT_LIMITS);
    CANOPEN_DEFINE_RW_OBJECT(0x607E, 0, Polarity,                      std::int8_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x607F, 0, MaxProfileVelocity,            std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6080, 0, MaxMotorSpeed,                 std::int32_t, UPDATE_JOINT_LIMITS);
    CANOPEN_DEFINE_RW_OBJECT(0x6081, 0, ProfileVelocity,               std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6082, 0, EndVelocity,                   std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6083, 0, ProfileAcceleration,           std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6084, 0, ProfileDeceleration,           std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6085, 0, QuickStopDeceleration,         std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6086, 0, MotionProfileType,             std::int16_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x6087, 0, TorqueSlope,                   std::uint32_t, 0);
    CANOPEN_DEFINE_RW_OBJECT(0x608F, 1, PositionEncoderResolutionNum,  std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x608F, 2, PositionEncoderResolutionDen,  std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6090, 1, VelocityEncoderResolutionNum,  std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6090, 2, VelocityEncoderResolutionDen,  std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6091, 1, GearRatioNum,                  std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6091, 2, GearRatioDen,                  std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6092, 1, FeedConstantNum,               std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6092, 2, FeedConstantDen,               std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6096, 1, VelocityFactorNum,             std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6096, 2, VelocityFactorDen,             std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6097, 1, AccelerationFactorNum,         std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x6097, 2, AccelerationFactorDen,         std::uint32_t, UPDATE_FACTORS);
    CANOPEN_DEFINE_RW_OBJECT(0x60C5, 0, MaxAcceleration,               std::int32_t, UPDATE_JOINT_LIMITS);
    CANOPEN_DEFINE_RW_OBJECT(0x60C6, 0, MaxDeceleration,               std::int32_t, UPDATE_JOINT_LIMITS);
    CANOPEN_DEFINE_RO_OBJECT(0x60F4, 0, FollowingErrorActualValue,     std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x60FA, 0, ControlEffort,                 std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x60FC, 0, PositionDemandInternalValue,   std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x60FF, 0, TargetVelocity,                std::int32_t, 0);
    CANOPEN_DEFINE_RO_OBJECT(0x6502, 0, SupportedDriveModes,           std::uint32_t, 0);


    /** Representation of the heartbeat (NMT state)
     */
    struct Heartbeat
    {
        static const uint64_t UPDATE_ID = UPDATE_HEARTBEAT;
    };

    /** Representation of the control word
     *
     * The control word changes the drive's operational state
     */
    struct ControlWord : ControlWordRegister
    {
        enum Transition
        {
            SHUTDOWN,
            SWITCH_ON,
            ENABLE_OPERATION,
            DISABLE_VOLTAGE,
            QUICK_STOP,
            DISABLE_OPERATION,
            FAULT_RESET
        };

        ControlWord(Transition transition, bool enable_halt)
            : transition(transition)
            , enable_halt(enable_halt) {}

        Transition transition;
        bool enable_halt;
    };

    /** Representation of the status word
     *
     * The status word is the main representation of the drive's state
     */
    struct StatusWord : StatusWordRegister
    {
        enum State
        {
            NOT_READY_TO_SWITCH_ON,
            SWITCH_ON_DISABLED,
            READY_TO_SWITCH_ON,
            SWITCH_ON,
            OPERATION_ENABLED,
            QUICK_STOP_ACTIVE,
            FAULT_REACTION_ACTIVE,
            FAULT
        };

        struct UnknownState : public std::runtime_error
        {
            using std::runtime_error::runtime_error;
        };

        uint16_t raw;
        State state;
        bool voltageEnabled;
        bool warning;
        bool targetReached;
        bool internalLimitActive;

        StatusWord(uint16_t raw, State state, bool voltageEnabled, bool warning, bool targetReached, bool internalLimitActive)
            : raw(raw)
            , state(state)
            , voltageEnabled(voltageEnabled)
            , warning(warning)
            , targetReached(targetReached)
            , internalLimitActive(internalLimitActive) {}
    };
}

#endif
