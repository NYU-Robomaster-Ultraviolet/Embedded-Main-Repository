#ifndef SHOOTER_SUBSYSTEM_HPP_
#define SHOOTER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "modm/math/filter/pid.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/util_macros.hpp"
#include "controls/standard/standard_constants.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/drivers.hpp"
#include "tap/algorithms/ramp.hpp"
#include "modm/math/filter/ramp.hpp"

namespace shooter
{
/**
 * A bare bones subsystem to control PWM flywheels
 */
class ShooterSubsystem : public tap::control::Subsystem
{
public:
    /**
     * This max output is measured in the c620 robomaster translated current.
     * Per the datasheet, the controllable current range is -16384 ~ 0 ~ 16384.
     * The corresponding speed controller output torque current range is
     * -20 ~ 0 ~ 20 A.
     *
     * For this demo, we have capped the output at 8000. This should be more
     * than enough for what you are doing.
     */
    static constexpr float MAX_CURRENT_OUTPUT = 8000.0f;

    /**
     * Constructs a new ChassisSubsystem with default parameters specified in
     * the private section of this class.
     */
    ShooterSubsystem(tap::Drivers *drivers)
        : tap::control::Subsystem(drivers),
        flywheelRamp(0.05f, 0.05f, 0.15f)
    {
    }

    ShooterSubsystem(const ShooterSubsystem &other) = delete;

    ShooterSubsystem &operator=(const ShooterSubsystem &other) = delete;

    ~ShooterSubsystem() = default;

    void initialize() override;

    /**
     * No-op function that is a placeholder because all interactions with motors are done
     * in setDesiredOutput.
     */
    void refresh() override;

    void setDesiredOutput(float output);

    float findRampOutput(float output);

    //void updateRpmPid(tap::algorithms::Ramp* ramp, tap::gpio::Pwm::Pin const flywheel);

    const tap::gpio::Pwm::Pin &getFlywheel1() const { return flywheel1; }
    const tap::gpio::Pwm::Pin &getFlywheel2() const { return flywheel2; }

private:
    ///< Motors.  Use these to interact with any dji style motors.
    tap::gpio::Pwm::Pin flywheel1 = tap::gpio::Pwm::C1;
    tap::gpio::Pwm::Pin flywheel2 = tap::gpio::Pwm::C2;

    // PID controllers for RPM feedback from wheels
    modm::filter::Ramp<float> flywheelRamp;

    ///< Any user input is translated into desired RPM for each motor.

    // Scale factor for converting joystick movement into RPM setpoint
    //static constexpr float RPM_SCALE_FACTOR = 4000.0f;

};  // class ChassisSubsystem

}  // namespace chassis


#endif  // CHASSIS_SUBSYSTEM_HPP_