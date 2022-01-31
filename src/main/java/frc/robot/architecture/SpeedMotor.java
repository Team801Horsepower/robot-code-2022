package frc.robot.architecture;

/**
 * Defines the methods for motors which
 * can rotate at a specific rate.
 */
public interface SpeedMotor {

    /**
     * Performs any required initialization.
     * (ex. zero the encoder)
     */
    public default void init() {
    }

    /**
     * Performs any periodic tasks required by
     * the motor. (ex. update PID loop)
     */
    public default void periodic() {
    }

    /**
     * Requests the motor rotate at the desired speed (rpm).
     * 
     * @param speed The speed to rotate in RPM.
     */
    public void setDesiredSpeed(double speed);

    /**
     * Returns the current speed of the motor.
     * 
     * @return The speed the motor is running at in RPM.
     * @apiNote This is not necessarily the last 
     * desired speed.
     */
    public double getCurrentSpeed();
}
