package frc.robot.architecture;

/**
 * Defines the methods for motors which can turn to face a specific angle.
 */
public interface AngleMotor {

    /**
     * Performs any required initialization. (ex. zero the encoder)
     */
    public default void init() {};

    /**
     * Performs any periodic tasks required by the motor. (ex. update PID loop)
     */
    public default void periodic() {}

    /**
     * Requests the motor face the desired angle.
     * 
     * @param angle The angle to face in radians.
     */
    public void setDesiredAngle(double angle);

    /**
     * Returns the current heading of the motor.
     * 
     * @return The current angle the motor is facing.
     * @apiNote This is not necessarily the last desired angle.
     */
    public double getCurrentAngle();
}
