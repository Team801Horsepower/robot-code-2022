package frc.robot.components;

public interface SpeedMotor {
    public default void init() {}
    public default void periodic() {}
    public void setDesiredSpeed(double speed);
    public double getCurrentSpeed();
}
