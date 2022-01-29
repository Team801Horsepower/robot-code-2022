package frc.robot.components;

public interface AngleMotor {
    public void init();
    public default void periodic() {}
    public void setDesiredAngle(double angle);
    public double getCurrentAngle();
}
