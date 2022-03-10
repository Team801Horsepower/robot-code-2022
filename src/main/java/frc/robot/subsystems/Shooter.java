package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.components.Neo;

public class Shooter extends SubsystemBase {

    public final Neo FLYWHEEL;

    public Shooter() {
        FLYWHEEL = new Neo(Constants.SHOOTER);
        FLYWHEEL.setGearRatio(Constants.SHOOTER_GEAR_RATIO);

        int speedPid = FLYWHEEL.getSpeedPid();
        FLYWHEEL.PID.setP(1.0, speedPid);
        FLYWHEEL.PID.setI(0.0, speedPid);
        FLYWHEEL.PID.setD(0.0, speedPid);
        FLYWHEEL.PID.setFF(0.2, speedPid);
    }

    /**
     * Set the target speed of the flywheel.
     *
     * @param speed speed is rad/s
     */
    public void setSpeed(double speed) {
        FLYWHEEL.setDesiredSpeed(speed);
    }

    public boolean ready() {
        return FLYWHEEL.velocityReached(1.0);
    }

    public void stop() {
        FLYWHEEL.setPower(0.0);
    }

    public void freeBall() {
        FLYWHEEL.setDesiredPosition(-Math.PI);
    }
    
    public boolean freeComplete() {
        return FLYWHEEL.positionReached(0.5);
    }
}
