package frc.robot.subsystems;

import org.ejml.data.FScalar;

import frc.robot.Constants;
import frc.robot.components.Neo;

public class Shooter {

    private final Neo FLYWHEEL;
    
    public Shooter() {
        FLYWHEEL = new Neo(Constants.SHOOTER);
        FLYWHEEL.setGearRatio(Constants.SHOOTER_GEAR_RATIO);

        int speedPid = FLYWHEEL.getSpeedPid();
    }

    /**
     * Set the target speed of the flywheel.
     * 
     * @param speed speed is rad/s
     */
    public void setSpeed(double speed) {
        FLYWHEEL.setDesiredSpeed(speed);
    }

    public void freeBall() {
        
    }
}
