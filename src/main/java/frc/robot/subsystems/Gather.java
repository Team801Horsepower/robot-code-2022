package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo550;

public class Gather extends SubsystemBase {

    private final Neo550 WHEELS;
    private final Neo550 ARM;

    public Gather() {
        ARM = new Neo550(Constants.GATHER_ARM);
        ARM.setGearRatio(Constants.GATHER_ARM_GEAR_RATIO);
        ARM.setPosition(0);

        WHEELS = new Neo550(Constants.GATHER_WHEELS);
        WHEELS.setGearRatio(Constants.GATHER_WHEELS_GEAR_RATIO);
    }

    public void lower() {
        ARM.setDesiredPosition(Math.PI / 8);
    }

    public void raise() {
        ARM.setDesiredPosition(0.0);
    }

    public void forward(double speed) {
        WHEELS.setDesiredSpeed(speed);
    }

    public void reverse(double speed) {
        WHEELS.setDesiredSpeed(-speed);
    }

    public void stop() {
        WHEELS.setDesiredSpeed(0.0);
    }
    
    public void tampBall() {
        WHEELS.setDesiredAngle(WHEELS.getCurrentAngle() - Math.PI / 4);
    }
    
    public void popBall() {
        
    }

    public boolean isRaised() {
        return Math.abs(ARM.getCurrentPosition()) < 0.1;
    }
}
