package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo;

public class Shooter extends SubsystemBase {

    public final Neo FLYWHEEL;
    
    public Shooter() {
        FLYWHEEL = new Neo(Constants.SHOOTER);
        FLYWHEEL.setGearRatio(Constants.SHOOTER_GEAR_RATIO);

        int speedPid = FLYWHEEL.getSpeedPid();
        FLYWHEEL.PID.setP(0.005, speedPid);
        FLYWHEEL.PID.setI(0.0, speedPid);
        FLYWHEEL.PID.setD(0.0, speedPid);
        FLYWHEEL.PID.setFF(0.002, speedPid);

        FLYWHEEL.CONTROLLER.setIdleMode(IdleMode.kBrake);

        int positionPid = FLYWHEEL.getPositionPid();
        FLYWHEEL.PID.setP(1.0, positionPid);
        FLYWHEEL.PID.setI(0.0, positionPid);
        FLYWHEEL.PID.setD(0.0, positionPid);

        SmartDashboard.putData(FLYWHEEL);
    }

    /**
     * Set the target speed of the flywheel.
     * 
     * @param speed speed is rad/s
     */
    public void setSpeed(double speed) {
        FLYWHEEL.setDesiredSpeed(speed);
    }

    public Command revFlywheel(double speed) {
        return FLYWHEEL.generateVelocityCommand(speed, 1.0, this);
    }

    public boolean ready() {
        return FLYWHEEL.velocityReached(1.0);
    }

    public void stop() {
        FLYWHEEL.setPower(0.0);
    }

    public Command freeBall() {
        return FLYWHEEL.generateRotationCommand(-2 * Math.PI * 2, Math.PI, this);
    }

    
}
