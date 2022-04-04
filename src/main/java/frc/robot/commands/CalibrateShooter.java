package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CalibrateShooter extends CommandBase {
    public CalibrateShooter() {
        addRequirements(RobotContainer.SHOOTER);
        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Shooter Speed", 196.5);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.SHOOTER.setSpeed(SmartDashboard.getNumber("Shooter Speed", 0.0));
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.SHOOTER.stop();
    }
    
}
