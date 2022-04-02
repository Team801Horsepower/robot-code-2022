package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RunShooter extends CommandBase {

    Translation2d goalLocation;

    public RunShooter() {
        addRequirements(RobotContainer.SHOOTER);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        goalLocation = RobotContainer.VISION.getGoalLocation();
        if (goalLocation != null) {
            RobotContainer.SHOOTER.setRange(Math.sqrt(Math.pow(goalLocation.getX(), 2.0) + Math.pow(goalLocation.getY(), 2.0)));
        } else {
            RobotContainer.SHOOTER.setRange(3.0);
        }
        RobotContainer.SHOOTER.start();
    }
    
    @Override
    public boolean isFinished() {
        return RobotContainer.SHOOTER.ready();
    }
}
