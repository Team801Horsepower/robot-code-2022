package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
            RobotContainer.SHOOTER.setRange(goalLocation.getY());
        } else {
            RobotContainer.SHOOTER.setRange(3.0);
        }
        RobotContainer.SHOOTER.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("RPM Reached", RobotContainer.SHOOTER.FLYWHEEL.velocityReached(Units.rotationsPerMinuteToRadiansPerSecond(50.0)));
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.SHOOTER.ready();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
}
