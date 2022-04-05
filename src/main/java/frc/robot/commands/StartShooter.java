package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class StartShooter extends CommandBase {

    Translation2d goalLocation;
    boolean setRange;

    public StartShooter(boolean setRange) {
        this.setRange = setRange;
        addRequirements(RobotContainer.SHOOTER);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        goalLocation = RobotContainer.VISION.getGoalLocation();
        if(setRange) {
            if (goalLocation != null) {
                RobotContainer.SHOOTER.setRange(goalLocation.getNorm());
            } else {
                RobotContainer.SHOOTER.setRange(3.0);
            }
        }
        RobotContainer.SHOOTER.start();
    }
    
    @Override
    public boolean isFinished() {
        return RobotContainer.SHOOTER.ready();
    }
}
