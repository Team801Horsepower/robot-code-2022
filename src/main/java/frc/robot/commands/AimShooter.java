package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AimShooter extends CommandBase {
    boolean goalLocated = false;

    public AimShooter() {
        addRequirements(RobotContainer.VISION, RobotContainer.CHASSIS);
    }

    @Override
    public void initialize() {
        var goalLocation = RobotContainer.VISION.getGoalLocation();
        goalLocated = goalLocation != null;
    }

    @Override
    public boolean isFinished() {
        return (goalLocated && super.isFinished()) || !goalLocated;
    }

    @Override
    public void execute() {
        if (goalLocated) {
            super.execute();
        }
    }
}
