package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AimShooter extends DriveToPose {
    boolean goalLocated = false;

    public AimShooter() {
        super(RobotContainer.CHASSIS.getCurrentPose());
        addRequirements(RobotContainer.VISION, RobotContainer.CHASSIS);
    }

    @Override
    public void initialize() {
        var targets = RobotContainer.VISION.locateTargets();
        var goalLocation = RobotContainer.VISION.locateGoal(targets);
        if (goalLocation == null) {
            return;
        }
        var rotation = new Rotation2d(-Math.atan2(goalLocation.getX(), goalLocation.getY()));
        var transform = new Transform2d(new Translation2d(), rotation);
        var newPose = RobotContainer.CHASSIS.getCurrentPose().transformBy(transform);
        targetPose = newPose;
        goalLocated = true;

        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return goalLocated && super.isFinished();
    }

    @Override
    public void execute() {
        if (goalLocated) {
            super.execute();
        }
    }
}
