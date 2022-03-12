package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AimShooter extends DriveToPose {
    boolean goalLocated = false;

    public AimShooter() {
        super(RobotContainer.CHASSIS.getCurrentPose(), Units.inchesToMeters(1.0), Units.degreesToRadians(1.0));
        addRequirements(RobotContainer.VISION);
    }

    @Override
    public void initialize() {
        var goalLocation = RobotContainer.VISION.getGoalLocation();
        if (goalLocation == null) {
            goalLocated = false;
            targetPose = RobotContainer.CHASSIS.getCurrentPose();
            return;
        }
        var rotation = new Rotation2d(-Math.atan2(goalLocation.getX(), goalLocation.getY()));
        targetPose = new Pose2d(RobotContainer.CHASSIS.getCurrentPose().getTranslation(), rotation);
        goalLocated = true;

        super.initialize();
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
