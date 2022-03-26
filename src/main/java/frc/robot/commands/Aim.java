package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Aim extends CommandBase {

    private static final double RANGE_P = 1.0;
    private static final double RANGE_I = 0.0;
    private static final double RANGE_D = 0.0;

    private static final double ANGLE_P = 1.0;
    private static final double ANGLE_I = 0.0;
    private static final double ANGLE_D = 0.0;

    private boolean goalLocated = false;

    private ProfiledPIDController angleController;

    public Aim() {
        addRequirements(RobotContainer.VISION, RobotContainer.CHASSIS);
        angleController = new ProfiledPIDController(ANGLE_P, ANGLE_I, ANGLE_D, new TrapezoidProfile.Constraints(Chassis.MAX_TURN_SPEED, Chassis.MAX_TURN_ACCELERATION));
        angleController.setGoal(0.0);
    }

    @Override
    public void initialize() {
        var goalLocation = RobotContainer.VISION.getGoalLocation();
        goalLocated = goalLocation != null;
    }

    @Override
    public boolean isFinished() {
        return (goalLocated && angleController.atGoal()) || !goalLocated;
    }

    @Override
    public void execute() {
        Translation2d goalLocation = RobotContainer.VISION.getGoalLocation();
        if (goalLocation != null) {
            double angleSpeed = angleController.calculate(Math.atan2(goalLocation.getX(), goalLocation.getY()));
            RobotContainer.CHASSIS.robotDrive(0.0, 0.0, angleSpeed);
        }
    }
}
