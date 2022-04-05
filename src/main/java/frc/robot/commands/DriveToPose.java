package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class DriveToPose extends CommandBase {
    protected Pose2d targetPose;

    ProfiledPIDController distanceController;

    public DriveToPose(Pose2d targetPose, double toleranceDistance) {
        this.targetPose = targetPose;
        addRequirements(RobotContainer.CHASSIS);

        distanceController = new ProfiledPIDController(1.0, 0.001, 0.075, new TrapezoidProfile.Constraints(Chassis.MAX_DRIVE_SPEED, Chassis.MAX_DRIVE_ACCELERATION));
        distanceController.setTolerance(toleranceDistance);
        distanceController.setGoal(0.0);
    }
    
    public void initialize() {
        var error = RobotContainer.CHASSIS.getCurrentPose().minus(targetPose);
        var errorDistance = error.getTranslation().getNorm(); // TODO: needed for trapezoidal
        distanceController.reset(errorDistance);
        RobotContainer.CHASSIS.setHeading(targetPose.getRotation().getRadians(), true);
    }

    public void execute() {
        var error = RobotContainer.CHASSIS.getCurrentPose().minus(targetPose);
        var errorDistance = error.getTranslation().getNorm();
        
        double distanceOutput = distanceController.calculate(errorDistance);

        double x = distanceOutput * error.getTranslation().getX() / errorDistance;
        double y = distanceOutput * error.getTranslation().getY() / errorDistance;

        if (distanceController.atSetpoint()) {
            x = 0.0;
            y = 0.0;
        }

        // The order of these is important.
        System.out.println("Driving");
        System.out.println(RobotContainer.CHASSIS.headingReached());
        System.out.println(distanceController.atGoal());
        RobotContainer.CHASSIS.robotDrive(x, y, 0.0);
        RobotContainer.CHASSIS.setHeading(targetPose.getRotation().getRadians(), false);
    }

    public boolean isFinished() {
        return distanceController.atGoal() && RobotContainer.CHASSIS.headingReached();
    }

    public void end(boolean interrupted) {
        RobotContainer.CHASSIS.stop();
        if (!interrupted) {
            System.out.println("Pose Reached");
        }
    }
}
