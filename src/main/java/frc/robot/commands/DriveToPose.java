package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;

public class DriveToPose extends CommandBase {
    protected Pose2d targetPose;

    PIDController distanceController; // TODO: Change to trapezoidal and use Chassis.MAX_SPEED;

    public DriveToPose(Pose2d targetPose, double toleranceDistance) {
        this.targetPose = targetPose;
        addRequirements(RobotContainer.CHASSIS);

        distanceController = new PIDController(0.4, 0.0, 0.075);
        distanceController.setTolerance(toleranceDistance);
        distanceController.setSetpoint(0.0);
    }
    
    public void initialize() {
        var error = RobotContainer.CHASSIS.getCurrentPose().minus(targetPose);
        var errorDistance = error.getTranslation().getNorm(); // TODO: needed for trapezoidal
        distanceController.reset();
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
        RobotContainer.CHASSIS.fieldDrive(x, y, 0.0);
        RobotContainer.CHASSIS.setHeading(targetPose.getRotation().getRadians(), false);
    }

    public boolean isFinished() {
        return distanceController.atSetpoint() && RobotContainer.CHASSIS.headingReached();
    }

    public void end(boolean interrupted) {
        RobotContainer.CHASSIS.stop();
        if (!interrupted) {
            System.out.println("Pose Reached");
        }
    }
}
