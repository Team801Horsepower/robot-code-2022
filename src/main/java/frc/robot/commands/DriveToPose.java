package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveToPose extends CommandBase {
    protected Pose2d targetPose;

    ProfiledPIDController distanceController;
    ProfiledPIDController omegaController;

    public DriveToPose(Pose2d targetPose, double toleranceDistance, double toleranceAngle) {
        this.targetPose = targetPose;

        addRequirements(RobotContainer.CHASSIS);

        distanceController = new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Constants.PATH_MAX_VELOCITY, Constants.PATH_MAX_ACCELERATION));
        distanceController.setTolerance(toleranceDistance);
        distanceController.setGoal(0.0);
        omegaController = new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Constants.PATH_MAX_ANGULAR_VELOCITY, Constants.PATH_MAX_ANGULAR_ACCELERATION));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        omegaController.setTolerance(toleranceAngle);
        omegaController.setGoal(0.0);
    }
    
    public void initialize() {
        var error = RobotContainer.CHASSIS.getCurrentPose().minus(targetPose);
        var errorDistance = error.getTranslation().getDistance(Constants.ORIGIN);
        var errorOmega = error.getRotation().getRadians();
        distanceController.reset(errorDistance);
        omegaController.reset(errorOmega);
    }

    public void execute() {
        var error = RobotContainer.CHASSIS.getCurrentPose().minus(targetPose);
        var errorDistance = error.getTranslation().getDistance(Constants.ORIGIN);
        var errorOmega = error.getRotation().getRadians();

        double distanceCalculation = distanceController.calculate(errorDistance);
        double omegaCalculation = omegaController.calculate(errorOmega);

        double x = distanceCalculation * error.getTranslation().getX() / errorDistance;
        double y = distanceCalculation * error.getTranslation().getY() / errorDistance;

        System.out.println("x: " + x + ", y: " + y + ", omega: " + omegaCalculation);
        RobotContainer.CHASSIS.fieldDrive(x, y, omegaCalculation);
    }

    public boolean isFinished() {
        return distanceController.atSetpoint() && omegaController.atSetpoint();
    }

    public void end(boolean interrupted) {
        RobotContainer.CHASSIS.stop();
        if (!interrupted) {
            System.out.println("Pose Reached");
        }
    }
}
