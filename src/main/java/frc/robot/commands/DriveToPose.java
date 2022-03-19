package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveToPose extends CommandBase {
    protected Pose2d targetPose;

    PIDController distanceController;
    ProfiledPIDController omegaController;

    public DriveToPose(Pose2d targetPose, double toleranceDistance, double toleranceAngle) {
        this.targetPose = targetPose;

        addRequirements(RobotContainer.CHASSIS);

        distanceController = new PIDController(0.4, 0.0, 0.075);
        distanceController.setTolerance(toleranceDistance);
        distanceController.setSetpoint(0.0);
        omegaController = new ProfiledPIDController(0.5, 0.0, 0.0, new TrapezoidProfile.Constraints(Constants.PATH_MAX_ANGULAR_VELOCITY, Constants.PATH_MAX_ANGULAR_ACCELERATION));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        omegaController.setTolerance(toleranceAngle);
        omegaController.setGoal(0.0);

        SmartDashboard.putData(distanceController);
        SmartDashboard.putData(omegaController);
    }
    
    public void initialize() {
        var error = RobotContainer.CHASSIS.getCurrentPose().minus(targetPose);
        var errorDistance = error.getTranslation().getNorm();
        var errorOmega = error.getRotation().getRadians();
        distanceController.reset();
        omegaController.reset(errorOmega);
    }

    public void execute() {
        var error = RobotContainer.CHASSIS.getCurrentPose().minus(targetPose);
        var errorDistance = error.getTranslation().getNorm();
        var errorOmega = error.getRotation().getRadians();
        
        double distanceErr = distanceController.calculate(errorDistance);
        double distanceCalculation = distanceErr - 0.5;
        double omegaCalculation = omegaController.calculate(errorOmega);

        double x = distanceCalculation * error.getTranslation().getX() / errorDistance;
        double y = distanceCalculation * error.getTranslation().getY() / errorDistance;

        if (distanceController.atSetpoint()) {
            x = 0.0;
            y = 0.0;
        }

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
