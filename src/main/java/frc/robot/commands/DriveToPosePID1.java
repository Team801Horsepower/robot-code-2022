package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.util.Units;

public class DriveToPosePID1 extends CommandBase {
    protected Pose2d targetPose;

    PIDController distanceController;
    PIDController omegaController;

    public DriveToPosePID1(Pose2d targetPose) {
        this.targetPose = targetPose;

        addRequirements(RobotContainer.chassis);

        distanceController = new PIDController(1.0, 0.0, 0.0);
        distanceController.setTolerance(Units.inchesToMeters(1.0));
        omegaController = new PIDController(1.0, 0.0, 0.0);
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        omegaController.setTolerance(Units.degreesToRadians(2.0));
    }

    public void execute() {
        var error = targetPose.minus(RobotContainer.chassis.getCurrentPose());
        var errorDistance = error.getTranslation().getDistance(Constants.ORIGIN);
        var errorOmega = error.getRotation().getRadians();

        double distanceCalculation = distanceController.calculate(errorDistance);
        double omegaCalculation = omegaController.calculate(errorOmega);

        double x = distanceCalculation * error.getTranslation().getX() / errorDistance;
        double y = distanceCalculation * error.getTranslation().getY() / errorDistance;

        RobotContainer.chassis.fieldDrive(x, y, omegaCalculation);
    }

    public boolean isFinished() {
        return distanceController.atSetpoint() && omegaController.atSetpoint();
    }
}
