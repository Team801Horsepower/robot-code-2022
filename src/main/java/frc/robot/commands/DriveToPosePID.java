package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotContainer;
import frc.robot.utilities.Utils;
import edu.wpi.first.math.util.Units;

public class DriveToPosePID extends ParallelCommandGroup {
    Pose2d targetPose;

    double angle = 0.0;
    double x = 0.0;
    double y = 0.0;

    PIDController angleController;
    PIDController xController;
    PIDController yController;

    public DriveToPosePID(Pose2d targetPose) {
        this.targetPose = targetPose;

        addRequirements(RobotContainer.chassis);

        var anglePID = new PIDCommand(
            new PIDController(1.0, 1.0, 1.0),
            () -> RobotContainer.chassis.getCurrentPose().getRotation().getRadians(),
            () -> this.targetPose.getRotation().getRadians(),
            output -> angle = output
        );
        angleController = anglePID.getController();
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(5.0));

        var xPID = new PIDCommand(
            new PIDController(0.5, 0.5, 0.5),
            () -> RobotContainer.chassis.getCurrentPose().getX(),
            () -> this.targetPose.getX(),
            output -> x = output
        );
        xController = xPID.getController();
        xController.setTolerance(Units.inchesToMeters(1.0));

        var yPID = new PIDCommand(
            new PIDController(0.5, 0.5, 0.5),
            () -> RobotContainer.chassis.getCurrentPose().getY(),
            () -> this.targetPose.getY(),
            output -> y = output
        );
        yController = yPID.getController();
        yController.setTolerance(Units.inchesToMeters(1.0));

        addCommands(anglePID, xPID, yPID);
    }

    @Override
    public void execute() {
        super.execute();
        System.out.println("x: " + x + ", y: " + y + ", angle: " + angle);
        double locAngle = angleController.atSetpoint() ? 0.0 : angle;
        double locX = xController.atSetpoint() ? 0.0 : x;
        double locY = yController.atSetpoint() ? 0.0 : y;
        RobotContainer.chassis.fieldDrive(locX, -locY, -locAngle);
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint();
    }
}
