package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.architecture.Drive;
import frc.robot.commands.PathPlannerControllerCommand;
import frc.robot.components.AHRSGyroEncoder;
import frc.robot.components.SwerveDrive;
import frc.robot.components.SwerveModule;
import frc.robot.components.SwerveModule2020;

/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends SubsystemBase {

    // Front
    // 2 1
    // 3 4

    private AHRSGyroEncoder gyro;
    private Drive drive;

    private Pose2d pose;
    private Field2d field = new Field2d();

    public Chassis() {
        super();
        SmartDashboard.putData(field);

        gyro = new AHRSGyroEncoder();
        gyro.SENSOR.calibrate();

        pose = new Pose2d();

        drive = new SwerveDrive(
            new SwerveModule[] {
                new SwerveModule2020(Constants.POD_1_DRIVE, Constants.POD_1_TURN, false),
                new SwerveModule2020(Constants.POD_2_DRIVE, Constants.POD_2_TURN, false),
                new SwerveModule2020(Constants.POD_3_DRIVE, Constants.POD_3_TURN, true),
                new SwerveModule2020(Constants.POD_4_DRIVE, Constants.POD_4_TURN, true)
            },
            // x is forward 
            // y is leftward
            new Translation2d[] {
                new Translation2d(1, -1),
                new Translation2d(1, 1),
                new Translation2d(-1, 1),
                new Translation2d(-1, -1)
            },
            gyro
        );
    }

    /**
     * This method should be called upon the robotInit (regardless of mode)
     */
    public void init(Pose2d initialPose) {
        gyro.setPosition(initialPose.getRotation().getRadians());
        drive.init();
        drive.resetPose(initialPose);
    }

    public void periodic() {
        drive.periodic();
        pose = drive.getCurrentPose();
        field.setRobotPose(pose);
        // System.out.println("pose: " + pose);
    }

    /**
     * Drive at the specified speeds in relation to the robot cooridate system
     * 
     * @param forward the speed in m/s to drive in the robot's forward direction
     * @param leftward the speed in m/s to drive in the robot's leftwards direction
     * @param angular the speed in rad/s in the counter-clockwise direction
     */
    public void robotDrive(double forward, double leftward, double angular) {
        double speed = forward * forward + leftward * leftward;
        if (speed > Constants.MAX_ROBOT_DRIVE_SPEED * Constants.MAX_ROBOT_DRIVE_SPEED)
        {
            speed = Math.sqrt(speed);
            forward = forward / speed * Constants.MAX_ROBOT_DRIVE_SPEED;
            leftward = leftward / speed * Constants.MAX_ROBOT_DRIVE_SPEED;
        }
        drive.setDesiredSpeeds(forward, leftward, angular);
    }

    /**
     * Drive at the specified speeds in relation to the field cooridate system.
     * 
     *
     * @param fieldForward the speed in m/s to drive forward
     * @param fieldLeftward the speed in m/s to drive leftward
     * @param angular the speed in rad/s in the counter-clockwise direction
     */
    public void fieldDrive(double fieldForward, double fieldLeftward, double angular) {
        double robotAngle = gyro.getCurrentAngle();
        robotDrive(
            fieldForward * Math.cos(robotAngle) + fieldLeftward * Math.sin(robotAngle),
            - fieldForward * Math.sin(robotAngle) + fieldLeftward * Math.cos(robotAngle),
            angular
        );
    }

    public void stop() {
        drive.setDesiredSpeeds(new ChassisSpeeds());
    }

    public Pose2d getCurrentPose() {
        return pose;
    }

    public Command generatePathFollowCommand(PathPlannerTrajectory trajectory,
            PIDController xController, PIDController yController,
            ProfiledPIDController thetaController) {
        return new PathPlannerControllerCommand(trajectory, this::getCurrentPose, xController,
                yController, thetaController, drive::setDesiredSpeeds, this);
    }

    public void reset() {
        System.out.println("Resetting");
        gyro.SENSOR.calibrate();
        gyro.setPosition(0.0);
        drive.reset();
        pose = new Pose2d();
        drive.resetPose(pose);
        field.setRobotPose(pose);
    }
}
