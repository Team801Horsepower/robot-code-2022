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

    public static final double MAX_DRIVE_SPEED = 7.0;
    public static final double MAX_DRIVE_ACCELERATION = 2.0;
    public static final double MAX_TURN_SPEED = Math.PI;
    public static final double MAX_TURN_ACCELERATION = Math.PI;

    // Front
    // 2 1
    // 3 4

    // Swerve Pod Motor CAN IDs
    private static final int POD_1_DRIVE = 9; // Right Front
    private static final int POD_1_TURN = 10;
    @SuppressWarnings("unused")
    private static final int POD_1_THROUGHBORE = 1;

    private static final int POD_2_DRIVE = 12; // Left Front
    private static final int POD_2_TURN = 11;
    @SuppressWarnings("unused")
    private static final int POD_2_THROUGHBORE = 2;

    private static final int POD_3_DRIVE = 20; // Left Rear
    private static final int POD_3_TURN = 19;
    @SuppressWarnings("unused")
    private static final int POD_3_THROUGHBORE = 3;

    private static final int POD_4_DRIVE = 1; // Right Rear
    private static final int POD_4_TURN = 2;
    @SuppressWarnings("unused")
    private static final int POD_4_THROUGHBORE = 4;

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
                new SwerveModule2020(POD_1_DRIVE, POD_1_TURN, false),
                new SwerveModule2020(POD_2_DRIVE, POD_2_TURN, false),
                new SwerveModule2020(POD_3_DRIVE, POD_3_TURN, true),
                new SwerveModule2020(POD_4_DRIVE, POD_4_TURN, true)
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
        if (speed > MAX_DRIVE_SPEED * MAX_DRIVE_SPEED)
        {
            speed = Math.sqrt(speed);
            forward = forward / speed * MAX_DRIVE_SPEED;
            leftward = leftward / speed * MAX_DRIVE_SPEED;
        }
        if (Math.abs(angular) > MAX_TURN_SPEED) {
            angular = Math.signum(angular) * MAX_TURN_SPEED;
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

    public void fieldDrive(ChassisSpeeds speeds) {
        fieldDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public void stop() {
        drive.setDesiredSpeeds(new ChassisSpeeds());
    }

    public Pose2d getCurrentPose() {
        return pose;
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
