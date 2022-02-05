package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.architecture.Drive;
import frc.robot.components.SwerveModule;
import frc.robot.components.DriveMotor;
import frc.robot.components.SwerveDrive;
import frc.robot.components.TurnMotor;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

// import java.util.Map;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends SubsystemBase {

    // Front
    // 2 1
    // 3 4

    private AHRS gyro;
    private Drive drive;

    private Pose2d pose;

    NetworkTableEntry error_x;
    NetworkTableEntry error_y;

    public Chassis() {
        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus. */
            /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
            /*
             * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.
             */
            gyro = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }

        pose = new Pose2d();

        drive = new SwerveDrive(
                new SwerveModule[] {
                        new SwerveModule(new DriveMotor(Constants.POD_1_DRIVE, 1),
                                new TurnMotor(Constants.POD_1_TURN, 1),
                                Constants.DRIVE_METERS_PER_RADIAN),
                        new SwerveModule(new DriveMotor(Constants.POD_2_DRIVE, 1),
                                new TurnMotor(Constants.POD_2_TURN, 1),
                                Constants.DRIVE_METERS_PER_RADIAN),
                        new SwerveModule(new DriveMotor(Constants.POD_3_DRIVE, 1),
                                new TurnMotor(Constants.POD_3_TURN, 1),
                                Constants.DRIVE_METERS_PER_RADIAN),
                        new SwerveModule(new DriveMotor(Constants.POD_4_DRIVE, 1),
                                new TurnMotor(Constants.POD_4_TURN, 1),
                                Constants.DRIVE_METERS_PER_RADIAN),},
                new Translation2d[] {
                    new Translation2d(-1, 1),
                    new Translation2d(-1, -1),
                    new Translation2d(1, -1),
                    new Translation2d(1, 1)},
                gyro);

        NetworkTableInstance networkTableInst = NetworkTableInstance.getDefault();

        // Get the table within instance that contains the data. There can
        NetworkTable networkTable = networkTableInst.getTable("datatable");
        error_x = networkTable.getEntry("error_x");
        error_y = networkTable.getEntry("error_y");
    }

    /**
     * This method must be called before the Chassis is used
     */
    public void init() {
        gyro.calibrate();
        drive.init();
    }

    public void periodic() {
        drive.periodic();
        pose = drive.getCurrentPose();
    }

    /**
     * This method will be called once per scheduler run in Autonomous
     */

    public void directionDrive(double speed, double angle) {

        drive.setDesiredSpeeds(0, 0, angle);
    }

    public void stop() {
        drive.setDesiredSpeeds(new ChassisSpeeds());
    }

    public void robotDrive(double forward, double leftward, double omega) {
        drive.setDesiredSpeeds(forward * Constants.MAX_ROBOT_SPEED, leftward * Constants.MAX_ROBOT_SPEED, omega);
    }

    public void fieldDrive(double fieldForward, double fieldLeftward, double omega) {
        double yawAngle = -gyro.getYaw() * Math.PI / 180;
        double magnitude = Math.sqrt(Math.pow(fieldForward, 2) + Math.pow(fieldLeftward, 2));
        double robotAngle = (Math.PI / 2) - yawAngle + ((Math.atan2(fieldLeftward, fieldForward) + 2 * Math.PI) % (2 * Math.PI));
        double robotForward = Math.sin(robotAngle) * magnitude;
        double robotLeftward = -Math.cos(robotAngle) * magnitude;
        robotDrive(robotForward, robotLeftward, omega);
    }

    public Pose2d getCurrentPose() {
        return pose;
    }
}
