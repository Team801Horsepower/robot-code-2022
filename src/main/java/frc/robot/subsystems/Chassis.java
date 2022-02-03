package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.architecture.Drive;
import frc.robot.components.SwerveModule;
import frc.robot.components.DriveMotor;
import frc.robot.components.SwerveDrive;
import frc.robot.components.TurnMotor;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.utilities.Utils;
import frc.robot.utilities.PID;
import frc.robot.utilities.RollingAverage;

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

    private boolean initialized = false;

    // Speed component for rotation about the Z axis. [-x, x]
    private static double vTheta;

    // heading about a unit circle in radians.
    private final double joystickTurnMultiplier = 50.0;
    private static double desiredHeading; // rotates about the Z axis [0,360) deg.
    private static double currentHeading; // rotates about the Z axis [0,360) deg.

    private final double headingThreshold = 0.05;
    private final int headingAverageNumberOfSamples = 5;

    private PID headingPID;
    private RollingAverage averageHeading;

    NetworkTableEntry error_x;
    NetworkTableEntry error_y;

    public Chassis() {

        vTheta = 0;

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

        drive = new SwerveDrive(
                new SwerveModule[] {
                        new SwerveModule(new DriveMotor(Constants.POD_1_DRIVE, 1),
                                new TurnMotor(Constants.POD_1_TURN, 1),
                                Constants.DRIVE_METERS_PER_ROTATION),
                        new SwerveModule(new DriveMotor(Constants.POD_2_DRIVE, 1),
                                new TurnMotor(Constants.POD_2_TURN, 1),
                                Constants.DRIVE_METERS_PER_ROTATION),
                        new SwerveModule(new DriveMotor(Constants.POD_3_DRIVE, 1),
                                new TurnMotor(Constants.POD_3_TURN, 1),
                                Constants.DRIVE_METERS_PER_ROTATION),
                        new SwerveModule(new DriveMotor(Constants.POD_4_DRIVE, 1),
                                new TurnMotor(Constants.POD_4_TURN, 1),
                                Constants.DRIVE_METERS_PER_ROTATION),},
                new Translation2d[] {
                    new Translation2d(-1, 1),
                    new Translation2d(-1, -1),
                    new Translation2d(1, -1),
                    new Translation2d(1, 1)},
                gyro);

        headingPID = new PID(Constants.HEADING_P, Constants.HEADING_I, Constants.HEADING_D);
        averageHeading = new RollingAverage(headingAverageNumberOfSamples);

        // set initial desired heading to the current actual heading.
        desiredHeading = currentHeading = gyro.getYaw();

        // initially setup the PID parameters
        headingPID.setOutputLimits(Constants.HEADING_OUTPUT_LIMIT_LOW,
                Constants.HEADING_OUTPUT_LIMIT_HIGH);
        headingPID.setMaxIOutput(Constants.HEADING_MAX_I_OUT);
        headingPID.setOutputRampRate(Constants.HEADING_OUTPUT_RAMPRATE);
        headingPID.setOutputFilter(Constants.HEADING_OUTPUT_FILTER);
        headingPID.setAngleUnits(PID.AngleUnit.degrees);
        headingPID.setSetpointRange(Constants.HEADING_SETPOINT_RANGE);
        headingPID.setContinousInputRange(360);
        headingPID.setContinous(true); // lets PID know we are working with a continuous range
                                       // [0-360)

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
        initialized = true;
    }

    public void periodic() {
        drive.periodic();
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

    public void drive(double forward, double leftward, double omega) {
        drive.setDesiredSpeeds(forward, leftward, omega);
    }

    // grab the imu heading and crunch out the values used for navigation and
    // telemetry.
    // This method produces the heading input component to the motors from the PID
    // that holds the
    // desired angle. The error from the PID is sent to the motors in the vTheta
    // variable.
    private double IMUAngleProcessing() {
        // in degrees +/- 0 to 180 where CCW is - and CW is + //TODO Verify CW is
        // negative angle
        double yawAngle = gyro.getYaw();

        // System.out.printf("yawAngle: %.4f desired: %.4f curr: %.4f\n", yawAngle, desiredHeading,
        // currentHeading);

        // convert imu angle range to our [0, 360) range
        if (yawAngle < 0) {
            currentHeading = yawAngle + 360;
        } else {
            currentHeading = yawAngle;
        }

        return headingPID.getOutput(currentHeading, desiredHeading);
    }

}
