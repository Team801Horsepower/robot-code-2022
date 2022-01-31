package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.architecture.Drive;
import frc.robot.architecture.SwerveModule;
import frc.robot.components.DriveMotor;
import frc.robot.components.SwerveDrive;
import frc.robot.components.TurnMotor;
import frc.robot.Constants;
/**
 * Subsystem to control the entire drive base
 */
public class Chassis extends SubsystemBase {

    // Front
    // 2 1
    // 3 4

    private Gyro gyro;
    private Drive drive;

    private boolean initialized = false;

    public Chassis() {
        gyro = new ADXRS450_Gyro();
        drive = new SwerveDrive(
                new SwerveModule[] {
                    new SwerveModule(
                        new DriveMotor(Constants.POD_1_DRIVE, 1),
                        new TurnMotor(Constants.POD_1_TURN, 1),
                        Constants.DRIVE_METERS_PER_ROTATION
                    ),
                    new SwerveModule(
                        new DriveMotor(Constants.POD_2_DRIVE, 1),
                        new TurnMotor(Constants.POD_2_TURN, 1),
                        Constants.DRIVE_METERS_PER_ROTATION
                    ),
                    new SwerveModule(
                        new DriveMotor(Constants.POD_3_DRIVE, 1),
                        new TurnMotor(Constants.POD_3_TURN, 1),
                        Constants.DRIVE_METERS_PER_ROTATION
                    ),
                    new SwerveModule(
                        new DriveMotor(Constants.POD_4_DRIVE, 1),
                        new TurnMotor(Constants.POD_4_TURN, 1),
                        Constants.DRIVE_METERS_PER_ROTATION
                    ),
                },
                new Translation2d[] {
                    // TODO Add module offsets
                }, gyro);
    }

    /**
     * This method must be called before the Chassis is used
     */
    public void init() {
        gyro.calibrate();
        drive.init();
        initialized = true;
    }

    /**
     * This method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        if (initialized) {
            drive.periodic();
        }
    }
}
