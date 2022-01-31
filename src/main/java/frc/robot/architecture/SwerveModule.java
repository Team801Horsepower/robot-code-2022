package frc.robot.architecture;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** A class which implements the boilerplate code for running a typical swerve module. */
public class SwerveModule {

    private SpeedMotor driveMotor;
    private AngleMotor turnMotor;

    private double lastAngle;
    private boolean flipFlag;

    protected double metersPerRad;

    private SwerveModuleState state;

    /**
     * Creates a new `SwerveModule` instance
     * 
     * @param driveMotor A motor which implements the `SpeedMotor` interface.
     * @param turnMotor A motor which implements the `TurnMotor` interface.
     * @param metersPerRad A conversion constant with the units meters/rad which converts from
     *        radians to meters. (Depends on gear ratios, wheel size, etc.)
     */
    public SwerveModule(SpeedMotor driveMotor, AngleMotor turnMotor, double metersPerRad) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.metersPerRad = metersPerRad;
        this.state = new SwerveModuleState();
    }

    /** Initializes both motors. */
    public void init() {
        driveMotor.init();
        turnMotor.init();
    }

    /** Calls `.periodic()` on both motors. */
    public void periodic() {
        driveMotor.periodic();
        turnMotor.periodic();
    }

    /**
     * Convenience function to set Speed and Angle at the same time.
     * 
     * @param state A `SwerveModuleState` describing the desired state.
     */
    public void setDesiredState(SwerveModuleState state) {
        setDesiredAngle(state.angle.getRadians());
        setDesiredSpeed(state.speedMetersPerSecond);
    }

    /**
     * Requests the module to drive at the desired speed.
     * 
     * @param speed The speed to rotate in m/s.
     */
    public void setDesiredSpeed(double speed) {
        if (flipFlag) {
            speed = -speed;
        }
        driveMotor.setDesiredSpeed(speed / metersPerRad);
    }

    /**
     * Requests the module face the desired angle.
     * 
     * @param angle The angle to face in radians.
     */
    public void setDesiredAngle(double angle) {
        angle %= 2 * Math.PI;
        double errorAngle = Math.abs(angle - lastAngle);
        lastAngle = angle;

        if (errorAngle > Math.PI / 2 && errorAngle < 3 * Math.PI / 2) {
            flipFlag = !flipFlag;
        }

        if (flipFlag) {
            angle = (angle + Math.PI) % (2 * Math.PI);
        }

        turnMotor.setDesiredAngle(angle);
    }

    /**
     * Returns the current Speed and Angle in a `SwerveModuleState`.
     */
    public SwerveModuleState getCurrentState() {
        state.angle = new Rotation2d(turnMotor.getCurrentAngle());
        state.speedMetersPerSecond = driveMotor.getCurrentSpeed() * metersPerRad;
        return state;
    }
}
