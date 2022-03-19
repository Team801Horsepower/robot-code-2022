package frc.robot.subsystems;

import java.util.Collections;
import java.util.Map;
import java.util.TreeMap;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.components.Neo;
import frc.robot.utilities.InterpolatedLookupTable;

public class Shooter extends SubsystemBase {

    public final Neo FLYWHEEL;

    private double targetedRange = MIN_RANGE;
    private Double wheelSpeed = RANGE_TO_VELOCITY.get(targetedRange);
    
    public Shooter() {
        FLYWHEEL = new Neo(Constants.SHOOTER);
        FLYWHEEL.setGearRatio(SHOOTER_GEAR_RATIO);

        int speedPid = FLYWHEEL.getSpeedPid();
        FLYWHEEL.PID.setP(0.010, speedPid);
        FLYWHEEL.PID.setI(0.00001, speedPid);
        FLYWHEEL.PID.setD(0.0, speedPid);
        FLYWHEEL.PID.setFF(0.0009, speedPid);
        FLYWHEEL.PID.setIZone(50.0, speedPid);

        FLYWHEEL.CONTROLLER.setIdleMode(IdleMode.kBrake);

        int positionPid = FLYWHEEL.getPositionPid();
        FLYWHEEL.PID.setP(0.5, positionPid);
        FLYWHEEL.PID.setI(0.0, positionPid);
        FLYWHEEL.PID.setD(0.0, positionPid);

        // SmartDashboard.putData("FLYWHEEL", FLYWHEEL);
    }

    /**
     * Revs the flywheel to shoot at the targeted range.
     */
    public void start() {
        FLYWHEEL.setDesiredSpeed(wheelSpeed);
    }

    public void periodic() {

    }

    /**
     * @param range the range to shoot in meters
     */
    public void setRange(double range) {
        targetedRange = range;
        wheelSpeed = RANGE_TO_VELOCITY.get(range);
        if (wheelSpeed == null) {
            wheelSpeed = 0.0;
        }
    }

    public void setSpeed(double speed) {
        FLYWHEEL.setDesiredSpeed(speed);
    }

    public boolean ready() {
        return FLYWHEEL.velocityReached(VELOCITY_TOLERANCE);
    }

    public void stop() {
        FLYWHEEL.setPower(0.0);
    }

    /**
     * Returns a {@link Command} which clears the shooter of any balls.
     * @return
     */
    public Command prepare() {
        return FLYWHEEL.generateRotationCommand(FREE_BALL_ROTATION, POSITION_TOLERANCE, this);
    }

    /**
     * Returns a {@link Command} which revs the shooter and returns when it's ready.
     */
    public Command revShooter() {
        return FLYWHEEL.generateVelocityCommand(() -> wheelSpeed, VELOCITY_TOLERANCE, this);
    }

    private static final double FREE_BALL_ROTATION = Units.degreesToRadians(-270.0);
    
    private static final double VELOCITY_TOLERANCE = Units.rotationsPerMinuteToRadiansPerSecond(30.0);
    private static final double POSITION_TOLERANCE = Units.degreesToRadians(10.0);
    
    private static final InterpolatedLookupTable<Double, Double> RANGE_TO_VELOCITY = new InterpolatedLookupTable<>(
        Collections.unmodifiableNavigableMap(new TreeMap<Double, Double>(Map.of(
            2.4, 200.0,
            2.7, 205.0,
            3.2, 230.0,
            3.7, 260.0,
            4.2, 287.0,
            4.7, 310.0,
            5.2, 375.0
        ))),
        (t, x1, y1, x2, y2) -> {return (t - x1)/(x2 - x1) * (y2 - y1) + y1;}
    );
    
    public static final double MAX_RANGE = RANGE_TO_VELOCITY.firstKey();
    public static final double MIN_RANGE = RANGE_TO_VELOCITY.lastKey();
    
    public static final double SHOOTER_GEAR_RATIO = 1.0;
}
