package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.architecture.PositionMotor;
import frc.robot.architecture.SpeedMotor;
import frc.robot.utilities.Utils;

public class Neo implements SpeedMotor, PositionMotor, Sendable {

    public static final String PART_NAME = "REV-21-1650";

    public static final double MAX_SPEED = 5650.0; // The max speed in native units (RPM)
    public static final double STALL_CURRENT = 105; // The stall current (Amps)
    public static final double STALL_TORQUE = 2.5; // The stall torque (Nm)

    public final int CAN_ID;
    public final CANSparkMax CONTROLLER;
    public final SparkMaxPIDController PID;

    /** Exposed for advanced controls, for most applications do not mess with this. */
    private final RelativeEncoder ENCODER;
    
    private double gearRatio = 1.0;
    private int speedPid = 0, positionPid = 1;

    private double desiredSpeed = 0.0;
    private double desiredPosition = 0.0;

    /**
     * Creates an instance of Neo which refers to a Neo or a Neo550 attached to a SparkMax.
     * Update the PID and motor configuration before use.
     * @param canId
     */
    public Neo(int canId) {
        CAN_ID = canId;
        CONTROLLER = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        ENCODER = CONTROLLER.getEncoder();
        ENCODER.setPositionConversionFactor(2 * Math.PI);
        ENCODER.setVelocityConversionFactor(2 * Math.PI / 60.0);
        PID = CONTROLLER.getPIDController();

        CONTROLLER.setSmartCurrentLimit(40);
        SmartDashboard.putData(this);
    }

    @Override
    public void init() {}

    @Override
    public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SparkMax");
        builder.addBooleanProperty("inverted", () -> { return CONTROLLER.getInverted(); }, (value) -> { CONTROLLER.setInverted(value); });
        builder.addBooleanProperty("brake", () -> { return CONTROLLER.getIdleMode() == IdleMode.kBrake; }, (value) -> { CONTROLLER.setIdleMode(value ? IdleMode.kBrake : IdleMode.kCoast); });

        for (int i = 0; i < 4; i++) {
            final int t = i;
            builder.addDoubleProperty("p_" + t, () -> { return PID.getP(t); }, (value) -> { PID.setP(value, t); });
            builder.addDoubleProperty("i_" + t, () -> { return PID.getI(t); }, (value) -> { PID.setI(value, t); });
            builder.addDoubleProperty("d_" + t, () -> { return PID.getD(t); }, (value) -> { PID.setD(value, t); });
            builder.addDoubleProperty("ff_" + t, () -> { return PID.getFF(t); }, (value) -> { PID.setFF(value, t); });
            builder.addDoubleProperty("iz_" + t, () -> { return PID.getIZone(t); }, (value) -> { PID.setIZone(value, t); });
        }
        
        builder.addDoubleProperty("position", this::getCurrentPosition, (value) -> {});
        builder.addDoubleProperty("speed", this::getCurrentSpeed, (value) -> {});


    }
    
    /**
     * Duty Cycle control 
     * 
     * @param power approximate speed [-1.0, 1.0]
     */
    public void setPower(double power) {
        PID.setReference(power, ControlType.kDutyCycle);
    } 

    public void setDesiredSpeed(double speed) {
        double maxSpeed = getMaxSpeed();
        if (speed > maxSpeed) {
            System.err.println("Tried to exceed max speed: " + speed + "rad/s (max is "
                    + maxSpeed + "rad/s)");
            speed = maxSpeed;
        }
        desiredSpeed = speed;
        PID.setReference(speed, ControlType.kVelocity, speedPid);
    }

    @Override
    public double getCurrentSpeed() {
        return ENCODER.getVelocity();
    }

    @Override
    public void setDesiredPosition(double position) {
        desiredPosition = position;
        PID.setReference(position, ControlType.kPosition, positionPid);
    }

    @Override
    public double getCurrentPosition() {
        return ENCODER.getPosition();
    }

    public void setPositionPid(int pidSlot) {
        positionPid = pidSlot;
    }

    public int getPositionPid() {
        return positionPid;
    }

    public void setSpeedPid(int pidSlot) {
        speedPid = pidSlot;
    }

    public int getSpeedPid() {
        return speedPid;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    /**
     * Sets the ratio from the motor shaft to the output shaft.
     * @param gearRatio the ratio output_shaft_rotations / motor_shaft_rotations
     */
    public void setGearRatio(double gearRatio) {
        ENCODER.setPositionConversionFactor(ENCODER.getPositionConversionFactor() / this.gearRatio * gearRatio);
        ENCODER.setVelocityConversionFactor(ENCODER.getVelocityConversionFactor() / this.gearRatio * gearRatio);
        this.gearRatio = gearRatio;
    }

    public void setPosition(double newPosition) {
        PID.setReference(newPosition, ControlType.kPosition, positionPid);
        ENCODER.setPosition(newPosition);
    }
    
    protected double getMaxSpeed(double nativeMaxSpeed) {
        return nativeMaxSpeed * ENCODER.getVelocityConversionFactor();
    }

    public double getMaxSpeed() {
        return getMaxSpeed(MAX_SPEED);
    }

    public boolean positionReached(double tolerance) {
        return Utils.almostEqual(desiredPosition, getCurrentPosition(), tolerance);
    }

    public boolean velocityReached(double tolerance) {
        return Utils.almostEqual(desiredSpeed, getCurrentSpeed(), tolerance);
    }

    public Command generatePositionCommand(double position, double tolerance, Subsystem... requirements) {
        CommandBase command = new CommandBase() {

            @Override
            public void initialize() {
                setDesiredPosition(position);
            }

            @Override
            public void execute() {
                System.out.println("Setting Position " + position);
            }

            @Override
            public boolean isFinished() {
                return positionReached(tolerance);
            }

            @Override
            public void end(boolean interrupted) {
                if (!interrupted)
                    System.out.println("Position Reached");
            }
        };
        command.addRequirements(requirements);
        return command;
    }

    public Command generateRotationCommand(double rotation, double tolerance, Subsystem... requirements) {
        CommandBase command = new CommandBase() {

            @Override
            public void initialize() {
                setDesiredPosition(getCurrentPosition() + rotation);
            }

            @Override
            public void execute() {
                System.out.println("Setting Rotation" + rotation);
            }

            @Override
            public boolean isFinished() {
                return positionReached(tolerance);
            }

            @Override
            public void end(boolean interrupted) {
                if (!interrupted)
                    System.out.println("Rotation Reached");
            }
        };
        command.addRequirements(requirements);
        return command;
    }

    public Command generateVelocityCommand(double velocity, double tolerance, Subsystem... requirements) {
        CommandBase command = new CommandBase() {

            @Override
            public void initialize() {
                setDesiredSpeed(velocity);
            }

            @Override
            public void execute() {
                System.out.println("Setting Velocity: " + velocity);
            }

            @Override
            public boolean isFinished() {
                return velocityReached(tolerance);
            }

            @Override
            public void end(boolean interrupted) {
                if (!interrupted)
                    System.out.println("Velocity Reached");
            }
        };
        command.addRequirements(requirements);
        return command;
    }
}
