package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo550;

public class Gather extends SubsystemBase {

    public final Neo550 WHEELS;
    public final Neo550 ARM;

    private final double ARM_P = 1.0;
    private final double ARM_I = 0.0;
    private final double ARM_D = 0.0;

    private final double WHEELS_P = 2.0;
    private final double WHEELS_I = 0.005;
    private final double WHEELS_D = 0.0;
    private final double WHEELS_FF = 0.0;
    private final double WHEELS_IZ = 0.5;

    private final double POSITION_TOLERANCE = 0.01;

    private boolean lowering = false;

    public Gather() {
        ARM = new Neo550(Constants.GATHER_ARM);
        ARM.setGearRatio(GATHER_ARM_GEAR_RATIO);

        int positionPid = ARM.getPositionPid();
        ARM.PID.setP(ARM_P, positionPid);
        ARM.PID.setI(ARM_I, positionPid);
        ARM.PID.setD(ARM_D, positionPid);

        WHEELS = new Neo550(Constants.GATHER_WHEELS);
        WHEELS.setGearRatio(GATHER_WHEELS_GEAR_RATIO);

        positionPid = WHEELS.getPositionPid();
        WHEELS.PID.setP(WHEELS_P, positionPid);
        WHEELS.PID.setI(WHEELS_I, positionPid);
        WHEELS.PID.setD(WHEELS_D, positionPid);
        WHEELS.PID.setFF(WHEELS_FF, positionPid);
        WHEELS.PID.setIZone(WHEELS_IZ);

        // SmartDashboard.putData("WHEELS", WHEELS);
    }

    public void lower() {
        ARM.setPower(-0.2);
        lowering = true;
    }

    public void raise() {
        ARM.setPower(0.2);
        lowering = false;
    }

    /**
     * Run positive values to gather. Negative values to spit. 
     * 
     * @param power [-1.0, 1.0]
     */
    public void run(double power) {
        WHEELS.setPower(power);
    }

    public void stop() {
        WHEELS.setPower(0.0);
    }

    public Command tampBall() {
        return WHEELS.generateRotationCommand(Constants.GATHER_TAMP_ROTATION, POSITION_TOLERANCE, this);
    }

    public Command fireBall() {
        return WHEELS.generateRotationCommand(Constants.GATHER_FIRE_ROTATION, POSITION_TOLERANCE, this);
    }

    public boolean isRaised() {
        return ARM.getCurrentPosition() > Constants.ARM_RAISED_POSITION;
    }

    public boolean isLowered() {
        return ARM.getCurrentPosition() < Constants.ARM_LOWERED_POSITION;
    }

    public void clear() {
        WHEELS.setPosition(0.0);
        WHEELS.setDesiredPosition(0.0);
    }

    public void reset() {
        clear();
        ARM.setPosition(0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (lowering && isLowered()) {
            ARM.setPower(0.0);
        }
        if (!lowering && isRaised()) {
            ARM.setPower(0.0);
        }
    }

    public static final double GATHER_WHEELS_GEAR_RATIO = 9.0;
    public static final double GATHER_ARM_GEAR_RATIO = 30.0;
}
