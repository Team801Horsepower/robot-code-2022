package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo;
import frc.robot.components.Neo550;

public class Gatherer extends SubsystemBase {

    public final Neo WHEELS;
    public final Neo550 ARM;

    private final double ARM_P = 1.0;
    private final double ARM_I = 0.0;
    private final double ARM_D = 0.0;

    public static final double GATHER_WHEELS_GEAR_RATIO = 9.0;
    public static final double GATHER_ARM_GEAR_RATIO = 30.0;

    private boolean lowering = false;

    public Gatherer() {
        ARM = new Neo550(Constants.GATHER_ARM);
        ARM.setGearRatio(GATHER_ARM_GEAR_RATIO);

        int positionPid = ARM.getPositionPid();
        ARM.PID.setP(ARM_P, positionPid);
        ARM.PID.setI(ARM_I, positionPid);
        ARM.PID.setD(ARM_D, positionPid);
        // ARM.CONTROLLER.setSmartCurrentLimit(5, 20);

        WHEELS = new Neo(Constants.GATHER_WHEELS);
        WHEELS.setGearRatio(GATHER_WHEELS_GEAR_RATIO);
        // WHEELS.CONTROLLER.setSmartCurrentLimit(3, 20);

        WHEELS.CONTROLLER.setIdleMode(IdleMode.kBrake);

        // SmartDashboard.putData("WHEELS", WHEELS);
    }

    public void lower() {
        ARM.setPower(-0.75);
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
        WHEELS.setPower(0.4 * power);
    }

    public void stop() {
        WHEELS.setPower(0.0);
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

    public boolean isJammed() {
        return WHEELS.CONTROLLER.getOutputCurrent() > 40.0;
    }
}
