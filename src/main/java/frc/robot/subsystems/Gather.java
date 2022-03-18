package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.Neo550;

public class Gather extends SubsystemBase {

    public final Neo550 WHEELS;
    public final Neo550 ARM;

    private boolean lowering = false;

    public Gather() {
        ARM = new Neo550(Constants.GATHER_ARM);
        ARM.setGearRatio(Constants.GATHER_ARM_GEAR_RATIO);

        int positionPid = ARM.getPositionPid();
        ARM.PID.setP(1.0, positionPid);
        ARM.PID.setI(0.0, positionPid);
        ARM.PID.setD(0.0, positionPid);

        WHEELS = new Neo550(Constants.GATHER_WHEELS);
        WHEELS.setGearRatio(Constants.GATHER_WHEELS_GEAR_RATIO);

        positionPid = WHEELS.getPositionPid();
        WHEELS.PID.setP(1.0, positionPid);
        WHEELS.PID.setI(0.001, positionPid);
        WHEELS.PID.setD(0.0, positionPid);
        WHEELS.PID.setFF(0.01, positionPid);

        SmartDashboard.putData(WHEELS);
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
        return WHEELS.generatePositionCommand(() -> WHEELS.getCurrentPosition() -1.9 * Math.PI, 0.01, this);
    }

    public Command fireBall() {
        return WHEELS.generatePositionCommand(() -> WHEELS.getCurrentPosition() + 1.75 * Math.PI, 0.01, this);
    }

    public boolean isRaised() {
        return ARM.getCurrentPosition() > Constants.ARM_RAISED_POSITION;
    }

    public boolean isLowered() {
        return ARM.getCurrentPosition() < Constants.ARM_LOWERED_POSITION;
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
}
