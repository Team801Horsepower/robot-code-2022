package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.components.Neo;

public class Feeder extends SubsystemBase {
    private final Neo FEEDER;
    private final ColorSensorV3 COLOR_SENSOR;

    private final double FEEDER_P = 1.0;
    private final double FEEDER_I = 0.0;
    private final double FEEDER_D = 0.0;
    private final double FEEDER_FF = 0.0;

    private final double POSITION_TOLERANCE = 0.01;
    private final double FEEDER_GEAR_RATIO = 14.2857143;

    private boolean loaded = false;

    public Feeder() {
        FEEDER = new Neo(Constants.FEEDER);
        FEEDER.setGearRatio(FEEDER_GEAR_RATIO);

        int positionPid = FEEDER.getPositionPid();
        FEEDER.PID.setP(FEEDER_P, positionPid);
        FEEDER.PID.setI(FEEDER_I, positionPid);
        FEEDER.PID.setD(FEEDER_D, positionPid);
        FEEDER.PID.setFF(FEEDER_FF, positionPid);

        FEEDER.CONTROLLER.setIdleMode(IdleMode.kBrake);

        FEEDER.setPosition(0.0);

        COLOR_SENSOR = new ColorSensorV3(edu.wpi.first.wpilibj.I2C.Port.kOnboard);
    }

    public void periodic() {
        loaded = COLOR_SENSOR.getProximity() > Constants.FEEDER_THRESHOLD;
        SmartDashboard.putBoolean("Feeder Loaded", loaded);
    }

    public boolean isLoaded() {
        return loaded;
    }

    /**
     * Run positive values to feed. Negative values to spit. 
     * 
     * @param power [-1.0, 1.0]
     */

    public void run(double power) {
        FEEDER.setPower(power);
    }

    public void stop() {
        FEEDER.setPower(0.0);
    }

    public Command feed() {
        return FEEDER.generateRotationCommand(Constants.FEEDER_1_BALL, POSITION_TOLERANCE, this);
    }

    public void reset() {
        FEEDER.setPosition(0.0);
        FEEDER.setDesiredPosition(0.0);
    }
}
