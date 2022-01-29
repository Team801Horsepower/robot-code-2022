package frc.robot.components;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class DriveMotor implements SpeedMotor {
    private double currentSpeed = 0.0;
    private double desiredSpeed = 0.0;

    private SparkMaxPIDController sparkPID;
    private CANSparkMax sparkMotor;
    private RelativeEncoder sparkEncoder;

    /**
     * 
     * @param motorID of the Spark Max
     * @param motorIndex of the motor index
     */
    public DriveMotor(int motorID, int motorIndex) {
        sparkMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        sparkPID = sparkMotor.getPIDController();
        sparkEncoder = sparkMotor.getEncoder();

        sparkPID.setP(Constants.DRIVE_P);
        sparkPID.setI(Constants.DRIVE_I);
        sparkPID.setD(Constants.DRIVE_D);
        sparkPID.setIZone(Constants.DRIVE_IZ);
        sparkPID.setFF(Constants.DRIVE_FF);
        sparkPID.setOutputRange(Constants.DRIVE_MIN_OUTPUT, Constants.DRIVE_MAX_OUTPUT);

        sparkMotor.setInverted(Constants.DRIVE_INVERT[motorIndex]);
        sparkMotor.setIdleMode(Constants.DRIVE_IDLEMODE[motorIndex]);
        sparkMotor.setSmartCurrentLimit(Constants.DRIVE_MAX_CURRENT_STALL,
                Constants.DRIVE_MAX_CURRENT_RUN);
    }

    /**
     * 
     * @param speed desired speed of the motor shaft in radians/sec
     */
    public void setDesiredSpeed(double speed) {
        desiredSpeed = speed;
        sparkPID.setReference(desiredSpeed, ControlType.kDutyCycle);
    }

    /**
     * 
     * @return motor shaft velocity in radians/sec
     */
    public double getCurrentSpeed() {
        currentSpeed = sparkEncoder.getVelocity();
        return currentSpeed;
    }
}
