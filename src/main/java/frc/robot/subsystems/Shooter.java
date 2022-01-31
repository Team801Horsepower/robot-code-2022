/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
  private CANPIDController shooterPID;
  private CANPIDController breachPID;
  private CANSparkMax shooterMotor;
  private CANSparkMax breachMotor;
  private CANEncoder shooterEncoder;


  /**
   * Creates a new Shooter.
   */
  public Shooter() 
  {
    shooterMotor = new CANSparkMax(Constants.shooterMotorID, MotorType.kBrushless);
    shooterPID = shooterMotor.getPIDController();
    shooterEncoder = shooterMotor.getEncoder(EncoderType.kHallSensor, 42);
    shooterPID.setP(Constants.SHOOTER_P);
    shooterPID.setI(Constants.SHOOTER_I);
    shooterPID.setD(Constants.SHOOTER_D);
    shooterPID.setIZone(Constants.SHOOTER_IZ);
    shooterPID.setFF(Constants.SHOOTER_FF);
    shooterMotor.setInverted(Constants.SHOOTER_INVERTED);
    shooterPID.setOutputRange(Constants.SHOOTER_OUTPUT_MIN, Constants.SHOOTER_OUTPUT_MAX);

    shooterMotor.setSmartCurrentLimit(Constants.SHOOTER_MAX_CURRENT_STALL, Constants.SHOOTER_MAX_CURRENT_RUN);

    shooterEncoder.setVelocityConversionFactor(1);

    breachMotor = new CANSparkMax(Constants.breachMotorID, MotorType.kBrushless);
    breachPID = breachMotor.getPIDController();
    breachMotor.setInverted(Constants.BREACH_INVERTED);
  }

  //TODO: find correct speed
  public void enableShooter()
  {
    shooterPID.setReference(Constants.SHOOTER_RPM, ControlType.kVelocity);
    holdDown();
  }


  public void popUp()
  {
    breachPID.setReference(Constants.BREACH_UPSPEED, ControlType.kDutyCycle);
  }


  public void holdDown()
  {
    breachPID.setReference(Constants.BREACH_DOWNSPEED, ControlType.kDutyCycle);
  }


  public void stop()
  {
    shooterPID.setReference(0, ControlType.kDutyCycle);
    breachPID.setReference(0, ControlType.kDutyCycle);
  }


  public boolean isReady()
  {
    // System.out.printf("RPM:  %f\n", shooterEncoder.getVelocity());
    return Math.abs(shooterEncoder.getVelocity() - Constants.SHOOTER_RPM) < Constants.SHOOTER_RPM_WINDOW;
  }
}
