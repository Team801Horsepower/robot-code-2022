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

public class Winch extends SubsystemBase 
{
  private CANPIDController winchPID;
  private CANSparkMax winchMotor;
  private CANEncoder winchEncoder;

  /**
   * Creates a new LifterWinch.
   */
  public Winch() 
  {
    // Winch Settings
    winchMotor = new CANSparkMax(Constants.winchMotorID, MotorType.kBrushless);
    winchPID = winchMotor.getPIDController();
    winchEncoder = winchMotor.getEncoder(EncoderType.kHallSensor, Constants.NEO_ENCODER_CNTS_PER_REV);
    winchEncoder.setPositionConversionFactor(1);

    winchPID.setP(Constants.WINCH_P);
    winchPID.setI(Constants.WINCH_I);
    winchPID.setD(Constants.WINCH_D);
    winchPID.setIZone(Constants.WINCH_IZ);
    winchPID.setFF(Constants.WINCH_FF);
    winchPID.setOutputRange(Constants.WINCH_MIN_OUTPUT, Constants.WINCH_MAX_OUTPUT);
    
    winchMotor.setSmartCurrentLimit(Constants.WINCH_MAX_CURRENT_STALL, Constants.WINCH_MAX_CURRENT_RUN);
  }


  public void winchUp(double y) // in lead screw rotations...
  {
    //Manual control of the winch.
    winchPID.setReference(y, ControlType.kDutyCycle);

  }


  public boolean safeToDrive()
  {
      return (winchEncoder.getPosition() < Constants.WINCH_SAFE_TO_DRIVE);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run

  }
}
