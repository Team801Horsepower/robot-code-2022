/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorWheel extends SubsystemBase 
{
  private SparkMaxPIDController spinnerPID;
  private CANSparkMax spinnerMotor;;
  private RelativeEncoder spinnerEncoder;

  
  private boolean spinnerZeroedFlag;

  private SparkMaxLimitSwitch m_reverseLimit;

  /**
   * Creates a new color wheel spinner gizmo subsystem.
   */
  public ColorWheel() 
  {
    spinnerMotor = new CANSparkMax(Constants.colorWheelMotorID, MotorType.kBrushless);
    spinnerPID = spinnerMotor.getPIDController();
    spinnerEncoder = spinnerMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    spinnerPID.setP(Constants.COLORWHEEL_P);
    spinnerPID.setI(Constants.COLORWHEEL_I);
    spinnerPID.setD(Constants.COLORWHEEL_D);
    spinnerPID.setIZone(Constants.COLORWHEEL_IZ);
    spinnerPID.setFF(Constants.COLORWHEEL_FF);
    spinnerPID.setOutputRange(Constants.COLORWHEEL_MIN_OUTPUT, Constants.COLORWHEEL_MAX_OUTPUT_RAISE);

    spinnerMotor.setSmartCurrentLimit(Constants.COLORWHEEL_MAX_CURRENT_STALL, Constants.COLORWHEEL_MAX_CURRENT_RUN);

  
    // for the Neo 550 motor built in encoder we need to do the external gear reductions math in the setPositionConversionFactor
    // 8 to 1 for the drive wheel to color wheel and 10 to 1 for the for the gearbox on the motor. 80 motor rotations = 1 color wheel rotation
    spinnerEncoder.setPositionConversionFactor(1);  // encoder will return rotations
    //575 motor shaft rotations to lift/bring down

    spinnerZeroedFlag = false;

    // limit switch is zero point (fully retracted)
    m_reverseLimit = spinnerMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_reverseLimit.enableLimitSwitch(true); 
  }

  public void rotateColorWheel(double rotations)
  {
    if(spinnerEncoder.getPosition() < Constants.SPINNER_MAX_HEIGHT - 2 )  // The -2 accounts for any small error in the encoder value
    {
      spinnerPID.setOutputRange(Constants.COLORWHEEL_MIN_OUTPUT, Constants.COLORWHEEL_MAX_OUTPUT_RAISE);
      spinnerPID.setReference( Constants.SPINNER_MAX_HEIGHT, CANSparkMax.ControlType.kPosition);
    }
    else
    {
      spinnerPID.setOutputRange(Constants.COLORWHEEL_MIN_OUTPUT, Constants.COLORWHEEL_MAX_OUTPUT_SPIN);
      //convert color wheel rotations to motorshaft rotations 80 shatf rotations = 1 color wheel rotation
      spinnerPID.setReference(spinnerEncoder.getPosition() + rotations * 80, CANSparkMax.ControlType.kPosition);
    }
  }


  // sets arm to fully retracted and zeros the encoder
  public void resetSpinner()
  {
    spinnerZeroedFlag = false;
    // run till the limit switch stops it... 
    spinnerPID.setReference(-0.4, CANSparkMax.ControlType.kDutyCycle);  
  }

  public void spinnerResetting() 
  {
    // This method will be called once per scheduler run
    // test if at the fully retracted position and reset the encoder to zero
    // flag keeps from banging the encoder every scheduler run.
    if(m_reverseLimit.isPressed() && !spinnerZeroedFlag)
    {
      spinnerEncoder.setPosition(0);
      spinnerZeroedFlag = true;
      spinnerPID.setReference(0, CANSparkMax.ControlType.kPosition);
    }
  }

  public double getHeight()
  {
      return spinnerEncoder.getPosition();
  }

  public boolean getSpinnerZeroedFlag()
  {
      return this.spinnerZeroedFlag;
  }
    
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
