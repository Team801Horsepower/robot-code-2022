/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.ArmReset;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private SparkMaxPIDController armPID;
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;

  private boolean armZeroedFlag;

  private SparkMaxLimitSwitch m_reverseLimit;


  /**
   * Creates a new LifterWinch.
   */
  public Arm() {
    // Arm Settings
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    armMotor.setInverted(Constants.ARM_INVERT);
    armMotor.setIdleMode(Constants.ARM_IDLEMODE);
    armMotor.setSmartCurrentLimit(Constants.ARM_MAX_CURRENT_STALL, Constants.ARM_MAX_CURRENT_RUN);

    armPID = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();
    armPID.setP(Constants.ARM_P);
    armPID.setI(Constants.ARM_I);
    armPID.setD(Constants.ARM_D);
    armPID.setIZone(Constants.ARM_IZ);
    armPID.setFF(Constants.ARM_FF);
    armPID.setOutputRange(Constants.ARM_MIN_OUTPUT, Constants.ARM_MAX_OUTPUT);

    // for the Neo 550 motor built in encoder we need to do the external gear reductions math in the
    // setPositionConversionFactor
    // 10 to 1 for the gearbox on the motor.
    armEncoder.setPositionConversionFactor(1); // encoder will now return lead screw rotations
    armZeroedFlag = false;

    // limit switch is zero point (fully retracted)
    m_reverseLimit = armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_reverseLimit.enableLimitSwitch(true);
  }


  public void setArmHeight(double rotations) // in lead screw rotations...
  {
    if (armZeroedFlag) {
      // Lifter Motor must rotate 400 times to go 4 inches on lead screw, and 40 inches in height.
      armPID.setReference(rotations, ControlType.kPosition);
    } else {
      new ArmReset().schedule();
    }
  }


  // sets arm to fully retracted and zeros the encoder
  public void resetArm() {
    armZeroedFlag = false;
    // run till the limit switch stops it... using 500 because that should be more than the
    // lead screw length.
    armPID.setReference(Constants.ARM_POSITION_RESET, ControlType.kPosition);
  }

  public void armResetting() {
    // This method will be called once per scheduler run
    // test if at the fully retracted position and reset the encoder to zero
    // flag keeps from banging the encoder every scheduler run.
    if (m_reverseLimit.isPressed() && !armZeroedFlag) {
      armEncoder.setPosition(0);
      armZeroedFlag = true;
      armPID.setReference(0, ControlType.kPosition);
    }
  }


  @Override
  public void periodic() {}


  public double getHeight() {
    return armEncoder.getPosition();
  }


  public boolean getArmZeroedFlag() {
    return this.armZeroedFlag;
  }

}
