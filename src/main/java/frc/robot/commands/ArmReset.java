/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmReset extends CommandBase {
  /**
   * Command to lower the arm to the home position and reset the encoder.
   */
  public ArmReset() {
    addRequirements(RobotContainer.ARM);
  }

  @Override
  public void initialize() {
    RobotContainer.ARM.resetArm();
  }

  @Override
  public void execute() {
    RobotContainer.ARM.armResetting();
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.ARM.getArmZeroedFlag();
  }
}
