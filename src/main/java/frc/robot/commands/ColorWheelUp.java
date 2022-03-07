/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ColorWheelUp extends CommandBase {
  /**
   * Creates a new ColorWheelUp.
   */
  public ColorWheelUp() {
    addRequirements(RobotContainer.COLOR_WHEEL);
  }

  @Override
  public void initialize() {
    RobotContainer.COLOR_WHEEL.rotateColorWheel(Constants.COLORWHEEL_ROTATION_COUNT);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
