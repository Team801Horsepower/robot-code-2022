/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ColorWheelDown extends CommandBase {
  /**
   * Creates a new ColorWheelDown.
   */
  public ColorWheelDown() {
    addRequirements(RobotContainer.COLOR_WHEEL);
  }

  @Override
  public void initialize() {
    RobotContainer.COLOR_WHEEL.resetSpinner();
  }

  @Override
  public void execute() {
    RobotContainer.COLOR_WHEEL.spinnerResetting();
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.COLOR_WHEEL.getSpinnerZeroedFlag();
  }
}
