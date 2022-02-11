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
    addRequirements(RobotContainer.colorWheel);
  }

  @Override
  public void initialize() {
    RobotContainer.colorWheel.resetSpinner();
  }

  @Override
  public void execute() {
    RobotContainer.colorWheel.spinnerResetting();
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.colorWheel.getSpinnerZeroedFlag();
  }
}
