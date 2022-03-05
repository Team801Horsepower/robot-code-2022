// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.Utils;

public class DriveToPose extends CommandBase {

  protected Pose2d targetPose;

  /** Creates a new DriveToPoint. */
  public DriveToPose(Pose2d targetPose) {
    addRequirements(RobotContainer.chassis);
    this.targetPose = targetPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d error = targetPose.minus(RobotContainer.chassis.getCurrentPose());

    RobotContainer.chassis.fieldDrive(error.getX(),
        error.getY(),
        -error.getRotation().getRadians());
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return Utils.almostEqual(RobotContainer.chassis.getCurrentPose(), targetPose, 0.1);
  }
}
