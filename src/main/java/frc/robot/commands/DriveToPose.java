// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveToPose extends CommandBase {

  Pose2d targetPose;

  /** Creates a new DriveToPoint. */
  public DriveToPose(Pose2d targetPose) {
    addRequirements(RobotContainer.chassis);
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d error = RobotContainer.chassis.getCurrentPose();

    RobotContainer.chassis.fieldDrive(targetPose.getX()-error.getX(), targetPose.getY()-error.getY(), error.getRotation().getRadians()-targetPose.getRotation().getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
