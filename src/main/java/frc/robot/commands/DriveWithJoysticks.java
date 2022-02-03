/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveWithJoysticks extends CommandBase {
    /**
     * Creates a new DriveWithJoysticks.
     */
    public DriveWithJoysticks() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = RobotContainer.io.getDriverExpoLeftX(2.5);
        double y = RobotContainer.io.getDriverExpoLeftY(2.5);
        if (RobotContainer.winch.safeToDrive()) {
            RobotContainer.chassis.drive(
                -y/4,
                -x/4,
                RobotContainer.io.getDriverExpoRightX(2.5)); // TODO changed sign of X right may
                                                                 // need to be done elsewhere
        } else {
            RobotContainer.chassis.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
