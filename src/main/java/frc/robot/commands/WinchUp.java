/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class WinchUp extends CommandBase {
    /**
     * .
     */
    public WinchUp() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.winch);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // ONLY move in the positive direction regardless of the stick direction.
        RobotContainer.winch.winchUp(Math.abs(RobotContainer.io.getManipulatorLeftY()));
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // stop the motor if the command is interrupted.
        RobotContainer.winch.winchUp(0);
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
