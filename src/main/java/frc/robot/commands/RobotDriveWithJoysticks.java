/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.RobotContainer;

public class RobotDriveWithJoysticks extends CommandBase {
    /**
     * Creates a new RobotDriveWithJoysticks.
     */
    public RobotDriveWithJoysticks() {
        addRequirements(RobotContainer.CHASSIS);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        double forward = IO.Joystick.DriverLeft.getForward();
        double leftward = IO.Joystick.DriverLeft.getLeftward();
        double angular = IO.Joystick.DriverRight.getLeftward();
        
        forward *= Constants.MAX_ROBOT_DRIVE_SPEED;
        leftward *= Constants.MAX_ROBOT_DRIVE_SPEED;
        angular *= Constants.MAX_ROBOT_TURN_SPEED;

        RobotContainer.CHASSIS.robotDrive(forward, leftward, angular);
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
