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

public class FieldDriveWithJoysticks extends CommandBase {
    /**
     * Creates a new FieldDriveWithJoysticks.
     */
    public FieldDriveWithJoysticks() {
        addRequirements(RobotContainer.CHASSIS);
    }

    @Override
    public void execute() {
        double forward = IO.Joystick.DriverLeft.getForward();
        double leftward = IO.Joystick.DriverLeft.getLeftward();
        double angular = IO.Joystick.DriverRight.getLeftward();
        
        forward *= Constants.MAX_ROBOT_DRIVE_SPEED;
        leftward *= Constants.MAX_ROBOT_DRIVE_SPEED;
        angular *= Constants.MAX_ROBOT_TURN_SPEED;

        if (RobotContainer.WINCH.safeToDrive()) {
            RobotContainer.CHASSIS.fieldDrive(forward, leftward, angular);
        } else {
            RobotContainer.CHASSIS.stop();
        }
    }

    @Override
    public void end(boolean iterrupted) {
        RobotContainer.CHASSIS.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
