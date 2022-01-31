/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ForwardGather extends CommandBase
{
    /**
     * Creates a new DriveWithJoysticks.
     */
    public ForwardGather()
    {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.gatherer);
        addRequirements(RobotContainer.magazine);
        //addRequirements(RobotContainer.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        RobotContainer.gatherer.forward();
        RobotContainer.magazine.forward();
        //RobotContainer.shooter.holdDown();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.gatherer.stop();
        RobotContainer.magazine.stop();
        //RobotContainer.shooter.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
