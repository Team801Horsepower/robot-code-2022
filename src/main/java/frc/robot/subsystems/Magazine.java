/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// NOTE: Complain to Luke Newcomb for problems with this subsystem

public class Magazine extends SubsystemBase
{
    private CANSparkMax sparkMotor;
    private CANPIDController sparkPID;

    /**
     * Creates a new Magazine
     */
    public Magazine()
    {
        // Initialize the Magazine motor (NEO)
        sparkMotor = new CANSparkMax(Constants.MAGAZINE_MOTOR_ID, MotorType.kBrushless);
        sparkMotor.setInverted(Constants.MAGAZINE_INVERTED);
        sparkPID = sparkMotor.getPIDController();
        
    }

    public void forward()
    {
        sparkPID.setReference(Constants.MAGAZINE_SPEED, ControlType.kDutyCycle);
    }

    public void stop()
    {
        sparkPID.setReference(0, ControlType.kDutyCycle);
    }

    public void reverse()
    {
        sparkPID.setReference(-Constants.MAGAZINE_SPEED, ControlType.kDutyCycle);
    }
}
