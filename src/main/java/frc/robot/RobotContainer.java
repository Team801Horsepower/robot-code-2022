/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ArmLow;
import frc.robot.commands.ArmMid;
import frc.robot.commands.ColorWheelDown;
import frc.robot.commands.ColorWheelUp;
import frc.robot.commands.FieldDriveWithJoysticks;
import frc.robot.commands.RobotDriveWithJoysticks;
import frc.robot.commands.WinchUp;
import frc.robot.commands.ForwardGather;
import frc.robot.commands.ReverseGather;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Gatherer;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Winch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ColorWheel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public static final IO io = new IO();

    public static final Chassis chassis = new Chassis();
    public static final Gatherer gatherer = new Gatherer();
    public static final Magazine magazine = new Magazine();
    public static final Arm arm = new Arm();
    public static final Winch winch = new Winch();
    public static final Shooter shooter = new Shooter();
    public static final ColorWheel colorWheel = new ColorWheel();

    private final Command m_autoCommand = new FieldDriveWithJoysticks();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set the default commands for each subsystem
        winch.setDefaultCommand(new WinchUp());
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        IO.Button.DriverLeftBumper.value.whileHeld(new RobotDriveWithJoysticks());
    }

    /** Reinitializes all the subsystems without a reboot */
    public void reset() {
        chassis.init();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoCommand;
    }
}
