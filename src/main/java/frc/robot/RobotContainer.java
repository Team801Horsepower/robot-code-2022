/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmReset;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.ColorWheelDown;
import frc.robot.commands.ColorWheelUp;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FieldDriveWithJoysticks;
import frc.robot.commands.ForwardGather;
import frc.robot.commands.ReverseGather;
import frc.robot.commands.RobotDriveWithJoysticks;
import frc.robot.commands.Shoot;
import frc.robot.commands.WinchUp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Gatherer;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Winch;

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
    public static final Vision vision = new Vision();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set the default commands for each subsystem
        chassis.setDefaultCommand(new RobotDriveWithJoysticks());
        winch.setDefaultCommand(new WinchUp());
        // Configure the button bindings
        configureButtonBindings();
    }

    public void init() {
        chassis.init();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        IO.Button.DriverLeftBumper.value.whileHeld(new FieldDriveWithJoysticks());
        IO.Button.DriverRightBumper.value.whileHeld(new DriveToPose(new Pose2d()));

        IO.Button.DriverX.value.whenPressed(new ArmReset());
        IO.Button.DriverA.value.whenPressed(new ArmToPosition(Constants.ARM_POSITION_LOW));
        IO.Button.DriverB.value.whenPressed(new ArmToPosition(Constants.ARM_POSITION_MID));
        IO.Button.DriverY.value.whenPressed(new ArmToPosition(Constants.ARM_POSITION_HIGH));

        IO.Button.ManipulatorY.value.whenPressed(new ColorWheelUp());
        IO.Button.ManipulatorX.value.whenPressed(new ColorWheelDown());
        IO.Button.ManipulatorLeftBumper.value.whileHeld(new ForwardGather());
        IO.Button.ManipulatorRightBumper.value.whileHeld(new ReverseGather());
        IO.Button.ManipulatorB.value.whileHeld(new Shoot());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory testPath = PathPlanner.loadPath("Test", Constants.PATH_MAX_VELOCITY,
                Constants.PATH_MAX_ACCELERATION);

        Command command = chassis.generatePathFollowCommand(testPath, new PIDController(1, 0, 0),
                new PIDController(0.8, 0, 0),
                new ProfiledPIDController(1.0, 0, 0,
                        new TrapezoidProfile.Constraints(Constants.PATH_MAX_ANGULAR_VELOCITY,
                                Constants.PATH_MAX_ANGULAR_ACCELERATION)));
        return command;
    }
}
