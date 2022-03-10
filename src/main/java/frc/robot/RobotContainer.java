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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToPosePID;
import frc.robot.commands.DriveToPosePID1;
import frc.robot.commands.FieldDriveWithJoysticks;
import frc.robot.commands.RobotDriveWithJoysticks;
import frc.robot.commands.RunArms;
import frc.robot.commands.RunClaws;
import frc.robot.commands.GatherBall;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Gather;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    @SuppressWarnings("unused")
    private static final IO _IO = new IO();

    public static final Chassis CHASSIS = new Chassis();
    public static final Vision VISION = new Vision();
    public static final Gather GATHER = new Gather();
    public static final Shooter SHOOTER = new Shooter();
    public static final Climber CLIMBER = new Climber();

    public static final PowerDistribution POWER_DISTRIBUTION = new PowerDistribution();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set the default commands for each subsystem
        CHASSIS.setDefaultCommand(new RobotDriveWithJoysticks());
        // Configure the button bindings
        configureButtonBindings();
    }

    public void init() {
        CHASSIS.init();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Command gatherCommand = new GatherBall();
        IO.Button.DriverRightTrigger.value.whileHeld(gatherCommand);
        IO.Button.DriverLeftTrigger.value.whileHeld(gatherCommand);
        IO.Button.DriverA.value.whileHeld(new Shoot(1000.0));

        IO.Button.ManipulatorLeftBumper.value.whileHeld(new RunClaws(1.0));
        IO.Button.ManipulatorRightBumper.value.whileHeld(new RunClaws(-1.0));
        IO.Button.ManipulatorStart.value.whileHeld(new RunArms(1.0));
        IO.Button.ManipulatorBack.value.whileHeld(new RunArms(-1.0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory testPath = PathPlanner.loadPath("Test", Constants.PATH_MAX_VELOCITY,
                Constants.PATH_MAX_ACCELERATION);

        Command command = CHASSIS.generatePathFollowCommand(testPath, new PIDController(1, 0, 0),
                new PIDController(0.8, 0, 0),
                new ProfiledPIDController(1.0, 0, 0,
                        new TrapezoidProfile.Constraints(Constants.PATH_MAX_ANGULAR_VELOCITY,
                                Constants.PATH_MAX_ANGULAR_ACCELERATION)));
        return command;
    }
}
