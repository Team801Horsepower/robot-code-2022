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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RobotDriveWithJoysticks;
import frc.robot.commands.RunArms;
import frc.robot.commands.RunClaws;
import frc.robot.commands.Climb;
import frc.robot.commands.FieldDriveWithJoysticks;
import frc.robot.commands.GatherAuto;
import frc.robot.commands.GatherBall;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Gather;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

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

    private static final PathPlannerTrajectory AUTO_PATH = PathPlanner.loadPath("Test", Constants.PATH_MAX_VELOCITY, Constants.PATH_MAX_ACCELERATION);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set the default commands for each subsystem
        CHASSIS.setDefaultCommand(new FieldDriveWithJoysticks());
        // Configure the button bindings
        configureButtonBindings();
    }

    public void init() {
        CHASSIS.init(AUTO_PATH.getInitialPose());
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

        IO.Button.DriverA.value.whenPressed(
            GATHER.tampBall()
        );
        IO.Button.DriverB.value.whenPressed(
            GATHER.tampBall().andThen(
                SHOOTER.freeBall().andThen(
                    GATHER.tampBall()).andThen(
                        () -> SHOOTER.setSpeed(
                            Units.rotationsPerMinuteToRadiansPerSecond(3000.0)), SHOOTER)));
        IO.Button.DriverB.value.whenReleased(SHOOTER::stop, SHOOTER);

        IO.Button.DriverX.value.whenPressed(() -> SHOOTER.setSpeed(Units.rotationsPerMinuteToRadiansPerSecond(3000)), SHOOTER);
        IO.Button.DriverX.value.whenReleased(SHOOTER::stop, SHOOTER);

        IO.Button.DriverLeftBumper.value.whileHeld(new RobotDriveWithJoysticks());

        IO.Button.ManipulatorLeftBumper.value.whileHeld(new RunClaws(0.03));
        IO.Button.ManipulatorRightBumper.value.whileHeld(new RunClaws(-0.03));

        Command climbCommand = new Climb();
        IO.Button.ManipulatorRightTrigger.value.whileHeld(climbCommand);
        IO.Button.ManipulatorLeftTrigger.value.whileHeld(climbCommand);

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

        Command command = CHASSIS.generatePathFollowCommand(
            AUTO_PATH,
                new PIDController(1, 0, 0),
                new PIDController(0.8, 0, 0),
                new ProfiledPIDController(1.0, 0, 0,
                        new TrapezoidProfile.Constraints(Constants.PATH_MAX_ANGULAR_VELOCITY,
                                Constants.PATH_MAX_ANGULAR_ACCELERATION)))
                                .alongWith(new GatherAuto());
        return command;
    }
}
