/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RobotDriveWithJoysticks;
// import frc.robot.commands.RunClaws;
import frc.robot.commands.RunShooter;
import frc.robot.commands.Aim;
import frc.robot.commands.Climb;
import frc.robot.commands.FieldDriveWithJoysticks;
import frc.robot.commands.Gather;
import frc.robot.commands.PathPlannerControllerCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Gatherer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.Utils;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    @SuppressWarnings("unused")
    private static final IO _IO = new IO();

    public static final Chassis CHASSIS = new Chassis();
    public static final Vision VISION = new Vision();
    public static final Gatherer GATHER = new Gatherer();
    public static final Shooter SHOOTER = new Shooter();
    public static final Climber CLIMBER = new Climber();
    public static final Feeder FEEDER = new Feeder();

    public static final PowerDistribution POWER_DISTRIBUTION = new PowerDistribution();
    public static final Constants.AutonomousRoutine AUTONOMOUS_ROUTINE = Constants.AUTO_ROUTINES[Preferences.getInt("AUTO_ROUNTINE", 0)];

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CHASSIS.setDefaultCommand(new FieldDriveWithJoysticks());
        configureButtonBindings();
    }

    public void init() {
        CHASSIS.init(AUTONOMOUS_ROUTINE.INITIAL_POSE);
        GATHER.clear();
    }

    public void reset() {
        CHASSIS.reset();
        GATHER.reset();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Command gatherCommand = new Gather(true);
        IO.Button.DriverRightTrigger.value.whileHeld(gatherCommand);
        IO.Button.DriverLeftTrigger.value.whileHeld(gatherCommand);

        IO.Button.DriverLeftBumper.value.whileHeld(new Aim());
        IO.Button.DriverRightBumper.value.whenPressed(
            (new RunShooter())
            .andThen(FEEDER.feed(true))
            .andThen(new RunShooter())
            .andThen(() -> GATHER.run(1.0))
            .andThen(() -> FEEDER.run(1.0))
        ).whenReleased(
            () -> {
                GATHER.stop();
                FEEDER.stop();
                SHOOTER.stop();
            }, GATHER, SHOOTER, FEEDER
        );

        IO.Button.DriverA.value.whenPressed(GATHER.tampBall());
        IO.Button.DriverY.value.whenPressed(GATHER.fireBall());
        IO.Button.DriverX.value.toggleWhenPressed(new RunShooter());
        IO.Button.DriverB.value.whenPressed(() -> GATHER.run(1.0), GATHER).whenReleased(() -> GATHER.stop(), GATHER);

        IO.Button.DriverLeftStick.value.toggleWhenPressed(new RobotDriveWithJoysticks());
        //IO.Button.DriverBack.value.whenPressed(SHOOTER::runBelt).whenReleased(SHOOTER::stop);

        // IO.Button.ManipulatorLeftBumper.value.whileHeld(new RunClaws(0.03));
        // IO.Button.ManipulatorRightBumper.value.whileHeld(new RunClaws(-0.03));

        Command climbCommand = new Climb();
        IO.Button.ManipulatorRightTrigger.value.whileHeld(climbCommand);
        IO.Button.ManipulatorLeftTrigger.value.whileHeld(climbCommand);

        // IO.Button.ManipulatorX.value.whenPressed(() -> CLIMBER.setClawPosition(2.369), CLIMBER);
        // IO.Button.ManipulatorY.value.whenPressed(() -> CLIMBER.setClawPosition(-0.343), CLIMBER);
        // IO.Button.ManipulatorB.value.whenPressed(() -> CLIMBER.setClawPosition(-2.4), CLIMBER);
        // IO.Button.ManipulatorA.value.whenPressed(() -> CLIMBER.raiseArm(), CLIMBER);
        IO.Button.ManipulatorStart.value.whenPressed(() -> CLIMBER.setDesiredPosition(1.088));
        IO.Button.ManipulatorA.value.whenPressed(() -> CLIMBER.setDesiredPosition(-1.5));
        IO.Button.ManipulatorX.value.whenPressed(() -> CLIMBER.setDesiredPosition(0.0));
        IO.Button.ManipulatorY.value.whenPressed(() -> CLIMBER.setDesiredPosition(-4.67));
        IO.Button.ManipulatorB.value.whenPressed(() -> CLIMBER.setDesiredPosition(-Math.PI));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AUTONOMOUS_ROUTINE.COMMAND;
    }
}
