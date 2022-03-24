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
import frc.robot.commands.RunClaws;
import frc.robot.commands.RunShooter;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FieldDriveWithJoysticks;
import frc.robot.commands.GatherBall;
import frc.robot.commands.PathPlannerControllerCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Gather;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

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
    public static final Gather GATHER = new Gather();
    public static final Shooter SHOOTER = new Shooter();
    public static final Climber CLIMBER = new Climber();

    public static final PowerDistribution POWER_DISTRIBUTION = new PowerDistribution();

    private static final PathPlannerTrajectory AUTO_PATH = PathPlanner.loadPath(Preferences.getString("AUTO_PATH", "Drive Backwards"), Chassis.MAX_DRIVE_SPEED,
            Chassis.MAX_DRIVE_ACCELERATION);
    private static final Pose2d INITIAL_POSE = new Pose2d(AUTO_PATH.getInitialPose().getTranslation(), AUTO_PATH.getInitialState().holonomicRotation);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CHASSIS.setDefaultCommand(new FieldDriveWithJoysticks());
        configureButtonBindings();
    }

    public void init() {
        CHASSIS.init(INITIAL_POSE);
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
        Command gatherCommand = new GatherBall();
        IO.Button.DriverRightTrigger.value.whileHeld(gatherCommand);
        IO.Button.DriverLeftTrigger.value.whileHeld(gatherCommand);

        IO.Button.DriverRightBumper.value.whileHeld(new DriveToPose(INITIAL_POSE, 0.05));

        IO.Button.DriverA.value.whenPressed(GATHER.tampBall());
        IO.Button.DriverY.value.whenPressed(GATHER.fireBall());
        IO.Button.DriverX.value.toggleWhenPressed(new RunShooter());
        IO.Button.DriverB.value.whenPressed(() -> GATHER.run(1.0), GATHER).whenReleased(() -> GATHER.stop(), GATHER);

        IO.Button.DriverLeftStick.value.toggleWhenPressed(new RobotDriveWithJoysticks());

        IO.Button.ManipulatorLeftBumper.value.whileHeld(new RunClaws(0.03));
        IO.Button.ManipulatorRightBumper.value.whileHeld(new RunClaws(-0.03));

        Command climbCommand = new Climb();
        IO.Button.ManipulatorRightTrigger.value.whileHeld(climbCommand);
        IO.Button.ManipulatorLeftTrigger.value.whileHeld(climbCommand);

        IO.Button.ManipulatorX.value.whenPressed(() -> CLIMBER.setClawPosition(2.369), CLIMBER);
        IO.Button.ManipulatorY.value.whenPressed(() -> CLIMBER.setClawPosition(-0.343), CLIMBER);
        IO.Button.ManipulatorB.value.whenPressed(() -> CLIMBER.setClawPosition(-2.4), CLIMBER);
        IO.Button.ManipulatorA.value.whenPressed(() -> CLIMBER.raiseArm(), CLIMBER);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Command command =  new PathPlannerControllerCommand(AUTO_PATH, 0.05);
        return command;
    }
}
