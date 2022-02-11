package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link PathPlannerTrajectory}
 *
 * <p>
 * This command outputs the ChassisSpeeds to follow the path.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory but rather goes to
 * the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class PathPlannerControllerCommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final HolonomicDriveController m_controller;
    private final Consumer<ChassisSpeeds> m_outputSpeeds;

    /**
     * Constructs a new ({@link PathPlannerControllerCommand}) that when executed will follow the
     * provided trajectory. This command will not return output voltages but rather
     * ({@link ChassisSpeeds}) from the position controllers.
     *
     * <p>
     * Note: The controllers will *not* set the speeds to zero upon completion of the path- this is
     * left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *        provide this.
     * @param xController The Trajectory Tracker PID controller for the robot's x position.
     * @param yController The Trajectory Tracker PID controller for the robot's y position.
     * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
     * @param outputSpeeds The raw output module states from the position controllers.
     * @param requirements The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public PathPlannerControllerCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> pose,
            PIDController xController, PIDController yController,
            ProfiledPIDController thetaController, Consumer<ChassisSpeeds> outputSpeeds,
            Subsystem... requirements) {
        m_trajectory = trajectory;
        m_pose = pose;

        m_controller = new HolonomicDriveController(xController, yController, thetaController);
        m_outputSpeeds = outputSpeeds;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        var targetChassisSpeeds =
                m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
        m_outputSpeeds.accept(targetChassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}

