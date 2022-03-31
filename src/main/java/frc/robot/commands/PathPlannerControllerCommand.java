package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory
 * {@link PathPlannerTrajectory}
 *
 * <p>
 * This command outputs the ChassisSpeeds to follow the path.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes to
 * the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class PathPlannerControllerCommand extends DriveToPose  {
    private final Timer m_timer = new Timer();
    public final PathPlannerTrajectory m_trajectory;

    /**
     * Constructs a new ({@link PathPlannerControllerCommand}) that when executed
     * will follow the
     * provided trajectory. This command will not return output voltages but rather
     * ({@link ChassisSpeeds}) from the position controllers.
     *
     * <p>
     * Note: The controllers will *not* set the speeds to zero upon completion of
     * the path- this is
     * left to the user, since it is not appropriate for paths with nonstationary
     * endstates.
     *
     * @param trajectory      The trajectory to follow.
     */
    public PathPlannerControllerCommand(PathPlannerTrajectory trajectory, double distanceTolerance) {
        super(RobotContainer.CHASSIS.getCurrentPose(), distanceTolerance);
        m_trajectory = trajectory;
    }

    public PathPlannerControllerCommand(String pathName, double distanceTolerance) {
        this(PathPlanner.loadPath(pathName, Chassis.MAX_DRIVE_SPEED, Chassis.MAX_DRIVE_ACCELERATION), distanceTolerance);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        super.initialize();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        targetPose = new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation);
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()) && super.isFinished();
    }
}
