package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Chassis;

import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    // FIELD PARAMETERS
    // public static final Pose2d GOAL_POSE = new Pose2d(new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(0.5 * 27.0)), new Rotation2d(Units.degreesToRadians(24.0)));
    // public static final Translation2d TARGET_LOCK_POSITION = new Translation2d(-0.1, 4.0);
    // public static final double TARGET_HEIGHT = Units.inchesToMeters(8.0 * 12.0 + 7.0 - 2.0);
    // public static final double TARGET_TAPE_WIDTH = Units.inchesToMeters(2.0);
    // public static final double TARGET_RADIUS = Units.inchesToMeters(26.0 + 11.0 / 16.0);
    // public static final double TARGET_OFFSET_ANGLE = Units.degreesToRadians(9.75);

    // AUTO PATHS
    public static final Map<String, PathPlannerTrajectory> AUTO_PATHS = Map.of(
        "Drive Backwards", PathPlanner.loadPath("Drive Backwards", Chassis.MAX_DRIVE_SPEED, Chassis.MAX_DRIVE_ACCELERATION),
        "Bottom 2 Ball", PathPlanner.loadPath("Bottom 2 Ball", Chassis.MAX_DRIVE_SPEED, Chassis.MAX_DRIVE_ACCELERATION)
        
    );
    
    public static final AutonomousRoutine[] AUTO_ROUTINES = {
        new AutonomousRoutine(
            new DriveToPose(new Pose2d(4.39, 4.09, new Rotation2d(0.0)), 0.1),
            new Pose2d(6.09, 4.09, new Rotation2d(0.0))
        ),
    };

    public static class AutonomousRoutine {

        public final Pose2d INITIAL_POSE;
        public final Command COMMAND;

        private AutonomousRoutine(Command command, Pose2d initialPose) {
            this.COMMAND = command;
            this.INITIAL_POSE = initialPose;
        }
    }
}
