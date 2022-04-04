// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
    private static final PhotonCamera goalCamera = new PhotonCamera("goalCamera");

    public static final double GOAL_CAMERA_HEIGHT = Units.inchesToMeters(34.5);
    public static final double GOAL_CAMERA_PITCH = Units.degreesToRadians(35.0);
    public static final double GOAL_CAMERA_HORIZONTAL_FOV = Units.degreesToRadians(55.02);
    public static final double GOAL_CAMERA_VERTICAL_FOV = Units.degreesToRadians(30.15);
    public static final int GOAL_CAMERA_PIXEL_WIDTH = 1280;
    public static final int GOAL_CAMERA_PIXEL_HEIGHT = 720;

    private Translation2d lastGoalLocation;

    /** Creates a new Vision. */
    public Vision() {
        
    }

    @Override
    public void periodic() {
        lastGoalLocation = locateGoal(locateTargets(goalCamera.getLatestResult()));
        goalCamera.setDriverMode(false);
        if (lastGoalLocation != null) {
            System.out.println(lastGoalLocation);
            SmartDashboard.putBoolean("Target Found", true);
        } else {
            SmartDashboard.putBoolean("Target Found", false);
        }
    }

    public Translation2d getGoalLocation() {
        return lastGoalLocation;
    }

    public double calcYaw(double x) {
        double w = (double)GOAL_CAMERA_PIXEL_WIDTH / 2.0;
        double length = w / Math.tan(GOAL_CAMERA_HORIZONTAL_FOV / 2.0);
        return Math.atan2(x - w, length);
    }

    public double calcPitch(double y) {
        double h = (double)GOAL_CAMERA_PIXEL_HEIGHT / 2.0;
        double length = h / Math.tan(GOAL_CAMERA_VERTICAL_FOV / 2.0);
        return Math.atan2(h - y, length);
    }

    public Translation2d targetOffset(double yaw, double pitch, double targetHeight) {
        double height = targetHeight - GOAL_CAMERA_HEIGHT;
        double dist = height / Math.tan(GOAL_CAMERA_PITCH + pitch);
        double length = Math.sqrt(dist * dist + height * height);
        double x = Math.tan(yaw) * length;
        return new Translation2d(x, dist);
    }

    public static Translation2d circleFit(Translation2d[] points, double radius, double lossLimit) {
        if (points.length < 8) {
            return null;
        }

        var zero = new Translation2d();
        var r = radius;
        var center = new Translation2d(0.0, 0.0);
        for (var point : points) {
            center = center.plus(point);
        }
        center = center.div(points.length);
        for (int j = 0; j < 50; j++) {
            double loss = 0.0;
            for (var point : points) {
                loss += Math.abs(point.minus(center).getDistance(zero) - radius);
            }
            loss /= points.length;
            if (loss <= lossLimit) {
                break;
            }

            var lossDeriv = new double[] {0.0, 0.0};
            for (var point : points) {
                var p = new double[] {point.getX(), point.getY()};
                var c = new double[] {center.getX(), center.getY()};
                double d = point.getDistance(center);
                for (int i = 0; i < 2; i++) {
                    lossDeriv[i] -= ((p[i] - c[i]) * Math.abs(d - r)) / (d * (d - r));
                }
            }
            lossDeriv[0] /= points.length;
            lossDeriv[1] /= points.length;

            center = center.minus((new Translation2d(lossDeriv[0], lossDeriv[1])).times(1.0 - Math.exp(-loss)));

            if (j == 49) {
                // System.out.println("iteration cap reached; loss: " + loss);
                center = null;
            }
        }
        return center;
    }

    void plotPoints(Translation2d[] points, Translation2d center) {
        var pose = frc.robot.RobotContainer.CHASSIS.getCurrentPose();
        List<Double> fieldPoints = new ArrayList<Double>();
        double yaw = -pose.getRotation().getRadians();
        for (int i = 0; i <= points.length; i++) {
            var point = i == points.length ? center : points[i];
            double x = -pose.getY() * 10.0 + point.getX() * Math.cos(yaw) + point.getY() * Math.sin(yaw);
            double y = pose.getX() * 10.0 + point.getY() * Math.cos(yaw) - point.getX() * Math.sin(yaw);
            fieldPoints.add(x);
            fieldPoints.add(y);
        }
        SmartDashboard.putNumberArray("Vision Target Locations", fieldPoints.toArray(new Double[fieldPoints.size()]));
    }

    public Translation2d[][] locateTargets(PhotonPipelineResult result) {
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
            List<TargetCorner> upperCorners = new ArrayList<TargetCorner>();
            List<TargetCorner> lowerCorners = new ArrayList<TargetCorner>();
            List<TargetCorner> centers = new ArrayList<TargetCorner>();

            for (int i = 0; i < result.targets.size(); i++) {
                var target = result.targets.get(i);
                targets.add(target);
                List<TargetCorner> targetCorners = target.getCorners();
                Collections.sort(targetCorners, Comparator.comparing((corner) -> corner.y));
                upperCorners.add(targetCorners.get(0));
                upperCorners.add(targetCorners.get(1));
                lowerCorners.add(targetCorners.get(2));
                lowerCorners.add(targetCorners.get(3));
                double centerX = (
                    targetCorners.get(0).x
                    + targetCorners.get(1).x
                    + targetCorners.get(2).x
                    + targetCorners.get(3).x
                ) / 4.0;
                double centerY = (
                    targetCorners.get(0).y
                    + targetCorners.get(1).y
                    + targetCorners.get(2).y
                    + targetCorners.get(3).y
                ) / 4.0;
                centers.add(new TargetCorner(centerX, centerY));
            }

            double upperHeight = Constants.TARGET_HEIGHT + Constants.TARGET_TAPE_WIDTH / 2.0;
            double lowerHeight = Constants.TARGET_HEIGHT - Constants.TARGET_TAPE_WIDTH / 2.0;

            List<Translation2d> upperProjected = new ArrayList<Translation2d>();
            List<Translation2d> lowerProjected = new ArrayList<Translation2d>();
            List<Translation2d> centerProjected = new ArrayList<Translation2d>();

            for (var pos : upperCorners) {
                double yaw = calcYaw(pos.x);
                double pitch = calcPitch(pos.y);
                upperProjected.add(targetOffset(yaw, pitch, upperHeight));
            }

            for (var pos : lowerCorners) {
                double yaw = calcYaw(pos.x);
                double pitch = calcPitch(pos.y);
                lowerProjected.add(targetOffset(yaw, pitch, lowerHeight));
            }

            for (var pos : centers) {
                double yaw = calcYaw(pos.x);
                double pitch = calcPitch(pos.y);
                centerProjected.add(targetOffset(yaw, pitch, Constants.TARGET_HEIGHT));
            }

            Translation2d[] upperProjectedArray = upperProjected.toArray(new Translation2d[upperProjected.size()]);
            Translation2d[] lowerProjectedArray = lowerProjected.toArray(new Translation2d[lowerProjected.size()]);
            Translation2d[] centerProjectedArray = centerProjected.toArray(new Translation2d[centerProjected.size()]);
            return new Translation2d[][] {upperProjectedArray, lowerProjectedArray, centerProjectedArray};
        }
        var empty = new Translation2d[] {};
        return new Translation2d[][] {empty, empty, empty};
    }

    public Translation2d locateGoal(Translation2d[][] targets) {
        Translation2d[] points = Stream.of(targets[0], targets[1], targets[2])
            .flatMap(Stream::of)
            .toArray(Translation2d[]::new);
        var circleCenter = circleFit(points, Constants.TARGET_RADIUS, Units.inchesToMeters(2.0));
        if (circleCenter != null) {
            plotPoints(points, circleCenter);
        }
        return circleCenter;
    }

    // Currently nonfunctional
    double updateGyro(double gyroAngle, Translation2d[][] targets) {
        var centers = targets[2];
        if (centers.length == 0) {
            return gyroAngle;
        }
        Translation2d closestCenter = null;
        double minSqrDist = Double.POSITIVE_INFINITY;
        for (var center : centers) {
            double sqrDist = center.getX() * center.getX() + center.getY() * center.getY();
            if (sqrDist < minSqrDist) {
                closestCenter = center;
                minSqrDist = sqrDist;
            }
        }

        Translation2d goalCenter = locateGoal(targets);
        if (goalCenter == null) {
            return gyroAngle;
        }

        double yawOffset = Math.atan2(goalCenter.getX() - closestCenter.getX(), goalCenter.getY() - closestCenter.getY());

        double[] possibleAngles = new double[16];
        for (int i = 0; i < 16; i++) {
            possibleAngles[i] = Constants.TARGET_OFFSET_ANGLE + (double)i * Math.PI / 8.0;
            possibleAngles[i] += yawOffset;
        }

        double closestPossibleAngle = Double.POSITIVE_INFINITY;
        for (var angle : possibleAngles) {
            if (Math.abs(angle - gyroAngle) < Math.abs(closestPossibleAngle - gyroAngle)) {
                closestPossibleAngle = angle;
            }
        }

        double confidence = 1.0 - Math.pow(1.4938, -(centers.length - 1));
        System.out.println("centers: " + centers.length + ", confidence: " + confidence);
        return gyroAngle * (1.0 - confidence) + closestPossibleAngle * confidence;
    }
}
