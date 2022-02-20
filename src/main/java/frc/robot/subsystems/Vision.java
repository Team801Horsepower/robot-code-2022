// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class Vision extends SubsystemBase {
    /** Creates a new Vision. */
    public Vision() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonCamera camera = new PhotonCamera("mmal_service_16.1");
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            List<TargetCorner> upperCorners = new ArrayList<TargetCorner>();
            List<TargetCorner> lowerCorners = new ArrayList<TargetCorner>();

            for (int i = 0; i < result.targets.size(); i++) {
                List<TargetCorner> targetCorners = result.targets.get(i).getCorners();
                Collections.sort(targetCorners, Comparator.comparing((corner) -> corner.y));
                upperCorners.add(targetCorners.get(0));
                upperCorners.add(targetCorners.get(1));
                lowerCorners.add(targetCorners.get(2));
                lowerCorners.add(targetCorners.get(3));
            }

            double upperHeight = Constants.TARGET_HEIGHT + Constants.TARGET_TAPE_WIDTH / 2.0;
            double lowerHeight = Constants.TARGET_HEIGHT - Constants.TARGET_TAPE_WIDTH / 2.0;
            List<Translation2d> projectedPoints = new ArrayList<Translation2d>(upperCorners.size() * 2);
            System.out.println("--- BEGIN UPPER Y/P ---");
            for (var corner : upperCorners) {
                double pitch = corner.x * Math.PI / 180.0;
                double yaw = corner.y * Math.PI / 180.0;
                System.out.println("(" + yaw + ", " + pitch + ")");
                projectedPoints.add(targetOffset(pitch, yaw, upperHeight));
            }
            System.out.println("---  END UPPER Y/P  ---");
            System.out.println("--- BEGIN LOWER Y/P ---");
            for (var corner : lowerCorners) {
                double pitch = corner.x * Math.PI / 180.0;
                double yaw = corner.y * Math.PI / 180.0;
                System.out.println("(" + yaw + ", " + pitch + ")");
                projectedPoints.add(targetOffset(pitch, yaw, lowerHeight));
            }
            System.out.println("---  END LOWER Y/P  ---");
            
            var circleCenter = circleFit(projectedPoints.toArray(new Translation2d[projectedPoints.size()]), Constants.TARGET_RADIUS, Units.inchesToMeters(2.0));
            System.out.println("Circle center: " + circleCenter);
        }
    }

    public Translation2d targetOffset(double pitch, double yaw, double targetHeight) {
        double dist = (targetHeight - Constants.CAMERA_HEIGHT) / Math.tan(pitch);
        double x = Math.sin(yaw) * dist;
        double y = Math.cos(yaw) * dist;
        return new Translation2d(x, y);
    }

    public static Translation2d circleFit(Translation2d[] points, double radius, double lossLimit) {
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
            if (loss <= lossLimit) {
                System.out.println("loss low enough: " + loss);
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

            // System.out.println("lossDeriv: " + lossDeriv[0] + ", " + lossDeriv[1]);
            center = center.minus((new Translation2d(lossDeriv[0], lossDeriv[1])).times(Math.exp(-loss)));
            
            if (j == 49) {
                System.out.println("iteration cap reached");
            }
        }
        return center;
    }
}
