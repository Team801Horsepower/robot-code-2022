// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonCamera camera = new PhotonCamera("mmal_service_16.1");
    var result = camera.getLatestResult();

    double cameraHeightMeters = Units.inchesToMeters(30);
    double targetHeightMeters = Units.inchesToMeters(67.5);
    double cameraPitchRadians = Units.degreesToRadians(16.7);

    if (result.hasTargets()) {
      double range1 =
          PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters,
              cameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));
      double avgArea = 0;
      for (var target : result.getTargets()) {
        avgArea += target.getArea();
      }
      avgArea /= result.getTargets().size();
      System.out.println("Range:" + range1 + " Avg Area: " + avgArea);
    }


  }
}
