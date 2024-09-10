// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPoseEstimator pEstimator;
  /** Creates a new Photon. */
  public Photon() {
    camera = new PhotonCamera("photoncamera");
    PhotonPoseEstimator poseEstimator = null;
    
    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    if (camera != null) {
      poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
       new Transform3d(new Translation3d(0, 0, 0), 
       new Rotation3d(0.0 , Units.degreesToRadians(0), Units.degreesToRadians(0) )));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
