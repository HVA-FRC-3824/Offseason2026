// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.poseVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.o2026.subsystems.drivebase.poseVision.PoseVision.VisionData;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PoseCameraIOPhoton implements PoseCameraIO {

  private Matrix<N4, N1> curStdDevs = Configs.Vision.kSingleTagStdDevs;

  private PhotonCamera m_camera;
  private PhotonPoseEstimator estimator;

  private Transform3d m_offset;

  public PoseCameraIOPhoton(String name, Transform3d m_offset) {

    m_camera = new PhotonCamera(name);
    estimator = new PhotonPoseEstimator(Constants.Vision.TagLayout, m_offset);

    this.m_offset = m_offset;
  }

  @Override
  public ArrayList<VisionData> getMeasurements() {

    return new ArrayList<>(
        m_camera.getAllUnreadResults().stream()
            .filter((result) -> result.hasTargets())
            .flatMap((result) -> estimator.estimateCoprocMultiTagPose(result).stream())
            .map(
                est -> {
                  Logger.recordOutput("m-pv", est.estimatedPose);
                  var targets =
                      est.targetsUsed.stream().mapToInt((target) -> target.fiducialId).toArray();
                  PoseCameraIO.getEstimationStdDevs(est.estimatedPose.toPose2d(), targets);
                  return new VisionData(
                      est.estimatedPose, est.timestampSeconds, curStdDevs, targets);
                })
            .toList());
  }

  public Transform3d getOffset() {
    return m_offset;
  }
}
