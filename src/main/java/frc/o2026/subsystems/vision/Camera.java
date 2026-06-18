// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.o2026.Constants;
import frc.o2026.Robot;
import frc.o2026.subsystems.vision.Vision.VisionData;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Camera {

  private Matrix<N4, N1> curStdDevs = Constants.Vision.kSingleTagStdDevs;

  private int m_xRes = 1080;
  private int m_yRes = 720;
  private double m_fov = 70;
  private double m_fps = 15;

  private PhotonCamera m_camera;
  private PhotonCameraSim m_simCamera;
  private PhotonPoseEstimator estimator;

  private Transform3d m_offset;

  boolean m_ignored;

  public Camera(String name, Transform3d m_offset) {

    m_camera = new PhotonCamera(name);
    estimator = new PhotonPoseEstimator(Constants.Vision.kTagLayout, m_offset);

    this.m_offset = m_offset;

    if (Robot.isSimulation()) {

      SimCameraProperties cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(m_xRes, m_yRes, Rotation2d.fromDegrees(m_fov));
      cameraProp.setCalibError(0.35, 0.10);
      cameraProp.setFPS(m_fps);
      cameraProp.setAvgLatencyMs(50);
      cameraProp.setLatencyStdDevMs(15);

      m_simCamera = new PhotonCameraSim(m_camera, cameraProp);

      m_simCamera.enableDrawWireframe(true);
    }
  }

  public ArrayList<VisionData> getMeasurements() {

    ArrayList<VisionData> measurements = new ArrayList<>();
    for (var result : m_camera.getAllUnreadResults()) {

      Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) visionEst = estimator.estimateLowestAmbiguityPose(result);

      updateEstimationStdDevs(visionEst, result.getTargets());

      if (visionEst.isEmpty()) continue;

      // Check if we're using any AprilTags (i.e. targets)
      EstimatedRobotPose est = visionEst.get();
      if (est.targetsUsed.size() == 0) continue;

      // Populate a list of targets to add to measurements
      int[] targetsUsed = new int[est.targetsUsed.size()];
      for (int i = 0; i < est.targetsUsed.size(); i++)
        targetsUsed[i] = est.targetsUsed.get(i).fiducialId;

      measurements.add(
          new VisionData(est.estimatedPose, est.timestampSeconds, curStdDevs, targetsUsed));
    }
    return measurements;
  }

  public PhotonCameraSim getSimCamera() {
    return m_simCamera;
  }

  public Transform3d getOffset() {
    return m_offset;
  }

  /// PHOTONVISION PROVIDED
  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this m_camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = Constants.Vision.kSingleTagStdDevs;
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = Constants.Vision.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;

        numTags++;
        avgDist +=
            tagPose
                .get()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = Constants.Vision.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.Vision.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        // max distance 15 meters
        if (numTags == 1 && avgDist > 15) {
          estStdDevs =
              VecBuilder.fill(
                  Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (Math.pow(avgDist, 2) / 30));
        }
        curStdDevs = estStdDevs;
      }
    }
  }
}
