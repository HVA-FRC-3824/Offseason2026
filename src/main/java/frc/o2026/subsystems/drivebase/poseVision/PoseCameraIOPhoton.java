// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.poseVision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.hardware.vision.VisionConfig;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import frc.o2026.subsystems.drivebase.poseVision.PoseVision.VisionData;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class PoseCameraIOPhoton implements PoseCameraIO {

  private Matrix<N4, N1> curStdDevs = Configs.Vision.kSingleTagStdDevs;

  protected PhotonCamera m_camera;
  private PhotonPoseEstimator estimator;

  private final VisionConfig m_config;

  private Optional<Consumer<Rotation3d>> m_gyroResetter = Optional.empty();

  private List<Pose2d> m_lastSeenTags = List.of();

  public PoseCameraIOPhoton(VisionConfig config) {

    m_config = config;

    m_camera = new PhotonCamera(m_config.name());
    estimator = new PhotonPoseEstimator(Constants.Vision.TagLayout, m_config.offset());
  }

  public void addGyroResetter(Consumer<Rotation3d> gyroResetter) {

    m_gyroResetter = Optional.of(gyroResetter);
  }

  @Override
  public ArrayList<VisionData> getMeasurements() {

    return new ArrayList<VisionData>(
        m_camera.getAllUnreadResults().stream()
            .filter(PhotonPipelineResult::hasTargets)
            .filter(result -> result.getBestTarget().poseAmbiguity > 0.3)
            .map(
                (result) -> {

                  // Use multiple tags to create a very accurate pose estimate
                  var est = estimator.estimateCoprocMultiTagPose(result);
                  // Because we trust this estimate so much we can reset our rotation to it
                  // Often, the gyro will drift significantly, this corrects it
                  if (est.isPresent() && result.getBestTarget().poseAmbiguity > 0.5) {
                    if (m_gyroResetter.isPresent())
                      m_gyroResetter.get().accept(est.get().estimatedPose.getRotation());
                    return est;
                  }

                  // Use gyro data in combination with tag data to get an estimate as
                  // accurate as your gyro
                  estimator.addHeadingData(
                      Timer.getTimestamp(), RobotState.getPoseEst().getRotation().toRotation2d());
                  // I've found that this data can be inaccurate when rotating at high velocity
                  if (RobotState.getAngularVelocity().lte(DegreesPerSecond.of(5.0))) {
                    est = estimator.estimatePnpDistanceTrigSolvePose(result);
                    if (est.isPresent()) return est;
                  }

                  // Take multiple estimations from multiple tags and average their results
                  est = estimator.estimateAverageBestTargetsPose(result);
                  if (est.isPresent()) return est;

                  // No complicated sensor fusion between multiple tags or gyro
                  // simply the best guess
                  est = estimator.estimateLowestAmbiguityPose(result);
                  return est;
                })
            .filter(Optional::isPresent)
            .map(Optional::get)
            // Filter frivolous pose estimates
            // .filter(
            //     result -> {
            //       var speeds = RobotState.getLastMeasuredSpeeds();
            //       var dist =
            //           result
            //               .estimatedPose
            //               .getTranslation()
            //               .getDistance(RobotState.getPoseEst().getTranslation());
            //       return dist
            //           < Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            //               * Configs.Vision.ScalarPositionTolerance;
            //     })
            // Filter frivolous rotations
            .filter(
                result -> {
                  var velocity = RobotState.getLastMeasuredSpeeds().omegaRadiansPerSecond;
                  var delta =
                      Math.abs(
                          result
                              .estimatedPose
                              .getRotation()
                              .minus(RobotState.getPoseEst().getRotation())
                              .getAngle());
                  return delta < Math.abs(velocity) * Configs.Vision.ScalarRotationTolerance;
                })
            .map(
                est -> {
                  var targets = est.targetsUsed.stream().mapToInt((target) -> target.fiducialId);

                  Logger.recordOutput(
                      "Vision/" + m_camera.getName() + "/lastMeasurement", est.timestampSeconds);

                  Logger.recordOutput("Vision/" + m_camera.getName() + "/est", est.estimatedPose);

                  var targetArray = targets.toArray();

                  // calculate the trust based on the distance of the tag(s) used
                  curStdDevs =
                      PoseCameraIO.getEstimationStdDevs(est.estimatedPose.toPose2d(), targetArray);

                  m_lastSeenTags =
                      est.targetsUsed.stream()
                          .map((target) -> target.fiducialId)
                          .map(PoseCameraIO::getTagPose)
                          .map(Pose3d::toPose2d)
                          .toList();

                  return new VisionData(
                      est.estimatedPose, est.timestampSeconds, curStdDevs, targetArray);
                })
            .toList());
  }

  public Transform3d getOffset() {
    return m_config.offset();
  }

  public List<Pose2d> getLastSeenTags() {

    return m_lastSeenTags;
  }
}
