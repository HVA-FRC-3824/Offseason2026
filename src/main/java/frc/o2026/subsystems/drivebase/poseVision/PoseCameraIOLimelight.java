// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.poseVision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.Alliance;
import frc.lib.hardware.vision.VisionConfig;
import frc.lib.hardware.vision.limelight.LimelightHelpers;
import frc.lib.hardware.vision.limelight.LimelightHelpers.PoseEstimate;
import frc.o2026.RobotState;
import frc.o2026.subsystems.drivebase.poseVision.PoseVision.VisionData;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PoseCameraIOLimelight implements PoseCameraIO {

  private final VisionConfig m_config;

  private List<Pose2d> m_lastSeenTags;

  public PoseCameraIOLimelight(VisionConfig config) {

    m_config = config;

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(
        m_config.name(),
        m_config.offset().getX(), // Forward offset (meters)
        m_config.offset().getY(), // Side offset (meters)
        m_config.offset().getZ(), // Height offset (meters)
        m_config.offset().getRotation().getMeasureX().in(Degrees), // Roll (degrees)
        m_config.offset().getRotation().getMeasureY().in(Degrees), // Pitch (degrees)
        m_config.offset().getRotation().getMeasureZ().in(Degrees) // Yaw (degrees)
        );
  }

  @Override
  public ArrayList<VisionData> getMeasurements() {

    var rot = RobotState.getPoseEst().getRotation();
    LimelightHelpers.SetRobotOrientation(
        m_config.name(),
        rot.getMeasureZ().in(Degrees),
        RobotState.getAngularVelocity().in(DegreesPerSecond),
        rot.getMeasureY().in(Degrees),
        0,
        rot.getMeasureX().in(Degrees),
        0);

    PoseEstimate mt2 =
        Alliance.isRed()
            ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(m_config.name())
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_config.name());

    if (mt2 == null) return new ArrayList<>();
    if (mt2.pose.getX() < 1e-4
        && mt2.pose.getRotation().getRadians() < 1e-4
        && mt2.pose.getY() < 1e-4) return new ArrayList<>();

    Logger.recordOutput("m-llEst", mt2.pose);

    List<Integer> tags =
        List.of(mt2.rawFiducials).stream().mapToInt((tag) -> tag.id).boxed().toList();

    m_lastSeenTags = tags.stream().map(PoseCameraIO::getTagPose).map(Pose3d::toPose2d).toList();

    var tagArr = tags.stream().mapToInt(x -> x).toArray();
    var measurements = new ArrayList<VisionData>(1);
    measurements.add(
        new VisionData(
            new Pose3d(mt2.pose),
            mt2.timestampSeconds,
            PoseCameraIO.getEstimationStdDevs(mt2.pose, tagArr),
            tagArr));

    return measurements;
  }

  @Override
  public Transform3d getOffset() {

    return m_config.offset();
  }

  @Override
  public List<Pose2d> getLastSeenTags() {

    return m_lastSeenTags;
  }
}
