// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.poseVision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.Alliance;
import frc.lib.hardware.vision.limelight.LimelightHelpers;
import frc.lib.hardware.vision.limelight.LimelightHelpers.PoseEstimate;
import frc.o2026.subsystems.drivebase.poseVision.PoseVision.VisionData;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Abstract base class for Limelight vision IO layers. Contains all shared NetworkTables reads, pose
 * filtering, and MegaTag2 logic.
 */
public class PoseCameraIOLimelight implements PoseCameraIO {

  private final String m_name;
  private final Supplier<Rotation3d> m_gyroSupplier;
  private final Supplier<AngularVelocity> m_gyroYawRateSupplier;

  private Transform3d m_offset;

  /**
   * Creates a new Limelight hardware layer.
   *
   * @param name the NetworkTables name of the Limelight
   * @param gyroAngleSupplier supplies the robot's gyro angle in degrees
   * @param gyroAngleRateSupplier supplies the robot's gyro angular rate in degrees per second
   * @param acceptMeasurements whether to process pose estimates from this Limelight
   */
  public PoseCameraIOLimelight(
      String name,
      Transform3d offset,
      Supplier<Rotation3d> gyroSupplier,
      Supplier<AngularVelocity> gyroYawRateSupplier) {

    m_name = name;
    m_offset = offset;
    m_gyroSupplier = gyroSupplier;
    m_gyroYawRateSupplier = gyroYawRateSupplier;

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        m_offset.getX(), // Forward offset (meters)
        m_offset.getY(), // Side offset (meters)
        m_offset.getZ(), // Height offset (meters)
        m_offset.getRotation().getX(), // Roll (degrees)
        m_offset.getRotation().getY(), // Pitch (degrees)
        m_offset.getRotation().getZ() // Yaw (degrees)
        );
  }

  @Override
  public ArrayList<VisionData> getMeasurements() {
    var rot = m_gyroSupplier.get();
    LimelightHelpers.SetRobotOrientation(
        m_name,
        rot.getMeasureZ().in(Degrees),
        m_gyroYawRateSupplier.get().in(DegreesPerSecond),
        rot.getMeasureY().in(Degrees),
        0,
        rot.getMeasureX().in(Degrees),
        0);

    PoseEstimate mt2 =
        Alliance.isRed()
            ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(m_name)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_name);

    int[] tags = List.of(mt2.rawFiducials).stream().mapToInt((tag) -> tag.id).toArray();

    var measurements = new ArrayList<VisionData>(1);
    measurements.add(
        new VisionData(
            new Pose3d(mt2.pose),
            mt2.timestampSeconds,
            PoseCameraIO.getEstimationStdDevs(mt2.pose, tags),
            tags));

    Logger.recordOutput("m-llEst", mt2.pose);

    return measurements;
  }

  @Override
  public Transform3d getOffset() {

    return m_offset;
  }
}
