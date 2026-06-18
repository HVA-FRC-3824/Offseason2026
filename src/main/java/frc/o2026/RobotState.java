// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.o2026.subsystems.vision.Vision.VisionData;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class RobotState {

  private Supplier<Rotation3d> gyroGetter;
  private Supplier<OdomData> swerveGetter;

  @Setter @Getter Pose2d simActualPose;

  private SwerveDrivePoseEstimator3d m_estimator =
      new SwerveDrivePoseEstimator3d(
          Constants.Chassis.Kinematics,
          new Rotation3d(), // Initial gyro angle
          new SwerveModulePosition[] { // Initial module positions
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0)),
            new SwerveModulePosition(0, new Rotation2d(0))
          },
          new Pose3d(14.0, 7.0, 0.0, new Rotation3d()) // Initial pose
          );

  private static RobotState m_instance;

  public static RobotState getInstance() {

    if (m_instance.equals(null)) m_instance = new RobotState();
    return m_instance;
  }

  public void updatePose() {

    m_estimator.update(gyroGetter.get(), swerveGetter.get().modulePoses());
  }

  public Pose3d getPose() {

    return m_estimator.getEstimatedPosition();
  }

  public Rotation2d getHeading() {

    return getPose().getRotation().toRotation2d();
  }

  public void resetPose(Pose3d pose) {

    m_estimator.resetPose(pose);
  }

  public void addVisionData(VisionData data) {

    m_estimator.addVisionMeasurement(
        data.visionMeasurement(), data.timestampSeconds(), data.stdDevs());
  }

  public static record OdomData(
      SwerveModulePosition[] modulePoses, SwerveModuleState[] moduleStates) {}
}
