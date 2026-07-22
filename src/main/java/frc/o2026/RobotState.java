// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import lombok.Getter;
import lombok.Setter;

public class RobotState {

  private static RobotState m_instance;

  public static RobotState getInstance() {

    if (m_instance == null) m_instance = new RobotState();
    return m_instance;
  }

  public static void decrementFuel() {
    simFuelCount--;
  }

  public static void incrementFuel() {
    simFuelCount++;
  }

  @Getter private static int simFuelCount = 8; // preloads

  @Setter @Getter private static Rotation2d SOTMRotTarget = new Rotation2d();
  @Setter @Getter private static Pose3d poseEst = new Pose3d();
  @Setter @Getter private static Pose3d simRealPose = new Pose3d();
  @Setter @Getter private static AngularVelocity angularVelocity = DegreesPerSecond.of(0.0);
  @Setter @Getter private static ChassisSpeeds lastMeasuredSpeeds = new ChassisSpeeds();
  @Setter @Getter private static boolean isSimIntaking = false;
  @Setter @Getter private static boolean isSimIndexing = false;
}
