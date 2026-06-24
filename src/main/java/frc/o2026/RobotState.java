// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;
import lombok.Setter;

public class RobotState {

  private static RobotState m_instance;

  public static RobotState getInstance() {

    if (m_instance == null) m_instance = new RobotState();
    return m_instance;
  }

  public void decrementFuel() {
    simFuelCount--;
  }

  public void incrementFuel() {
    simFuelCount++;
  }

  @Getter private int simFuelCount = 0;

  @Setter @Getter private Rotation2d SOTMRotTarget = new Rotation2d();
  @Setter @Getter private Pose3d poseEst = new Pose3d();
  @Setter @Getter private ChassisSpeeds lastMeasuredSpeeds = new ChassisSpeeds();
  @Setter @Getter private boolean isSimIntaking = false;
}
