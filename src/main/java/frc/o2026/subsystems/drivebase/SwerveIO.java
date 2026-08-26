// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {

  // Swerve module order for kinematics calculations
  //
  //         Front          Translation2d Coordinates
  //   FL +----------+ FR              ^ X
  //      | 0      1 |                 |
  //      |          |            Y    |
  //      |          |          <------+-------
  //      | 2      3 |                 |
  //   BL +----------+ BR              |

  @AutoLog
  public static class SwerveIOInputs {

    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public Pose3d pose = new Pose3d();
  }

  public default void updateInputs(SwerveIOInputs inputs) {}

  public default void driveRobotRelative(ChassisSpeeds speeds) {}
  public default void setModuleStates(SwerveModuleState[] states) {}
  public default void resetPose(Pose2d newPos) {}
  public default void resetGyro() {}
  public default void periodic() {}
}
