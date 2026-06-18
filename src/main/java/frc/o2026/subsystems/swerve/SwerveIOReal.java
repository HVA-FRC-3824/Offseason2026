// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Constants;

/// @brief Chassis subsystem for swerve drive control
///
///       Red                      <----- Zero Angle                       Blue
///                            <--- 0 degrees    180 degrees ---->     X  <-----
///   ---  +-------------------------------------------------------------------+ (0, 0)
///    ^   |                7  6              |             17 28           29 |
///    |   |                                  |                             30 |  |
///    |   |                                  |                                |  |
///    |   |                                  |                                |  V
///    |   |                                  |                                |
///    |   |                8  5              |             18 27              |  Y
/// 8.07 m | 16          9       4            |          19       26        31 |
///    |   | 15         10       3            |          20       25        32 |
///    |   |               11  2              |             21 24              |
///    |   |                                  |                                |
///    |   |                                  |                                |
///    |   | 14                               |                                |
///    V   | 13            12  1              |             22 23              |
///   ---  +-------------------------------------------------------------------+
///        |<----------------------------- 16.56 m --------------------------->|
///                                       Top View
public class SwerveIOReal extends SubsystemBase implements SwerveIO {

  // Swerve module order for kinematics calculations
  //
  //         Front          Translation2d Coordinates
  //   FL +----------+ FR              ^ X
  //      | 0      1 |                 |
  //      |          |            Y    |
  //      |          |          <------+-------
  //      | 2      3 |                 |
  //   BL +----------+ BR              |

  private SwerveModule m_frSwerveModules =
      new SwerveModule(
          Constants.CanIds.FrontRightDriveId,
          Constants.CanIds.FrontRightTurnId,
          Constants.CanIds.FrontRightEncoderId);
  private SwerveModule m_flSwerveModules =
      new SwerveModule(
          Constants.CanIds.FrontLeftDriveId,
          Constants.CanIds.FrontLeftTurnId,
          Constants.CanIds.FrontLeftEncoderId);
  private SwerveModule m_brSwerveModules =
      new SwerveModule(
          Constants.CanIds.BackRightDriveId,
          Constants.CanIds.BackRightTurnId,
          Constants.CanIds.BackRightEncoderId);
  private SwerveModule m_blSwerveModules =
      new SwerveModule(
          Constants.CanIds.BackLeftDriveId,
          Constants.CanIds.BackLeftTurnId,
          Constants.CanIds.BackLeftEncoderId);

  // private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
  //     Constants.Chassis.Kinematics,         // Kinematics object
  //     new Rotation2d(0),  // Initial gyro angle
  //     new SwerveModulePosition[]{new SwerveModulePosition(0, new Rotation2d(0)), new
  // SwerveModulePosition(0, new Rotation2d(0)), new SwerveModulePosition(0, new Rotation2d(0)), new
  // SwerveModulePosition(0, new Rotation2d(0))},     // Initial module positions
  //     new Pose2d()              // Initial pose, will be overriden by vision
  // );

  ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds(0, 0, 0);

  boolean m_xMode = false;

  public SwerveIOReal() {
    resetWheelAnglesToZero();
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {

    if (m_xMode) {
      // Set the module states to x mode
      setModuleStates((SwerveModuleState[]) Constants.Chassis.XishStates.toArray());

      // Save the desired speeds for logging later
      return;
    }

    // Save the desired states for use and logging later
    SwerveModuleState[] desiredStates = Constants.Chassis.Kinematics.toSwerveModuleStates(speeds);

    // Set the desired state for each swerve module
    setModuleStates(desiredStates);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    // Set the desired state for each swerve module
    m_flSwerveModules.setDesiredState(states[0]);
    m_frSwerveModules.setDesiredState(states[1]);
    m_blSwerveModules.setDesiredState(states[2]);
    m_brSwerveModules.setDesiredState(states[3]);
  }

  public void resetWheelAnglesToZero() {
    // Set the swerve wheel angles to zero
    m_flSwerveModules.setWheelAngleToForward(Constants.Chassis.FrontLeftForwardsAngle.in(Degrees));
    m_frSwerveModules.setWheelAngleToForward(Constants.Chassis.FrontRightForwardsAngle.in(Degrees));
    m_blSwerveModules.setWheelAngleToForward(Constants.Chassis.BackLeftForwardsAngle.in(Degrees));
    m_brSwerveModules.setWheelAngleToForward(Constants.Chassis.BackRightForwardsAngle.in(Degrees));
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
      m_flSwerveModules.getState(),
      m_frSwerveModules.getState(),
      m_blSwerveModules.getState(),
      m_brSwerveModules.getState()
    };

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      m_flSwerveModules.getPosition(),
      m_frSwerveModules.getPosition(),
      m_blSwerveModules.getPosition(),
      m_brSwerveModules.getPosition()
    };

    return positions;
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.Chassis.Kinematics.toChassisSpeeds(getModuleStates());
  }

  public boolean getIsXMode() {

    return m_xMode;
  }

  public void setIsXMode(boolean xMode) {

    m_xMode = xMode;
  }
}
;
