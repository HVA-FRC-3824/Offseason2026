// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.o2026.Constants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class SwerveIOSim implements SwerveIO {

  // Swerve module order for kinematics calculations
  //
  //         Front          Translation2d Coordinates
  //   FL +----------+ FR              ^ X
  //      | 0      1 |                 |
  //      |          |            Y    |
  //      |          |          <------+-------
  //      | 2      3 |                 |
  //   BL +----------+ BR              |

  // Create and configure a drivetrain simulation configuration
  private static DriveTrainSimulationConfig driveTrainSimulationConfig =
      DriveTrainSimulationConfig.Default()
          // Specify gyro type (for realistic gyro drifting and error simulation)
          .withGyro(COTS.ofPigeon2())
          // Specify swerve module (for realistic swerve dynamics)
          .withSwerveModule(
              COTS.ofMark4(
                  DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                  DCMotor.getNEO(1), // Steer motor is a Falcon 500
                  COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                  2)) // L3 Gear ratio
          .withTrackLengthTrackWidth(
              Constants.Chassis.WheelBaseMeters, Constants.Chassis.TrackWidthMeters)
          .withBumperSize(
              Constants.Chassis.WheelBaseMeters.plus(Inches.of(4.5).times(2.0)),
              Constants.Chassis.TrackWidthMeters.plus(Inches.of(4.5).times(2.0)));

  private static SelfControlledSwerveDriveSimulation m_swerveDriveSimulation =
      new SelfControlledSwerveDriveSimulation(
          new SwerveDriveSimulation(
              driveTrainSimulationConfig, new Pose2d(0, 0, new Rotation2d())));

  public static Rotation2d getGyroYaw() {
    return m_swerveDriveSimulation.getOdometryEstimatedPose().getRotation();
  }

  public static Pose2d getPose() {
    return m_swerveDriveSimulation.getOdometryEstimatedPose();
  }

  public static void setPose(Pose2d pose) {
    m_swerveDriveSimulation.setSimulationWorldPose(pose);
    m_swerveDriveSimulation.resetOdometry(pose);
  }

  public static void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    m_swerveDriveSimulation.addVisionEstimation(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds(0, 0, 0);

  boolean m_xMode = false;

  public SwerveIOSim() {
    SimulatedArena.getInstance()
        .addDriveTrainSimulation(m_swerveDriveSimulation.getDriveTrainSimulation());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {

    // speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, heading);
    // If the chassis is in x mode, than stay in x mode, ignoring the desired speeds
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

    Logger.runEveryN(1, () -> m_swerveDriveSimulation.periodic());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    // Set the desired state for each swerve module
    m_swerveDriveSimulation.runSwerveStates(states);
  }

  public void resetWheelAnglesToZero() {}

  public SwerveModuleState[] getModuleStates() {
    return m_swerveDriveSimulation.getMeasuredStates();
  }

  public SwerveModulePosition[] getModulePositions() {
    return m_swerveDriveSimulation.getLatestModulePositions();
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
