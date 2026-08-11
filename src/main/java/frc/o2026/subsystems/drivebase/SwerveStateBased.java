// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import frc.o2026.subsystems.drivebase.objectVision.ObjectCameraIO;
import frc.o2026.subsystems.drivebase.objectVision.ObjectVision;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.shared.Util;

import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SwerveStateBased extends SubsystemBase {

  private SwerveIO m_io;
  private ObjectVision m_objectDetection;

  private boolean m_fieldCentricity = true;

  FollowPath.Builder m_pathBuilder;

  private PIDController m_xController = new PIDController(2, 0, 0);
  private PIDController m_yController = new PIDController(2, 0, 0);
  private PIDController m_rotController = new PIDController(4, 0.0, 0.5);

  public SwerveStateBased(SwerveIO io, ObjectCameraIO odIo) {

    m_io = io;
    m_objectDetection = new ObjectVision(odIo);

    m_rotController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotController.setTolerance(Units.degreesToRadians(5.0));
    m_xController.setTolerance(0.01);
    m_xController.disableContinuousInput();
    m_yController.setTolerance(0.01);
    m_yController.disableContinuousInput();

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      e.printStackTrace();
      e.printStackTrace();
      return;
    }

    // Configure the AutoBuilder
    AutoBuilder.configure(
        () -> m_io.getPose(),
        (pose) -> m_io.resetPose(pose),
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        () -> m_io.getSpeeds(),
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        (speeds, feedforwards) -> m_io.driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0),
            // Rotation PID constants
            new PIDConstants(1.0, 0.0, 0.0)),
        config,
        Util::isRed,
        this // Subsystem req
        );

    m_pathBuilder =
        new FollowPath.Builder(
                this,
                m_io::getPose,
                m_io::getSpeeds,
                m_io::driveRobotRelative,
                new PIDController(2.0, 0.3, 1.5),
                new PIDController(4.0, 0.2, 0.1),
                new PIDController(0.2, 0.0, 0.0))
            .withTRatioBasedTranslationHandoffs(true)
            .withShouldFlip(Util::isRed);
  }

  public ChassisSpeeds getChassisSpeeds() {

    return Constants.Chassis.Kinematics.toChassisSpeeds(m_io.getModuleStates());
  }

  public Pose3d getPose() {

    return new Pose3d(m_io.getPose());
  }

  public Rotation2d getHeading() {

    return m_io.getGyroHeading();
  }

  public static enum SwerveDesiredState {
    driveField,
    driveRobot,
    driveDefault,
    aim,
    aimSOTM,
    intakeAssist,
    hardStop;

    public ChassisSpeeds speeds = new ChassisSpeeds();
    public Rotation2d rotationTarget = new Rotation2d();
    public Pose2d poseTarget = new Pose2d();

    public SwerveDesiredState with(ChassisSpeeds speeds) {
      this.speeds = speeds;
      return this;
    }

    public SwerveDesiredState with(Rotation2d rotationTarget) {
      this.rotationTarget = rotationTarget;
      return this;
    }

    public SwerveDesiredState with(Pose2d poseTarget) {
      this.poseTarget = poseTarget;
      return this;
    }
  }

  private SwerveDesiredState m_desiredState;

  @Override
  public void periodic() {

    m_io.periodic();

    switch (m_desiredState) {
      case driveField:
        drive(m_desiredState.speeds, true);
        break;

      case driveRobot:
        drive(m_desiredState.speeds, false);
        break;

      case driveDefault:
        drive(m_desiredState.speeds, m_fieldCentricity);
        break;

      case aim:
        drive(
            new ChassisSpeeds(
                m_desiredState.speeds.vxMetersPerSecond,
                m_desiredState.speeds.vyMetersPerSecond,
                m_rotController.calculate(
                    m_io.getGyroHeading().getRadians(),
                    m_desiredState.rotationTarget.getRadians())),
            false);
        break;

      case aimSOTM:
        drive(
            new ChassisSpeeds(
                m_desiredState.speeds.vxMetersPerSecond,
                m_desiredState.speeds.vyMetersPerSecond,
                m_rotController.calculate(
                    m_io.getGyroHeading().getRadians(),
                    RobotState.getSOTMRotTarget().getRadians())),
            m_fieldCentricity);
        break;

      case intakeAssist:
        drive(
            new ChassisSpeeds(
                Configs.Chassis.IntakeSpeed.in(MetersPerSecond),
                0.0,
                m_rotController.calculate(
                    m_io.getGyroHeading().getRadians(),
                    RobotState.getSOTMRotTarget().getRadians())),
            false);
        break;

      case hardStop:
        m_io.setModuleStates(Constants.Chassis.XishStates.toArray(SwerveModuleState[]::new));
        break;

      default:
        break;
    }

    RobotState.setLastMeasuredSpeeds(getChassisSpeeds());
    RobotState.setPoseEst(getPose());

    Logger.recordOutput("Swerve/fieldCentric", m_fieldCentricity);
    Logger.recordOutput("Swerve/m-speeds", getChassisSpeeds());
    Logger.recordOutput("Swerve/m-states", m_io.getModuleStates());
    Logger.recordOutput("Swerve/m-speeds", getChassisSpeeds());
    Logger.recordOutput("Swerve/m-pose", getPose());
    Logger.recordOutput("Swerve/m-heading", getHeading().getDegrees());
    Logger.recordOutput("Swerve/m-aimed", isAimed());

    Logger.recordOutput("Swerve/d-aimed", Radians.of(m_rotController.getSetpoint()).in(Degrees));
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {

    var desiredStates =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, (Util.isRed() ? getHeading() : getHeading().plus(Rotation2d.k180deg)))
            : speeds;

    m_io.driveRobotRelative(desiredStates);

    Logger.recordOutput("d-speeds", speeds);
    Logger.recordOutput(
        "d-states", Constants.Chassis.Kinematics.toSwerveModuleStates(desiredStates));
  }

  public void resetPose(Pose2d pose) {

    m_io.resetPose(pose);
  }

  public Command resetPoseCmd(Pose2d pose) {

    return runOnce(() -> resetPose(pose));
  }

  public Command ppPathPose(Pose2d pose) {
    // Build and return the command
    return AutoBuilder.pathfindToPoseFlipped(
        pose, Configs.Chassis.constraints, 0.0 // Goal end velocity in m/s
        );
  }

  public Command ppPathPoses(List<Pose2d> poses) {

    return defer(
        () -> {
          var speed =
              Math.hypot(
                  getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);

          // Build and return the command
          return AutoBuilder.pathfindThenFollowPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(poses),
                  Configs.Chassis.constraints,
                  new IdealStartingState(speed, getHeading()),
                  new GoalEndState(0.0, poses.get(poses.size() - 1).getRotation())),
              Configs.Chassis.constraints // Goal end velocity in m/s
              );
        });
  }

  public Command bLinePathPose(Pose2d pose) {

    return bLinePathPoses(List.of(pose));
  }

  public Command bLinePathPoses(List<Pose2d> poses) {

    return defer(
        () -> {
          return m_pathBuilder.build(
              new Path(
                  poses.stream().map(Path.Waypoint::new).map(pose -> (PathElement) pose).toList()));
        });
  }

  public Command crossTrench() {

    return defer(
        () -> {
          var translation = getPose().getTranslation().toTranslation2d();

          var pair =
              Constants.Field.BlueTrenchEntrances.getFirst()
                          .getFirst()
                          .plus(Constants.Field.BlueTrenchEntrances.getFirst().getSecond())
                          .div(2.0)
                          .getDistance(translation)
                      < Constants.Field.BlueTrenchEntrances.getSecond()
                          .getFirst()
                          .plus(Constants.Field.BlueTrenchEntrances.getSecond().getSecond())
                          .div(2.0)
                          .getDistance(translation)
                  ? Constants.Field.BlueTrenchEntrances.getFirst()
                  : Constants.Field.BlueTrenchEntrances.getSecond();

          Translation2d entry;
          Translation2d exit;

          if (pair.getFirst().getDistance(translation)
              < pair.getSecond().getDistance(translation)) {
            entry = pair.getFirst();
            exit = pair.getSecond();
          } else {
            entry = pair.getSecond();
            exit = pair.getFirst();
          }

          Rotation2d nearestCrossRot =
              getHeading().getDegrees() > 0
                  ? (getHeading().getDegrees() > 135 ? Rotation2d.k180deg : Rotation2d.kCCW_90deg)
                  : (getHeading().getDegrees() > -45 ? Rotation2d.kZero : Rotation2d.kCW_90deg);

          return bLinePathPoses(
              List.of(new Pose2d(entry, nearestCrossRot), new Pose2d(exit, nearestCrossRot)));
        });
  }

  public boolean isAtPose() {

    return m_xController.atSetpoint() && m_yController.atSetpoint();
  }

  public boolean isAimed() {

    return m_rotController.atSetpoint();
  }

  public boolean hasObjects() {

    return m_objectDetection.hasObjects();
  }

  public Command resetGyro() {

    return runOnce(
        () -> {
          var pose =
              new Pose2d(
                  m_io.getPose().getMeasureX(), m_io.getPose().getMeasureY(), new Rotation2d());
          m_io.resetGyro();
          m_io.resetPose(pose);
        });
  }

  public Command fieldCentricityOn() {
    return runOnce(() -> m_fieldCentricity = true).withName("Field Centricity On");
  }

  public Command toggleFieldCentricity() {
    return runOnce(() -> m_fieldCentricity = !m_fieldCentricity)
        .withName("Field Centricity Toggle");
  }

  public Command fieldCentricityOff() {
    return runOnce(() -> m_fieldCentricity = false).withName("Field Centricity Off");
  }
}
