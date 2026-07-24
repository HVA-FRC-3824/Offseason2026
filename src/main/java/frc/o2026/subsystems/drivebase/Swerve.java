// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alliance;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  private SwerveIO m_io;

  private boolean m_fieldCentricity = true;

  FollowPath.Builder m_pathBuilder;

  private PIDController m_xController = new PIDController(2, 0, 0);
  private PIDController m_yController = new PIDController(2, 0, 0);
  private PIDController m_rotController = new PIDController(5, 0.05, 0.03);

  public Swerve(SwerveIO io) {

    m_io = io;

    m_rotController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotController.setTolerance(Units.degreesToRadians(5.0));

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
        Alliance::isRed,
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
            .withTRatioBasedTranslationHandoffs(true);

    m_io.resetWheelAnglesToZero();
  }

  @Override
  public void periodic() {

    m_io.periodic();

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

  public ChassisSpeeds getChassisSpeeds() {

    return Constants.Chassis.Kinematics.toChassisSpeeds(m_io.getModuleStates());
  }

  public Pose3d getPose() {

    return new Pose3d(m_io.getPose());
  }

  public Rotation2d getHeading() {

    return m_io.getGyroHeading();
  }

  public void resetPose(Pose2d pose) {

    m_io.resetPose(pose);

    Logger.recordOutput("lastResetPose", pose);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {

    var desiredStates =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, (Alliance.isRed() ? getHeading() : getHeading().plus(Rotation2d.k180deg)))
            : speeds;

    m_io.driveRobotRelative(desiredStates);

    Logger.recordOutput("d-speeds", speeds);
    Logger.recordOutput(
        "d-states", Constants.Chassis.Kinematics.toSwerveModuleStates(desiredStates));
  }

  public Command resetSwerveModules() {

    return runOnce(() -> m_io.resetWheelAnglesToZero()).withName("resetSwerveModules");
  }

  public Command resetPoseCmd(Pose2d pose) {

    return runOnce(() -> resetPose(pose));
  }

  public Command drive(Supplier<ChassisSpeeds> speedsSupplier, boolean fieldCentric) {

    return run(() -> {
          drive(speedsSupplier.get(), fieldCentric);
        })
        .withName("Drive");
  }

  public Command drive(Supplier<ChassisSpeeds> speedsSupplier) {

    return drive(speedsSupplier, m_fieldCentricity);
  }

  public Command ppPathPose(Pose2d pose) {

    // Build and return the command
    return AutoBuilder.pathfindToPose(
        pose, Configs.Chassis.constraints, 0.0 // Goal end velocity in m/s
        );
  }

  public Command bLinePathPose(Pose2d pose, boolean shouldFlip) {

    return bLinePathPose(List.of(pose), shouldFlip);
  }

  public Command bLinePathPose(List<Pose2d> poses, boolean shouldFlip) {

    return m_pathBuilder
        .withShouldFlip(() -> shouldFlip)
        .build(
            new Path(
                poses.stream().map(Path.Waypoint::new).map(pose -> (PathElement) pose).toList()));
  }

  public Command pidPathPose(Supplier<Pose2d> target) {

    return run(() -> {
          var currPose = getPose().toPose2d();
          var targetPose = target.get();
          drive(
              new ChassisSpeeds(
                  m_xController.calculate(currPose.getX(), targetPose.getX()),
                  m_yController.calculate(currPose.getY(), targetPose.getY()),
                  m_rotController.calculate(
                      currPose.getRotation().getRadians(), targetPose.getRotation().getRadians())),
              true);
        })
        .until(
            () ->
                m_xController.atSetpoint()
                    && m_yController.atSetpoint()
                    && m_rotController.atSetpoint())
        .andThen(runOnce(() -> drive(new ChassisSpeeds(), m_fieldCentricity)));
  }

  public Command aimShoot() {

    return aim(() -> RobotState.getSOTMRotTarget()).withName("AimSOTM");
  }

  public Command aimSOTM(Supplier<ChassisSpeeds> speeds) {

    return aimMove(
            () -> speeds.get().div(8), () -> Optional.of(RobotState.getSOTMRotTarget()), false)
        .withName("AimSOTM");
  }

  public Command aim(Supplier<Rotation2d> angleSupplier) {

    return aimMove(ChassisSpeeds::new, () -> Optional.of(angleSupplier.get()), false)
        .withName("Aim");
  }

  public Command aimMove(
      Supplier<ChassisSpeeds> speedsSupplier,
      Supplier<Optional<Rotation2d>> angleSupplier,
      boolean moveWhenAimed) {

    return aimMove(speedsSupplier, angleSupplier, moveWhenAimed, m_fieldCentricity);
  }

  public Command aimMove(
      Supplier<ChassisSpeeds> speedsSupplier,
      Supplier<Optional<Rotation2d>> angleSupplier,
      boolean moveWhenAimed,
      boolean fieldCentric) {

    return run(() -> {
          var angle = angleSupplier.get();
          drive(
              new ChassisSpeeds(
                  moveWhenAimed
                      ? (isAimed() ? speedsSupplier.get().vxMetersPerSecond : 0.0)
                      : speedsSupplier.get().vxMetersPerSecond,
                  moveWhenAimed
                      ? (isAimed() ? speedsSupplier.get().vyMetersPerSecond : 0.0)
                      : speedsSupplier.get().vyMetersPerSecond,
                  m_rotController.calculate(
                      m_io.getGyroHeading().getRadians(),
                      angle.orElseGet(Rotation2d::new).getRadians())),
              fieldCentric);
        })
        .withName("AimMove");
  }

  public Command targetAssistedDrive(
      Supplier<ChassisSpeeds> speedsSupplier, Supplier<Optional<Rotation2d>> angleSupplier) {

    return run(() -> {
          drive(
              new ChassisSpeeds(
                  speedsSupplier.get().vxMetersPerSecond,
                  speedsSupplier.get().vyMetersPerSecond,
                  speedsSupplier.get().omegaRadiansPerSecond
                      + (angleSupplier.get().isEmpty()
                          ? 0.0
                          : m_rotController.calculate(
                                  m_io.getGyroHeading().getRadians(),
                                  angleSupplier.get().get().getRadians())
                              * Configs.Chassis.AssistPower)),
              m_fieldCentricity);
        })
        .withName("AimAssistCHEATER");
  }

  public Command crossTrench() {

    return defer(
        () -> {
          Translation2d entry;
          Translation2d exit;

          var trajPair =
              Alliance.isRed()
                  ? Constants.Field.RedTrenchEntrances
                  : Constants.Field.BlueTrenchEntrances;

          var translation = getPose().getTranslation().toTranslation2d();

          var pair =
              trajPair
                          .getFirst()
                          .getFirst()
                          .plus(trajPair.getFirst().getSecond())
                          .div(2.0)
                          .getDistance(translation)
                      < trajPair
                          .getSecond()
                          .getFirst()
                          .plus(trajPair.getSecond().getSecond())
                          .div(2.0)
                          .getDistance(translation)
                  ? trajPair.getFirst()
                  : trajPair.getSecond();

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

          // return bLinePathPose(
          //     List.of(new Pose2d(entry, nearestCrossRot), new Pose2d(exit, nearestCrossRot)),
          //     false);

          return ppPathPose(new Pose2d(entry, nearestCrossRot))
              .andThen(ppPathPose(new Pose2d(exit, nearestCrossRot)));
        });
  }

  public boolean isAimed() {

    return m_rotController.atSetpoint();
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

  public Command fieldCentricityToggle() {
    return runOnce(() -> m_fieldCentricity = !m_fieldCentricity)
        .withName("Field Centricity Toggle");
  }

  public Command fieldCentricityOff() {
    return runOnce(() -> m_fieldCentricity = false).withName("Field Centricity Off");
  }

  public Command xModeToggle() {
    return runOnce(() -> m_io.setIsXMode(!m_io.getIsXMode())).withName("XMode ON!!!");
  }

  public Command xModeOn() {
    return runOnce(() -> m_io.setIsXMode(true)).withName("XMode ON!!!");
  }

  public Command xModeOff() {
    return runOnce(() -> m_io.setIsXMode(false)).withName("XMode off :(");
  }
}
