// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alliance;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {

  private SwerveIO m_io;

  private boolean m_fieldCentricity = true;

  private PIDController m_aimController = new PIDController(2, 0, 0);

  public Swerve(SwerveIO io) {

    m_io = io;

    m_aimController.enableContinuousInput(-Math.PI, Math.PI);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
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
  }

  @Override
  public void periodic() {

    m_io.periodic();

    RobotState.getInstance().setLastMeasuredSpeeds(getChassisSpeeds());
    RobotState.getInstance().setPoseEst(getPose());

    Logger.recordOutput("m-pose", getPose());
    Logger.recordOutput("m-heading", getHeading());
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
  }

  private void driveFieldRelative(ChassisSpeeds speeds) {

    m_io.driveRobotRelative(
        m_fieldCentricity
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, (Alliance.isRed() ? getHeading() : getHeading().plus(Rotation2d.k180deg)))
            : speeds);
  }

  public Command resetSwerveModules() {
    return runOnce(() -> m_io.resetWheelAnglesToZero()).withName("resetSwerveModules");
  }

  public Command drive(Supplier<ChassisSpeeds> speedsSupplier) {

    return run(() -> {
          driveFieldRelative(speedsSupplier.get());
        })
        .withName("Drive");
  }

  public Command aimSOTM(Supplier<ChassisSpeeds> speedsSupplier) {

    return run(() -> {
          var speeds = speedsSupplier.get();
          speeds.omegaRadiansPerSecond =
              m_aimController.calculate(
                  getHeading().getRadians(),
                  RobotState.getInstance().getSOTMRotTarget().getRadians());

          driveFieldRelative(speeds);
        })
        .withName("AimSOTM");
  }

  public Command fieldCentricityOn() {
    return runOnce(() -> m_fieldCentricity = true).withName("Field Centricity On");
  }

  public Command fieldCentricityOff() {
    return runOnce(() -> m_fieldCentricity = false).withName("Field Centricity Off");
  }

  public Command xModeOn() {
    return runOnce(() -> m_io.setIsXMode(true)).withName("XMode ON!!!");
  }

  public Command xModeOff() {
    return runOnce(() -> m_io.setIsXMode(false)).withName("XMode off :(");
  }
}
