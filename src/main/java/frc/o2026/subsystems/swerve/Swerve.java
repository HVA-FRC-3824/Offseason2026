// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alliance;
import frc.o2026.RobotState;
import java.util.function.Supplier;

public class Swerve extends SubsystemBase {

  private SwerveIO m_io;

  private boolean m_fieldCentricity = true;

  public Swerve(SwerveIO io) {

    m_io = io;

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    // Configure the AutoBuilder
    AutoBuilder.configure(
        () -> RobotState.getInstance().getPose().toPose2d(),
        (pose) -> RobotState.getInstance().resetPose(new Pose3d(pose)),
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

  public Command resetSwerveModules() {
    return runOnce(() -> m_io.resetWheelAnglesToZero()).withName("resetSwerveModules");
  }

  public Command drive(Supplier<ChassisSpeeds> speedsSupplier) {

    return run(() -> {
          m_io.driveRobotRelative(
              m_fieldCentricity
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      speedsSupplier.get(),
                      (Alliance.isRed()
                          ? RobotState.getInstance().getHeading().unaryMinus()
                          : RobotState.getInstance().getHeading()) // Flip if red
                      )
                  : speedsSupplier.get());
        })
        .withName("Drive");
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
