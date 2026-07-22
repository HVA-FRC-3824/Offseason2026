// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.hardware.motor.MotorIO;
import frc.o2026.Configs;
import frc.o2026.RobotState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private MotorIO m_io;
  private MotorIO m_ioFollower;

  public Intake(MotorIO io, MotorIO ioFollower) {

    m_io = io;
    m_ioFollower = ioFollower;

    m_io.resetEncoder(Degrees.of(0.0));

    m_ioFollower.follow(m_io.getId(), true);
  }

  @Override
  public void periodic() {

    m_io.periodic();
    m_ioFollower.periodic();

    Logger.recordOutput("Intake/d-angle", m_io.getLastReference());
    Logger.recordOutput("Intake/m-angle", m_io.getPos().in(Rotations));

    var pose =
        new Pose3d(
            Inches.of(0.0),
            Inches.of(0.0),
            Meters.of(0.1),
            new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(90.0)));

    pose =
        pose.rotateAround(
            new Translation3d(Inches.of(0.0), Inches.of(0.0), Meters.of(0.1)),
            new Rotation3d(Degrees.of(0.0), m_io.getPos(), Degrees.of(0.0)));

    Logger.recordOutput("Intake/VizPoz", pose);
  }

  public Command stowed() {

    return runOnce(
        () -> {
          RobotState.setSimIntaking(false);
          m_io.setPosition(
              RobotBase.isReal() ? Configs.Intake.IntakeStowedTurns : Degrees.of(90.0));
        });
  }

  public Command deploy() {

    return runOnce(
        () -> {
          RobotState.setSimIntaking(true);
          m_io.setPosition(RobotBase.isReal() ? Configs.Intake.IntakedeployTurns : Degrees.of(0.0));
        });
  }

  public Command alligator() {
    return runOnce(() -> RobotState.setSimIntaking(false))
        .andThen(stowed())
        .andThen(new WaitCommand(Seconds.of(0.4)))
        .andThen(deploy())
        .andThen(new WaitCommand(Seconds.of(0.4)))
        .repeatedly();
  }
}
