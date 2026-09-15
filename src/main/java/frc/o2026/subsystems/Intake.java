// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.o2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Configs;
import frc.o2026.RobotState;
import frc.shared.hardware.motor.MotorIO;
import frc.shared.hardware.motor.MotorInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public static enum IntakeDesiredState {
    stowed,
    deployed,
    starting
  }

  @AutoLogOutput(key = "states/intake")
  private IntakeDesiredState m_desiredState = IntakeDesiredState.starting;

  private MotorIO m_io;
  private MotorIO m_ioFollower;

  private MotorInputsAutoLogged m_ioInputs = new MotorInputsAutoLogged();

  public Intake(MotorIO io, MotorIO ioFollower) {

    m_io = io;
    m_ioFollower = ioFollower;

    m_ioFollower.follow(m_io.getId(), true);
  }

  private IntakeDesiredState m_lastState = m_desiredState;

  public Command setState(IntakeDesiredState state) {

    return runOnce(() -> m_desiredState = state).withName(state.toString());
  }

  @Override
  public void periodic() {

    m_io.periodic();
    m_ioFollower.periodic();

    m_io.updateInputs(m_ioInputs);
    Logger.processInputs("Intake", m_ioInputs);

    if (m_desiredState != m_lastState) {
      switch (m_desiredState) {
        case stowed:
          RobotState.setSimIntaking(false);
          m_io.setPosition(
              RobotBase.isReal() ? Configs.Intake.IntakeStowedTurns : Degrees.of(90.0));
          break;

        case deployed:
          RobotState.setSimIntaking(true);
          m_io.setPosition(RobotBase.isReal() ? Configs.Intake.IntakeDeployTurns : Degrees.of(0.0));
          break;

        default:
          break;
      }

      m_lastState = m_desiredState;
    }

    var pose =
        new Pose3d(
            Inches.of(0.0),
            Inches.of(0.0),
            Meters.of(0.1),
            new Rotation3d(Degrees.of(0.0), Degrees.of(0.0), Degrees.of(90.0)));

    pose =
        pose.rotateAround(
            new Translation3d(Inches.of(0.0), Inches.of(0.0), Meters.of(0.1)),
            new Rotation3d(Degrees.of(0.0), m_ioInputs.position, Degrees.of(0.0)));

    Logger.recordOutput("Visualization/Intake", pose);
  }

  public Command resetPosAtBumper() {
    return runOnce(() -> m_io.resetEncoder(Configs.Intake.IntakeDeployTurns));
  }

  public Command resetPosAtStow() {
    return runOnce(() -> m_io.resetEncoder(Configs.Intake.IntakeStowedTurns));
  }
}
