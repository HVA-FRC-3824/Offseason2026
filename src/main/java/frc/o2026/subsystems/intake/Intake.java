// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.hardware.MotorIO;
import frc.o2026.Configs;
import frc.o2026.RobotState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private Angle m_desiredAngle;

  private MotorIO m_io;

  public Intake(MotorIO io) {

    m_io = io;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("d-intake", m_desiredAngle);
    Logger.recordOutput("m-intake", m_io.getPos());
  }

  public Command stowed() {

    return runOnce(
        () -> {
          RobotState.getInstance().setSimIntaking(false);
          m_io.setPosition(Configs.Intake.IntakeStowedTurns);
          m_desiredAngle = Configs.Intake.IntakeStowedTurns;
        });
  }

  public Command deploy() {

    return runOnce(
        () -> {
          RobotState.getInstance().setSimIntaking(true);
          m_io.setPosition(Configs.Intake.IntakedeployTurns);
          m_desiredAngle = Configs.Intake.IntakedeployTurns;
        });
  }

  public Command alligator() {
    return runOnce(() -> RobotState.getInstance().setSimIntaking(false))
        .andThen(stowed())
        .andThen(new WaitCommand(Seconds.of(0.4)))
        .andThen(deploy())
        .andThen(new WaitCommand(Seconds.of(0.4)))
        .repeatedly();
  }
}
