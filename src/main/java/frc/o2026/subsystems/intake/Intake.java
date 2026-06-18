// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.o2026.Constants;
import org.ironmaple.simulation.IntakeSimulation;

public class Intake extends SubsystemBase {

  private Angle m_desiredAngle;

  private IntakeIO m_io;

  public Intake(IntakeIO io) {

    m_io = io;
  }

  public Command starting() {
    return runOnce(() -> m_desiredAngle = Rotations.of(9999.9));
  }

  public Command stowed() {

    return runOnce(
        () -> {
          m_io.setPos(Constants.Intake.IntakeStowedTurns);
          m_desiredAngle = Constants.Intake.IntakeStowedTurns;
        });
  }

  public Command deployed() {

    return runOnce(
        () -> {
          m_io.setPos(Constants.Intake.IntakeDeployedTurns);
          m_desiredAngle = Constants.Intake.IntakeDeployedTurns;
        });
  }

  public Command alligator() {
    return stowed()
        .andThen(new WaitCommand(Seconds.of(0.4)))
        .andThen(deployed())
        .andThen(new WaitCommand(Seconds.of(0.4)))
        .repeatedly();
  }

  public IntakeSimulation getSimIntake() {
    return null;
  }
}
