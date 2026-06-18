// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Constants;

public class Flywheel extends SubsystemBase {
  private AngularVelocity m_lastInput = RotationsPerSecond.of(0.0);

  private int m_offsetIndex = 0;

  FlywheelIO m_io;

  SensorData m_data = new SensorData(Inches.of(0.0));

  public Flywheel(FlywheelIO io) {

    m_io = io;
  }

  public Command off() {

    return runOnce(
        () -> {
          m_io.stopFlywheel();
          m_lastInput = RotationsPerSecond.of(0.0);
        });
  }

  public Command manual(AngularVelocity velocity) {

    return runOnce(
        () -> {
          m_io.setFlywheel(velocity);
          m_lastInput = velocity;
        });
  }

  public Command set(Setpoints setpoint) {

    return runOnce(
        () -> {
          m_io.setFlywheel(setpoint.m_velocity);
          m_lastInput = setpoint.m_velocity;
        });
  }

  protected boolean isSpunUp() {
    return getReference().isNear(m_io.getMeasured(), Constants.Flywheel.SpunUpTolerance)
        && getReference().in(RotationsPerSecond) != 0.0;
  }

  protected AngularVelocity getReference() {
    return m_lastInput;
  }

  public record SensorData(Distance distFromTarget) {}

  public enum Setpoints {
    Backwards(Constants.Flywheel.CloseSpeed.times(-1.0)),
    Low(Constants.Flywheel.CloseSpeed),
    Mid(Constants.Flywheel.MiddleSpeed),
    Neutral(Constants.Flywheel.NeutralPassSpeed),
    Field(Constants.Flywheel.FieldPassSpeed);

    AngularVelocity m_velocity;

    private Setpoints(AngularVelocity vel) {
      m_velocity = vel;
    }
  }
}
