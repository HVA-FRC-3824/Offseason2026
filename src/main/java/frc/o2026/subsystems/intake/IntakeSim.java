// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Constants;

public class IntakeSim implements IntakeIO {
  private TalonIO m_motor;

  private DCMotorSim m_motorModel;

  public IntakeSim() {

    m_motor = new TalonIO(Constants.CanIds.IntakePositionFollowerMotorId, true); // is an X60

    m_motorModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2), 0.210408789, 1.0 // MOI from CAD
                ),
            DCMotor.getKrakenX60(2));
  }

  @Override
  public void setPos(Angle angle) {

    m_motor.setPosition(angle);

    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(RadiansPerSecond.of(m_motorModel.getAngularVelocityRadPerSec()));
  }

  @Override
  public Angle getPos() {
    return m_motor.getPos();
  }
}
