// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Constants;

public class IndexerSim implements IndexerIO {
  public MotorIO m_beltMotor;
  public MotorIO m_kickerMotor;

  public DCMotorSim m_indexerModel;
  public DCMotorSim m_kickerModel;

  public IndexerSim() {

    m_beltMotor =
        new TalonIO(
            Constants.CanIds.IndexerMotorId, Constants.Indexer.BeltConfig, true); // is an X60
    m_kickerMotor =
        new TalonIO(Constants.CanIds.KickerMotorId, Constants.Indexer.KickerConfig, false);

    m_indexerModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.021, 1.0 // MOI from CAD
                ),
            DCMotor.getKrakenX60(1));

    m_kickerModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1), 0.021, 1.0 // MOI from CAD
                ),
            DCMotor.getKrakenX44(1));
  }

  @Override
  public void setBelts(AngularVelocity indexerVelocity) {

    m_beltMotor.setVelocity(indexerVelocity);
    m_indexerModel.setInputVoltage(m_beltMotor.getAppliedVoltage().in(Volts));
    m_indexerModel.update(0.02);
    m_beltMotor.simPeriodic(m_indexerModel.getAngularVelocity());
  }

  @Override
  public void setKicker(AngularVelocity kickerVelocity) {

    m_kickerMotor.setVelocity(kickerVelocity);
    m_kickerModel.setInputVoltage(m_kickerMotor.getAppliedVoltage().in(Volts));
    m_kickerModel.update(0.02);
    m_kickerMotor.simPeriodic(m_kickerModel.getAngularVelocity());
  }

  @Override
  public void brakeMotors() {
    setBelts(RotationsPerSecond.of(0.0));
    setKicker(RotationsPerSecond.of(0.0));
  }

  @Override
  public AngularVelocity getIndexerVelocity() {
    return m_beltMotor.getVelocity();
  }

  @Override
  public AngularVelocity getKickerVelocity() {
    return m_kickerMotor.getVelocity();
  }
}
