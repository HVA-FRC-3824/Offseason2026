// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Constants;

public class IndexerTalonFX implements IndexerIO {
  public MotorIO m_beltMotor;
  public MotorIO m_kickerMotor;

  public IndexerTalonFX() {

    m_beltMotor = new TalonIO(Constants.CanIds.IndexerMotorId, Constants.Indexer.BeltConfig);
    m_kickerMotor = new TalonIO(Constants.CanIds.KickerMotorId, Constants.Indexer.KickerConfig);
  }

  @Override
  public void setBelts(AngularVelocity indexerVelocity) {

    m_beltMotor.setVelocity(indexerVelocity);
  }

  @Override
  public void setKicker(AngularVelocity kickerVelocity) {

    m_kickerMotor.setVelocity(kickerVelocity);
  }

  @Override
  public void brakeMotors() {
    m_beltMotor.brake();
    m_kickerMotor.brake();
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
