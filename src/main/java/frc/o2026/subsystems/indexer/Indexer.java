// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Constants;
import frc.o2026.RobotState;

public class Indexer extends SubsystemBase {
  IndexerIO m_io;

  public Indexer(IndexerIO io) {

    m_io = io;
  }

  private AngularVelocity m_desiredKicker = RotationsPerSecond.of(0.0);
  private AngularVelocity m_desiredIndexer = RotationsPerSecond.of(0.0);

  public Command off() {

    return runOnce(
        () -> {
          RobotState.getInstance().setSimIndexing(false);
          m_io.brakeMotors();
          m_desiredIndexer = RotationsPerSecond.of(0.0);
          m_desiredKicker = RotationsPerSecond.of(0.0);
        });
  }

  public Command on() {

    return runOnce(
        () -> {
          RobotState.getInstance().setSimIndexing(true);
          m_io.setBelts(Constants.Indexer.BeltTurnsPerSec);
          m_io.setKicker(Constants.Indexer.KickerWheelTurnsPerSec);
          m_desiredIndexer = Constants.Indexer.BeltTurnsPerSec;
          m_desiredKicker = Constants.Indexer.KickerWheelTurnsPerSec;
        });
  }

  public Command backwards() {

    return runOnce(
        () -> {
          RobotState.getInstance().setSimIndexing(true);
          m_io.setBelts(Constants.Indexer.BeltTurnsPerSec.times(-1.0));
          m_io.setKicker(Constants.Indexer.KickerWheelTurnsPerSec.times(-1.0));
          m_desiredIndexer = Constants.Indexer.BeltTurnsPerSec.times(-1.0);
          m_desiredKicker = Constants.Indexer.KickerWheelTurnsPerSec.times(-1.0);
        });
  }
}
