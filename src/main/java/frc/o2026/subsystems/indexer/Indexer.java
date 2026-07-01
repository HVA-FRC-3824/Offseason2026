// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.hardware.MotorIO;
import frc.o2026.Configs;

public class Indexer extends SubsystemBase {

  private MotorIO m_beltIO;
  private MotorIO m_kickIO;

  public Indexer(MotorIO beltIO, MotorIO kickIO) {

    m_beltIO = beltIO;
    m_kickIO = kickIO;
  }

  public Command off() {

    return runOnce(
        () -> {
          m_beltIO.brake();
          m_kickIO.brake();
        });
  }

  public Command on() {

    return runOnce(
        () -> {
          m_beltIO.setVelocity(Configs.Indexer.BeltTurnsPerSec);
          m_kickIO.setVelocity(Configs.Indexer.KickerWheelTurnsPerSec);
        });
  }

  public Command backwards() {

    return runOnce(
        () -> {
          m_beltIO.setVelocity(Configs.Indexer.BeltTurnsPerSec.times(-1.0));
          m_kickIO.setVelocity(Configs.Indexer.KickerWheelTurnsPerSec.times(-1.0));
        });
  }
}
