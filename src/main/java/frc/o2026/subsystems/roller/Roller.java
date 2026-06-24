// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Configs;

public class Roller extends SubsystemBase {

  private RollerIO m_io;

  public Roller(RollerIO io) {

    m_io = io;
  }

  public Command off() {
    return runOnce(m_io::brakeRoller);
  }

  public Command on() {
    return runOnce(() -> m_io.setRoller(Configs.Roller.IntakeDriveTurnsPerSec));
  }

  public Command backwards() {
    return runOnce(() -> m_io.setRoller(Configs.Roller.IntakeDriveTurnsPerSec.times(-1.0)));
  }
}
