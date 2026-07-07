// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public enum ControlMode {
  Manual,
  Teleop,
  Auto;

  private static ControlMode m_inst = Manual;

  private ControlMode() {}

  public static ControlMode getMode() {

    return m_inst;
  }

  public boolean isMode() {

    return m_inst.equals(this);
  }

  public Trigger getTrigger() {

    return new Trigger(this::isMode);
  }

  public void changeMode() {
    m_inst = this;
    Logger.recordOutput("ControlMode", this.name());
  }
}
