// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.roller;

import frc.lib.hardware.motor.ctre.TalonIO;
import frc.o2026.Configs;
import frc.o2026.Constants;

public class RollerIOTalonFX extends TalonIO implements RollerIO {

  public RollerIOTalonFX() {

    super(Constants.CanIds.FuelIntakeMotorId, Configs.Roller.RollerConfig);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
