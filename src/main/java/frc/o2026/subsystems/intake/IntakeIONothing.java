// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class IntakeIONothing implements IntakeIO {

  public IntakeIONothing() {}

  @Override
  public void setPos(Angle angle) {}

  @Override
  public Angle getPos() {

    return Rotations.of(0.0);
  }
}
