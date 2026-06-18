// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class RollerIONothing implements RollerIO {
  public RollerIONothing() {}

  @Override
  public void setRoller(AngularVelocity velocity) {}

  @Override
  public void brakeRoller() {}

  @Override
  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(0.0);
  }
}
