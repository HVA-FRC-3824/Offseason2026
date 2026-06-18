// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlywheelIONothing implements FlywheelIO {

  public FlywheelIONothing() {}

  @Override
  public void setFlywheel(AngularVelocity velocity) {}

  @Override
  public void stopFlywheel() {}

  @Override
  public AngularVelocity getMeasured() {
    return RotationsPerSecond.of(0.0);
  }
}
