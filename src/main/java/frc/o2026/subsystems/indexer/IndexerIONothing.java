// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerIONothing implements IndexerIO {

  public IndexerIONothing() {}

  @Override
  public void setBelts(AngularVelocity indexerVelocity) {}

  @Override
  public void setKicker(AngularVelocity kickerVelocity) {}

  @Override
  public void brakeMotors() {}

  @Override
  public AngularVelocity getIndexerVelocity() {
    return RotationsPerSecond.of(0.0);
  }

  @Override
  public AngularVelocity getKickerVelocity() {
    return RotationsPerSecond.of(0.0);
  }
}
