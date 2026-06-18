// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;

public class GyroSim implements GyroIO {
  private final Supplier<Rotation2d> m_gyroSim;
  private Rotation2d m_lastMeasurement;

  public GyroSim(Supplier<Rotation2d> gyroSim) {

    m_gyroSim = gyroSim;
    m_lastMeasurement = m_gyroSim.get();
  }

  @Override
  public Rotation2d getGyroRotation() {

    return m_gyroSim.get();
  }

  @Override
  public AngularVelocity getGyroAngularVelocity() {

    AngularVelocity velocity =
        RadiansPerSecond.of(
            m_gyroSim.get().minus(m_lastMeasurement).div(0.02).getMeasure().in(Radians));
    m_lastMeasurement = m_gyroSim.get();
    return velocity;
  }

  @Override
  public void reset() {}
}
