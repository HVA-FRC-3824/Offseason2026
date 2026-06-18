// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.o2026.Constants;

public class GyroPigeon implements GyroIO {
  Pigeon2 m_gyro;

  public GyroPigeon() {

    m_gyro = new Pigeon2(Constants.CanIds.PigeonGyroId);
  }

  @Override
  public Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  @Override
  public AngularVelocity getGyroAngularVelocity() {
    return m_gyro.getAngularVelocityZWorld().getValue();
  }

  @Override
  public void reset() {
    m_gyro.reset();
  }
}
