// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {

  private GyroIO m_io;

  public Gyro(GyroIO io) {

    m_io = io;
  }

  public Rotation2d getGyroRotation() {

    return m_io.getGyroRotation();
  }

  public AngularVelocity getGyroAngularVelocity() {

    return m_io.getGyroAngularVelocity();
  }

  public void reset() {

    m_io.reset();
  }
}
