// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Configs;
import frc.o2026.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {
  MotorIO m_motor;
  MotorIO m_motorFollower;

  public FlywheelIOTalonFX() {
    m_motor = new TalonIO(Constants.CanIds.FlywheelMotorId, Configs.Flywheel.Config);
    m_motorFollower =
        new TalonIO(Constants.CanIds.FlywheelFollowerMotorId, Configs.Flywheel.Config);

    m_motorFollower.follow(Constants.CanIds.FlywheelMotorId, true);
  }

  @Override
  public void setFlywheel(AngularVelocity velocity) {
    m_motor.setVelocity(velocity);
  }

  @Override
  public void stopFlywheel() {
    m_motor.brake();
  }

  @Override
  public AngularVelocity getMeasured() {
    return m_motor.getVelocity();
  }
}
