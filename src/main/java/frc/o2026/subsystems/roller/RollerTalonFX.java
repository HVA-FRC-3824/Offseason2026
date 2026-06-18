// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.roller;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Constants;

public class RollerTalonFX implements RollerIO {
  public MotorIO m_motor;

  public RollerTalonFX() {

    m_motor = new TalonIO(Constants.CanIds.FuelIntakeMotorId, Constants.Roller.RollerConfig);
  }

  @Override
  public void setRoller(AngularVelocity velocity) {
    m_motor.setVelocity(velocity);
  }

  @Override
  public void brakeRoller() {
    m_motor.brake();
  }

  @Override
  public AngularVelocity getVelocity() {
    return m_motor.getVelocity();
  }
}
