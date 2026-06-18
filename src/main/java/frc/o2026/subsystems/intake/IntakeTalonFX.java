// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Constants;

public class IntakeTalonFX implements IntakeIO {
  public MotorIO m_motor;
  public MotorIO m_motorFollower;

  public IntakeTalonFX() {

    m_motor =
        new TalonIO(Constants.CanIds.IntakePositionLeaderMotorId, Constants.Intake.PivotConfig);
    m_motorFollower =
        new TalonIO(Constants.CanIds.IntakePositionFollowerMotorId, Constants.Intake.PivotConfig);

    m_motorFollower.follow(Constants.CanIds.IntakePositionLeaderMotorId, true);
  }

  @Override
  public void setPos(Angle angle) {
    m_motor.setPosition(angle);
  }

  @Override
  public Angle getPos() {
    return m_motor.getPos();
  }
}
