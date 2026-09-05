// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.o2026.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Configs;
import frc.o2026.RobotState;
import frc.shared.hardware.motor.MotorIO;
import frc.shared.hardware.motor.MotorIO.MotorInputs;
import frc.shared.hardware.motor.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {

  public static enum RollerDesiredState {
    off,
    on,
    backwards
  }

  private RollerDesiredState m_desiredState = RollerDesiredState.off;

  private final MotorIO m_io;
  private final MotorInputs m_ioInputs = new MotorInputs();

  private final Timer m_blockageDetector = new Timer();
  private final Timer m_unblockDuration = new Timer();

  // Time stopped to start unblocking
  private static final Time blockTime = Seconds.of(1.0);
  // Time unblocking to start moving again
  private static final Time unblockingTime = Seconds.of(1.0);

  public Roller(MotorIO io) {

    m_io = io;
  }

  public Command setState(RollerDesiredState state) {

    return runOnce(() -> m_desiredState = state);
  }

  @Override
  public void periodic() {

    m_io.updateInputs(m_ioInputs);
    Logger.processInputs("Roller", (MotorInputsAutoLogged) m_ioInputs);

    switch (m_desiredState) {
      case on:

        // 1. Check if we're stopped and start timer before attempting unblock
        if (m_ioInputs.statorCurrent.in(Amps)
            > (Configs.Roller.RollerConfig.getStatorCurrent().in(Amps) - 0.5)) {
          m_blockageDetector.reset();
          m_blockageDetector.start();
        } else if (m_ioInputs.velocity.gt(RotationsPerSecond.of(2.0))) {
          m_blockageDetector.stop();
        }

        // 2. If the block timer exceeds the buffer time, start unblocking
        if (m_blockageDetector.get() > blockTime.in(Seconds)) {
          m_unblockDuration.reset();
          m_unblockDuration.start();
        }

        // 3. If the unblock duraiton
        if (m_unblockDuration.get() > unblockingTime.in(Seconds)) {
          m_unblockDuration.stop();
          m_blockageDetector.reset();
        }

        // 4. If we're unblocking, reverse
        if (m_unblockDuration.isRunning()) {
          m_io.setVelocity(Configs.Roller.IntakeTurnsPerSec.times(-1.0));
          break;
        }

        // 5. If nothing else, intake
        m_io.setVelocity(Configs.Roller.IntakeTurnsPerSec);
        break;

      case off:
        m_io.brake();

        m_unblockDuration.stop();
        m_unblockDuration.reset();
        m_blockageDetector.stop();
        m_blockageDetector.reset();
        break;
      case backwards:
        m_io.setVelocity(Configs.Roller.IntakeTurnsPerSec.times(-1.0));

        m_unblockDuration.stop();
        m_unblockDuration.reset();
        m_blockageDetector.stop();
        m_blockageDetector.reset();
        break;
    }

    if (m_ioInputs.lastReference >= 10.0) {
      RobotState.setSimIntaking(true);
    } else {
      RobotState.setSimIntaking(false);
    }
  }
}
