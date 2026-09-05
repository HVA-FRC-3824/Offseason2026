// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.o2026.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Configs;
import frc.o2026.RobotState;
import frc.shared.hardware.motor.MotorIO;
import frc.shared.hardware.motor.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private MotorIO m_beltIo;
  private MotorIO m_kickIo;

  private MotorInputsAutoLogged m_beltIoInputs = new MotorInputsAutoLogged();
  private MotorInputsAutoLogged m_kickIoInputs = new MotorInputsAutoLogged();

  public Indexer(MotorIO beltIo, MotorIO kickIo) {

    m_beltIo = beltIo;
    m_kickIo = kickIo;
  }

  public static enum IndexerDesiredState {
    off,
    on,
    backwards
  }

  private IndexerDesiredState m_desiredState = IndexerDesiredState.off;

  public Command setState(IndexerDesiredState desiredState) {

    return runOnce(() -> m_desiredState = desiredState);
  }

  @Override
  public void periodic() {

    m_beltIo.periodic();
    m_kickIo.periodic();

    Logger.processInputs("indexerBelt", m_beltIoInputs);
    Logger.processInputs("indexerKick", m_kickIoInputs);

    Logger.recordOutput("Sim/indexing", RobotState.isSimIndexing());

    switch (m_desiredState) {
      case off:
        RobotState.setSimIndexing(false);
        m_beltIo.brake();
        m_kickIo.brake();

        break;

      case on:
        RobotState.setSimIndexing(true);
        m_beltIo.setVelocity(Configs.Indexer.BeltTurnsPerSec);
        m_kickIo.setVelocity(Configs.Indexer.KickerWheelTurnsPerSec);

        break;

      case backwards:
        RobotState.setSimIndexing(false);
        m_beltIo.setVelocity(Configs.Indexer.BeltTurnsPerSec.times(-1.0));
        m_kickIo.setVelocity(Configs.Indexer.KickerWheelTurnsPerSec.times(-1.0));

        break;

      default:
        break;
    }
  }
}
