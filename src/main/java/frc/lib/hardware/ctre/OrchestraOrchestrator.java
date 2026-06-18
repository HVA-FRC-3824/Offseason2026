// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.ctre;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.EnumChooser;
import java.util.HashMap;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class OrchestraOrchestrator {

  // This is immedietly published
  private static EnumChooser<Song> m_songPlayer =
      new EnumChooser<Song>("Song Chooser", Song.GymLeader);

  static {
    m_songPlayer.onChange(OrchestraOrchestrator::playSong);
  }

  private static Orchestra m_orchestra = new Orchestra();

  private static HashMap<Integer, TalonFX> m_motors = new HashMap<Integer, TalonFX>();

  public static void addInstrument(TalonFX motor) {
    if (RobotBase.isSimulation()) return;

    if (m_motors.put(motor.getDeviceID(), motor) == null) m_orchestra.addInstrument(motor);
  }

  public static void removeInstrument(int id) {
    if (RobotBase.isSimulation()) return;

    if (m_motors.remove(id) != null) {

      m_orchestra.clearInstruments();
      m_motors.forEach(
          (canId, motor) -> {
            m_orchestra.addInstrument(motor);
          });
    }
  }

  public static void playSong(Song song) {
    if (RobotBase.isSimulation()) return;

    StatusCode status = m_orchestra.loadMusic(song.getPath());
    Logger.recordOutput("MusicErr/load", status.toString());

    status = m_orchestra.play();
    Logger.recordOutput("MusicErr/play", status.toString());
  }

  public static enum Song {
    Poofs("254.chrp"),
    Cynthia("cynthia.chrp"),
    GymLeader("gymleader.chrp"),
    Birthday("happybirthday.chrp"),
    Pirates("POTC.chrp"),
    Song2("song2.chrp"),
    Tetris("tetris.chrp"),
    UnderTheSea("underthesea.chrp");

    @Getter private String path;

    private Song(String path) {
      this.path = path;
    }
  }
}
