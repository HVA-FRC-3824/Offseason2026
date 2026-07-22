// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.subsystems.drivebase.SwerveIO;
import java.util.Map;

public interface PathingIO {

  public default void init(SubsystemBase subsystem, SwerveIO io) {}

  public void addCommands(Map<String, Command> commands);

  public Command pathToPose(Pose2d pose);

  public SendableChooser<Command> getAutoSelector();
}
