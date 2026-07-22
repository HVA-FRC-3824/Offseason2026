// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.pathing;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alliance;
import frc.o2026.subsystems.drivebase.SwerveIO;
import java.util.Map;

public class PathingIOPathPlanner implements PathingIO {

  @Override
  public void init(SubsystemBase subsystem, SwerveIO io) {

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      e.printStackTrace();
      e.printStackTrace();
      return;
    }

    // Configure the AutoBuilder
    AutoBuilder.configure(
        io::getPose,
        io::resetPose,
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        () -> io.getSpeeds(),
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        (speeds, feedforwards) -> io.driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0),
            // Rotation PID constants
            new PIDConstants(1.0, 0.0, 0.0)),
        config,
        Alliance::isRed,
        subsystem);
  }

  @Override
  public void addCommands(Map<String, Command> commands) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'addCommands'");
  }

  @Override
  public Command pathToPose(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'pathToPose'");
  }

  @Override
  public SendableChooser<Command> getAutoSelector() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAutoSelector'");
  }
}
