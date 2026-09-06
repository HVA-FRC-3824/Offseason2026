// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.o2026;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.shared.hardware.SimBattery;
import frc.shared.hardware.motor.ctre.OrchestraOrchestrator;
import frc.shared.rebuilt.BallSim;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    if (Constants.Impl == Constants.RobotImpl.Replay) {

      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    } else {

      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    m_robotContainer = new RobotContainer();

    OrchestraOrchestrator.sendChooser();
  }

  @Override
  public void robotInit() {

    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    SimBattery.calculateSupplyVoltage();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAuto();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {

    SimulatedArena.getInstance().simulationPeriodic();
    BallSim.getInstance().update();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    OrchestraOrchestrator.sendChooser();
  }
}
