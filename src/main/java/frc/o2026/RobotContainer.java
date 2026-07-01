// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.hardware.ctre.FlywheelSimIO;
import frc.lib.hardware.ctre.MotorSimIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.subsystems.drivebase.Swerve;
import frc.o2026.subsystems.drivebase.SwerveIOReal;
import frc.o2026.subsystems.drivebase.SwerveIOSim;
import frc.o2026.subsystems.flywheel.Flywheel;
import frc.o2026.subsystems.gyro.GyroPigeon;
import frc.o2026.subsystems.indexer.Indexer;
import frc.o2026.subsystems.intake.Intake;
import frc.o2026.subsystems.roller.Roller;
import frc.o2026.subsystems.roller.RollerIOTalonFX;

public class RobotContainer {

  private Swerve m_swerve =
      new Swerve(RobotBase.isReal() ? new SwerveIOReal(new GyroPigeon()) : new SwerveIOSim());

  private Roller m_roller = new Roller(new RollerIOTalonFX());

  private Indexer m_indexer =
      new Indexer(
          RobotBase.isSimulation()
              ? new TalonIO(Constants.CanIds.IndexerMotorId, Configs.Indexer.BeltConfig)
              : new MotorSimIO(
                  Constants.CanIds.IndexerMotorId,
                  Configs.Indexer.BeltConfig,
                  true,
                  new DCMotorSim(
                      LinearSystemId.createDCMotorSystem(
                          DCMotor.getKrakenX60(1), 0.021, 1.0 // MOI from CAD
                          ),
                      DCMotor.getKrakenX60(1)),
                  25),
          RobotBase.isSimulation()
              ? new TalonIO(Constants.CanIds.KickerMotorId, Configs.Indexer.KickerConfig)
              : new MotorSimIO(
                  Constants.CanIds.KickerMotorId,
                  Configs.Indexer.KickerConfig,
                  true,
                  new DCMotorSim(
                      LinearSystemId.createDCMotorSystem(
                          DCMotor.getKrakenX60(1), 0.021, 1.0 // MOI from CAD
                          ),
                      DCMotor.getKrakenX60(1)),
                  10));

  private Intake m_intake =
      new Intake(
          new MotorSimIO(
              Constants.CanIds.IntakePositionFollowerMotorId,
              Configs.Intake.PivotConfig,
              true,
              new DCMotorSim(
                  LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), 0.210408789, 1.0),
                  DCMotor.getKrakenX60(2),
                  0.2),
              1));

  private Flywheel m_flywheel =
      new Flywheel(
          RobotBase.isSimulation()
              ? new TalonIO(Constants.CanIds.FlywheelMotorId, Configs.Flywheel.Config, true)
              : new FlywheelSimIO(
                  Constants.CanIds.FlywheelMotorId,
                  Configs.Flywheel.Config,
                  true,
                  new FlywheelSim(
                      LinearSystemId.createFlywheelSystem(
                          DCMotor.getKrakenX60(2), 0.0011279246, 1.0),
                      DCMotor.getKrakenX60(2),
                      0.2),
                  1),
          new TalonIO(Constants.CanIds.FlywheelFollowerMotorId, Configs.Flywheel.Config, true));

  private CommandXboxController m_driver = new CommandXboxController(Constants.Usb.DrivePort);
  private CommandXboxController m_operator = new CommandXboxController(Constants.Usb.OperatorPort);

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    m_swerve.setDefaultCommand(m_swerve.drive(this::getSpeeds));

    m_driver
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                m_flywheel.auto(),
                m_swerve.aimSOTM(this::getSpeeds),
                m_indexer.on().onlyWhile(m_flywheel::isReady),
                m_intake.alligator()));

    m_driver.rightTrigger().onFalse(Commands.parallel(m_flywheel.off(), m_intake.stowed()));

    m_driver.leftTrigger().onTrue(Commands.parallel(m_intake.deploy(), m_roller.on()));
    m_driver.leftTrigger().onFalse(Commands.parallel(m_roller.off()));
  }

  private ChassisSpeeds getSpeeds() {

    double leftX = m_driver.getLeftX();
    double leftY = m_driver.getLeftY();
    double rightX = -m_driver.getRightX();

    double angle = Math.atan2(leftY, leftX);

    double magnitude = Math.sqrt(leftY * leftY + leftX * leftX);

    magnitude =
        Math.pow(Math.abs(magnitude), Configs.Chassis.TranslateExponentialPower)
            * magnitude; // expo stuff here

    double strafe = magnitude * Math.sin(angle);
    double forwards = magnitude * Math.cos(angle);

    double rot =
        Math.pow(Math.abs(rightX), Configs.Chassis.AngularExponentialPower)
            * rightX; // expo stuff here

    return new ChassisSpeeds(
        Configs.Chassis.MaximumLinear.times(strafe),
        Configs.Chassis.MaximumLinear.times(forwards),
        Configs.Chassis.MaximumAngularVelocity.times(rot));
  }

  public Command getInit() {

    return Commands.parallel(m_swerve.resetSwerveModules()).withName("Init");
  }

  public Command getAuto() {

    return m_autoChooser.getSelected();
  }
}
