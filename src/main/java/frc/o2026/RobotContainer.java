// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.o2026.Constants.Vision;
import frc.o2026.subsystems.flywheel.Flywheel;
import frc.o2026.subsystems.flywheel.FlywheelIONothing;
import frc.o2026.subsystems.gyro.Gyro;
import frc.o2026.subsystems.gyro.GyroPigeon;
import frc.o2026.subsystems.indexer.Indexer;
import frc.o2026.subsystems.indexer.IndexerIONothing;
import frc.o2026.subsystems.intake.Intake;
import frc.o2026.subsystems.intake.IntakeIONothing;
import frc.o2026.subsystems.roller.Roller;
import frc.o2026.subsystems.roller.RollerIONothing;
import frc.o2026.subsystems.swerve.Swerve;
import frc.o2026.subsystems.swerve.SwerveIOReal;

public class RobotContainer {

  private Swerve m_swerve = new Swerve(new SwerveIOReal());
  private Gyro m_gyro = new Gyro(new GyroPigeon());
  private Roller m_roller = new Roller(new RollerIONothing());
  private Indexer m_indexer = new Indexer(new IndexerIONothing());
  private Intake m_intake = new Intake(new IntakeIONothing());
  private Flywheel m_flywheel = new Flywheel(new FlywheelIONothing());
  private Vision m_vision = new Vision();

  private CommandXboxController m_driver = new CommandXboxController(Constants.Usb.DrivePort);
  private CommandXboxController m_operator = new CommandXboxController(Constants.Usb.OperatorPort);

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    m_swerve.setDefaultCommand(m_swerve.drive(this::getSpeeds));
  }

  private ChassisSpeeds getSpeeds() {

    double leftX = m_driver.getLeftX();
    double leftY = m_driver.getLeftY();
    double rightX = m_driver.getRightX();

    double angle = Math.atan2(leftY, leftX);

    double magnitude = Math.sqrt(leftY * leftY + leftX * leftX);

    magnitude =
        Math.pow(Math.abs(magnitude), Constants.Chassis.TranslateExponentialPower)
            * magnitude; // expo stuff here

    double strafe = magnitude * Math.sin(angle);
    double forwards = magnitude * Math.cos(angle);

    double rot =
        Math.pow(Math.abs(rightX), Constants.Chassis.AngularExponentialPower)
            * magnitude; // expo stuff here

    return new ChassisSpeeds(
        Constants.Chassis.MaximumLinear.times(strafe),
        Constants.Chassis.MaximumLinear.times(forwards),
        Constants.Chassis.MaximumAngularVelocity.times(rot));
  }

  public Command getInit() {

    return Commands.parallel(m_swerve.resetSwerveModules()).withName("Init");
  }

  public Command getAuto() {

    return m_autoChooser.getSelected();
  }
}
