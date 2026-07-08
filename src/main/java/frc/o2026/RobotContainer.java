// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.hardware.ctre.OrchestraOrchestrator;
import frc.lib.hardware.ctre.OrchestraOrchestrator.Song;
import frc.o2026.subsystems.drivebase.Swerve;
import frc.o2026.subsystems.drivebase.SwerveIOReal;
import frc.o2026.subsystems.drivebase.SwerveIOSim;
import frc.o2026.subsystems.drivebase.objectVision.ObjectCameraIOPhoton;
import frc.o2026.subsystems.drivebase.objectVision.ObjectVision;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOLimelight;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOPhoton;
import frc.o2026.subsystems.gyro.GyroPigeon;

public class RobotContainer extends SubsystemBase {

  private Swerve m_swerve =
      new Swerve(
          RobotBase.isReal()
              ? new SwerveIOReal(
                  new GyroPigeon(),
                  new PoseCameraIOPhoton(
                      Constants.Vision.CameraName3, Constants.Vision.RobotToCam3),
                  new PoseCameraIOLimelight(
                      Constants.Vision.CameraName2,
                      Constants.Vision.RobotToCam2,
                      () -> RobotState.getInstance().getPoseEst().getRotation(),
                      () -> RobotState.getInstance().getAngularVelocity()))
              : new SwerveIOSim());

  private ObjectVision m_odVision =
      new ObjectVision(
          new ObjectCameraIOPhoton(Constants.Vision.CameraName1, Constants.Vision.RobotToCam1));

  //   private Roller m_roller = new Roller(new RollerIOTalonFX());

  //   private Indexer m_indexer =
  //       new Indexer(
  //           RobotBase.isReal()
  //               ? new TalonIO(Constants.CanIds.IndexerMotorId, Configs.Indexer.BeltConfig)
  //               : new MotorSimIO(
  //                   Constants.CanIds.IndexerMotorId,
  //                   Configs.Indexer.BeltConfig,
  //                   true,
  //                   SimModels.Indexer,
  //                   SimModels.IndexerGearRatio),
  //           RobotBase.isReal()
  //               ? new TalonIO(Constants.CanIds.KickerMotorId, Configs.Indexer.KickerConfig)
  //               : new MotorSimIO(
  //                   Constants.CanIds.KickerMotorId,
  //                   Configs.Indexer.KickerConfig,
  //                   true,
  //                   SimModels.Indexer,
  //                   SimModels.IndexerGearRatio));

  //   private Intake m_intake =
  //       new Intake(
  //           RobotBase.isReal()
  //               ? new TalonIO(
  //                   Constants.CanIds.IntakePositionFollowerMotorId, Configs.Intake.PivotConfig)
  //               : new MotorSimIO(
  //                   Constants.CanIds.IntakePositionFollowerMotorId,
  //                   Configs.Intake.PivotConfig,
  //                   true,
  //                   SimModels.Intake,
  //                   SimModels.IntakeGearRatio));

  //   private Flywheel m_flywheel =
  //       new Flywheel(
  //           RobotBase.isReal()
  //               ? new TalonIO(Constants.CanIds.FlywheelMotorId, Configs.Flywheel.Config)
  //               : new FlywheelSimIO(
  //                   Constants.CanIds.FlywheelMotorId,
  //                   Configs.Flywheel.Config,
  //                   true,
  //                   SimModels.Flywheel,
  //                   SimModels.FlywheelGearRatio),
  //           new TalonIO(Constants.CanIds.FlywheelFollowerMotorId, Configs.Flywheel.Config));

  private CommandXboxController m_driver = new CommandXboxController(Constants.Usb.DrivePort);
  //   private CommandXboxController m_operator = new
  // CommandXboxController(Constants.Usb.OperatorPort);

  private SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.0);

  enum Limiting {
    TrigExp,
    Linear
  }

  Limiting m_limit = Limiting.Linear;

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    m_swerve.setDefaultCommand(m_swerve.drive(this::getSpeeds));

    m_driver.a().whileTrue(m_swerve.resetGyro());
    m_driver.y().onTrue(m_swerve.fieldCentricityToggle());

    m_driver
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                m_swerve
                    .aimMove(
                        () -> new ChassisSpeeds(0.2, 0.0, 0.0),
                        () -> m_odVision.directionToObject(0, m_swerve.getPose().toPose2d()),
                        true)
                    .onlyWhile(() -> m_odVision.hasObjects(0))));

    m_driver
        .rightBumper()
        .whileTrue(
            m_swerve.aimMove(this::getSpeeds, RobotState.getInstance()::getSOTMRotTarget, false));

    m_driver
        .b()
        .onTrue(new InstantCommand(() -> OrchestraOrchestrator.playSong(Song.GymLeader), m_swerve));

    // m_flywheel.setDefaultCommand(m_flywheel.auto());
    // m_swerve.setDefaultCommand(m_swerve.aimSOTM(ChassisSpeeds::new));
    // m_indexer.setDefaultCommand(m_indexer.on());
    // m_intake.setDefaultCommand(m_intake.alligator());

    // ControlMode.Teleop.getTrigger()
    //     .and(m_driver.rightTrigger())
    //     .whileTrue(
    //         Commands.parallel(
    //             m_flywheel.auto(),
    //             m_swerve.aimSOTM(this::getSpeeds),
    //             m_indexer.on().onlyWhile(m_flywheel::isReady).onlyWhile(m_swerve::isAimed),
    //             m_intake.alligator()));

    // ControlMode.Teleop.getTrigger()
    //     .and(m_driver.rightTrigger())
    //     .onFalse(Commands.parallel(m_flywheel.off(), m_intake.stowed()));

    // ControlMode.Teleop.getTrigger()
    //     .and(m_driver.leftTrigger())
    //     .onTrue(Commands.parallel(m_intake.deploy(), m_roller.on()));
    // ControlMode.Teleop.getTrigger()
    //     .and(m_driver.leftTrigger())
    //     .onFalse(Commands.parallel(m_roller.off()));

    // NamedCommands.registerCommand(
    //     "ShootRoutine",
    //     Commands.deferredProxy(
    //         () ->
    //             Commands.parallel(
    //                     m_flywheel.auto(),
    //                     m_swerve.aimSOTM(ChassisSpeeds::new),
    //                     m_indexer
    //                         .on()
    //                         .onlyWhile(m_flywheel::isReady)
    //                         .onlyWhile(m_swerve::isAimed),
    //                     m_intake.alligator())
    //                 .repeatedly()));
  }

  private ChassisSpeeds getSpeeds() {

    double strafe, forwards, rot;

    switch (m_limit) {
      case TrigExp:
        double angle = Math.atan2(m_driver.getLeftY(), m_driver.getLeftX());
        double magnitude =
            Math.sqrt(Math.pow(m_driver.getLeftY(), 2) + Math.pow(m_driver.getLeftX(), 2));

        magnitude =
            Math.pow(Math.abs(magnitude), Configs.Chassis.TranslateExponentialPower) * magnitude;

        strafe = magnitude * Math.sin(angle);
        forwards = magnitude * Math.cos(angle);

        rot =
            Math.pow(Math.abs(m_driver.getRightX()), Configs.Chassis.AngularExponentialPower)
                * m_driver.getRightX();
        break;
      default:
        forwards = m_xLimiter.calculate(-m_driver.getLeftX());
        strafe = m_yLimiter.calculate(-m_driver.getLeftY());
        rot = m_rotLimiter.calculate(-m_driver.getRightX());
        break;
    }

    return new ChassisSpeeds(
        Configs.Chassis.MaximumLinear.times(strafe).div(2),
        Configs.Chassis.MaximumLinear.times(forwards).div(2),
        Configs.Chassis.MaximumAngularVelocity.times(rot).div(2));
  }

  public Command getInit() {

    return Commands.parallel(m_swerve.resetSwerveModules()).withName("Init");
  }

  public Command getAuto() {

    return m_autoChooser.getSelected();
  }
}
