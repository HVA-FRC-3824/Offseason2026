// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.shared.GuitarController;
import frc.shared.NONBenevolentSalesman;
import frc.shared.Util;
import frc.shared.hardware.gyro.GyroIOPigeon;
import frc.shared.hardware.motor.MotorIONothing;
import frc.shared.hardware.motor.ctre.MotorIOFlywheelSim;
import frc.shared.hardware.motor.ctre.MotorIOSim;
import frc.shared.hardware.motor.ctre.MotorIOTalonFX;
import frc.o2026.subsystems.Flywheel;
import frc.o2026.subsystems.Indexer;
import frc.o2026.subsystems.Intake;
import frc.o2026.subsystems.Roller;
import frc.o2026.subsystems.Roller.RollerDesiredState;
import frc.o2026.subsystems.drivebase.Swerve;
import frc.o2026.subsystems.drivebase.SwerveIOReal;
import frc.o2026.subsystems.drivebase.SwerveIOSim;
import frc.o2026.subsystems.drivebase.objectVision.ObjectCameraIOPhoton;
import frc.o2026.subsystems.drivebase.objectVision.ObjectCameraIOSim;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOLimelight;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOPhoton;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOSim;

import java.util.function.Supplier;

public class RobotContainer extends SubsystemBase {

  private Swerve m_swerve;
  private Roller m_roller;
  private Indexer m_indexer;
  private Intake m_intake;
  private Flywheel m_flywheel;

  private static enum Robot {
    Real,
    DevBot,
    Sim
  }

  private static Robot m_impl = Robot.DevBot;

  private CommandXboxController m_driver = new CommandXboxController(Constants.Usb.DrivePort);
  //   private CommandXboxController m_operator = new
  // CommandXboxController(Constants.Usb.OperatorPort);
  private GuitarController m_guitar = new GuitarController(Constants.Usb.GuitarPort);
  private NONBenevolentSalesman m_creditOrDebit =
      new NONBenevolentSalesman(Constants.Usb.CreditPort);

  private SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.0);

  enum Limiting {
    TrigExp,
    Linear
  }

  Limiting m_limit = Limiting.TrigExp;

  private final SendableChooser<Command> m_autoChooser;

  private final Supplier<Command> shootCmd =
      () ->
          Commands.parallel(
              m_flywheel.auto(),
              m_indexer.on().onlyWhile(() -> m_flywheel.isReady() && m_swerve.isAimed()),
              m_intake.stowed());

  private final Supplier<Command> autoIntakeCmd =
      () ->
          Commands.parallel(
                  m_intake.deploy(),
                  m_roller.setState(RollerDesiredState.on),
                  m_swerve
                      .autoIntake()
                      .andThen(
                          m_swerve.drive(() -> new ChassisSpeeds(0.5, 0.0, Math.PI / 2), false))
                      .until(m_swerve::hasObjects)
                      .repeatedly())
              .withTimeout(3)
              .andThen(
                  Commands.sequence(m_roller.setState(RollerDesiredState.off), m_intake.stowed()));

  public RobotContainer() {

    switch (m_impl) {
      case Real:
        m_swerve =
            new Swerve(
                new SwerveIOReal(
                    new GyroIOPigeon(Constants.CanIds.PigeonGyroId),
                    new PoseCameraIOPhoton(Constants.Vision.FrontCamConfig),
                    new PoseCameraIOPhoton(Constants.Vision.WebCam),
                    new PoseCameraIOLimelight(Constants.Vision.LimelightOfDoomAndDespair)),
                new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig));

        m_roller = new Roller(new MotorIOTalonFX(Constants.CanIds.FuelIntakeMotorId, Configs.Roller.RollerConfig));

        m_indexer =
            new Indexer(
                new MotorIOTalonFX(Constants.CanIds.IndexerMotorId, Configs.Indexer.BeltConfig),
                new MotorIOTalonFX(Constants.CanIds.KickerMotorId, Configs.Indexer.KickerConfig));

        m_intake =
            new Intake(
                new MotorIOTalonFX(
                    Constants.CanIds.IntakePositionLeaderMotorId, Configs.Intake.PivotConfig),
                new MotorIOTalonFX(
                    Constants.CanIds.IntakePositionFollowerMotorId,
                    Configs.Intake.PivotConfig.withInverted(false)));

        m_flywheel =
            new Flywheel(
                new MotorIOTalonFX(Constants.CanIds.FlywheelMotorId, Configs.Flywheel.Config),
                new MotorIOTalonFX(
                    Constants.CanIds.FlywheelFollowerMotorId, Configs.Flywheel.Config));
        break;

      case DevBot:
        m_swerve =
            new Swerve(
                RobotBase.isReal()
                    ? new SwerveIOReal(
                        new GyroIOPigeon(Constants.CanIds.PigeonGyroId),
                        new PoseCameraIOPhoton(Constants.Vision.FrontCamConfig),
                        new PoseCameraIOPhoton(Constants.Vision.WebCam),
                        new PoseCameraIOLimelight(Constants.Vision.LimelightOfDoomAndDespair))
                    : new SwerveIOSim(
                        new PoseCameraIOSim(Constants.Vision.FrontCamConfig),
                        new PoseCameraIOSim(Constants.Vision.WebCam),
                        new PoseCameraIOSim(Constants.Vision.LimelightOfDoomAndDespair)),
                RobotBase.isReal()
                    ? new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig)
                    : new ObjectCameraIOSim(Constants.Vision.BackCamConfig));

        m_roller = new Roller(new MotorIONothing());

        m_indexer = new Indexer(new MotorIONothing(), new MotorIONothing());

        m_intake = new Intake(new MotorIONothing(), new MotorIONothing());

        m_flywheel = new Flywheel(new MotorIONothing(), new MotorIONothing());
        break;

      case Sim:
        m_swerve =
            new Swerve(
                new SwerveIOSim(
                    new PoseCameraIOSim(Constants.Vision.FrontCamConfig),
                    new PoseCameraIOSim(Constants.Vision.WebCam),
                    new PoseCameraIOSim(Constants.Vision.LimelightOfDoomAndDespair)),
                new ObjectCameraIOSim(Constants.Vision.BackCamConfig));

        m_roller = new Roller(new MotorIOSim(Constants.CanIds.FuelIntakeMotorId, Configs.Roller.RollerConfig, true, 0.05, 1, 1));

        m_indexer =
            new Indexer(
                new MotorIOSim(
                    Constants.CanIds.IndexerMotorId,
                    Configs.Indexer.BeltConfig,
                    true,
                    Constants.SimModels.IndexerMOI,
                    1,
                    Constants.SimModels.IndexerGearRatio),
                new MotorIOSim(
                    Constants.CanIds.KickerMotorId,
                    Configs.Indexer.KickerConfig,
                    true,
                    Constants.SimModels.IndexerMOI,
                    1,
                    Constants.SimModels.IndexerGearRatio));

        m_intake =
            new Intake(
                new MotorIOSim(
                    Constants.CanIds.IntakePositionLeaderMotorId,
                    Configs.Intake.PivotConfig,
                    true,
                    Constants.SimModels.IntakeMOI,
                    2,
                    Constants.SimModels.IntakeGearRatio),
                new MotorIOSim(
                    Constants.CanIds.IntakePositionFollowerMotorId,
                    Configs.Intake.PivotConfig,
                    true,
                    Constants.SimModels.IntakeMOI,
                    2,
                    Constants.SimModels.IntakeGearRatio));

        // Both sims use 2 motors to simulate the effects of the other motor
        m_flywheel =
            new Flywheel(
                new MotorIOFlywheelSim(
                    Constants.CanIds.FlywheelMotorId,
                    Configs.Flywheel.Config,
                    true,
                    Constants.SimModels.FlywheelMOI,
                    2,
                    Constants.SimModels.FlywheelGearRatio),
                new MotorIOFlywheelSim(
                    Constants.CanIds.FlywheelFollowerMotorId,
                    Configs.Flywheel.Config,
                    true,
                    Constants.SimModels.FlywheelMOI,
                    2,
                    Constants.SimModels.FlywheelGearRatio));
        break;
    }

    // AUTOS

    m_autoChooser = AutoBuilder.buildAutoChooser();
    m_autoChooser.addOption(
        "Custom",
        Commands.sequence(
                m_swerve.runOnce(
                    () ->
                        m_swerve.resetPose(
                            Util.flipOnRed(new Pose2d(4.4, 7.4, Rotation2d.kCCW_90deg)))),
                m_swerve
                    .bLinePathPose(Util.flipOnRed(new Pose2d(7.7, 7.5, Rotation2d.kCCW_90deg)))
                    .withTimeout(3),
                autoIntakeCmd.get(),
                m_swerve.crossTrench(),
                m_swerve
                    .pidPathPose(
                        () ->
                            (Util.flipOnRed(
                                new Pose2d(2.3, 2.8, RobotState.getSOTMRotTarget()))))
                    .alongWith(shootCmd.get()))
            .andThen(m_indexer.off()));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // DEFAULT COMMANDS

    m_swerve.setDefaultCommand(m_swerve.drive(this::getSpeeds));
    m_flywheel.setDefaultCommand(m_flywheel.off());
    m_indexer.setDefaultCommand(m_indexer.off());
    m_roller.setDefaultCommand(m_roller.setState(RollerDesiredState.off));

    // CONTROLLER BINDINGS

    m_driver.a().onTrue(m_swerve.resetGyro());
    m_driver.y().onTrue(m_swerve.toggleFieldCentricity());

    m_driver.rightTrigger().whileTrue(shootCmd.get().alongWith(m_swerve.aimSOTM(this::getSpeeds)));

    m_driver
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                m_intake.deploy(),
                m_roller.setState(RollerDesiredState.on),
                m_swerve.targetAssistedDrive(this::getSpeeds)));

    SmartDashboard.putData("Auto Intake", autoIntakeCmd.get());
    SmartDashboard.putData("Cross Trench", m_swerve.crossTrench());
    SmartDashboard.putData("SOTM", SOTM(new Translation2d(2.0, 3.9)));

    SmartDashboard.putData("yUp", m_swerve.drive(() -> new ChassisSpeeds(-0.5, 0.0, 0.0), true));
    SmartDashboard.putData("yDown", m_swerve.drive(() -> new ChassisSpeeds(0.5, 0.0, 0.0), true));
    SmartDashboard.putData("xLeft", m_swerve.drive(() -> new ChassisSpeeds(0.0, -0.5, 0.0), true));
    SmartDashboard.putData("xRight", m_swerve.drive(() -> new ChassisSpeeds(0.0, 0.5, 0.0), true));

    SmartDashboard.putData(
        "rotLeft", m_swerve.drive(() -> new ChassisSpeeds(0.0, 0.0, Math.PI / 2), true));
    SmartDashboard.putData(
        "rotRight", m_swerve.drive(() -> new ChassisSpeeds(0.0, 0.0, -Math.PI / 2), true));

    m_guitar.A().onTrue(m_swerve.drive(() -> new ChassisSpeeds(-0.5, 0.0, 0.0)));
    m_guitar.D().onTrue(m_swerve.drive(() -> new ChassisSpeeds(0.5, 0.0, 0.0)));
    m_guitar.G().onTrue(m_swerve.drive(() -> new ChassisSpeeds(0.0, -0.5, 0.0)));
    m_guitar.B().onTrue(m_swerve.drive(() -> new ChassisSpeeds(0.0, 0.5, 0.0)));
    m_guitar.E().onTrue(m_swerve.crossTrench());

    m_creditOrDebit
        .swipe()
        .onTrue(
            m_swerve.defer(
                () -> {
                  return Commands.race(
                      m_swerve.aim(m_swerve.getHeading().plus(Rotation2d.k180deg)).repeatedly(),
                      new WaitCommand(10));
                }));

    // PATHING COMMANDS & TRIGGERS

    NamedCommands.registerCommand(
        "ShootAll", shootCmd.get().repeatedly().alongWith(m_swerve.aimShoot()));
    NamedCommands.registerCommand("AutoIntake", autoIntakeCmd.get());

    new EventTrigger("DeployIntake")
        .onTrue(m_intake.deploy().andThen(m_roller.setState(RollerDesiredState.on)));
    new EventTrigger("StowIntake").onTrue(m_intake.stowed());
  }

  private ChassisSpeeds getSpeeds() {

    double leftY = MathUtil.applyDeadband(m_driver.getLeftY(), 0.01);
    double leftX = MathUtil.applyDeadband(m_driver.getLeftX(), 0.01);
    double rightX = MathUtil.applyDeadband(m_driver.getRightX(), 0.01);

    double strafe, forwards, rot;

    switch (m_limit) {
      case TrigExp:
        double angle = Math.atan2(leftY, leftX);
        double magnitude = Math.sqrt(Math.pow(leftY, 2) + Math.pow(leftX, 2));

        magnitude =
            Math.pow(Math.abs(magnitude), Configs.Chassis.TranslateExponentialPower) * magnitude;

        strafe = magnitude * Math.sin(angle);
        forwards = magnitude * Math.cos(angle);

        rot =
            -Math.pow(Math.abs(m_driver.getRightX()), Configs.Chassis.AngularExponentialPower)
                * m_driver.getRightX();
        break;
      default:
        forwards = m_xLimiter.calculate(leftX);
        strafe = m_yLimiter.calculate(leftY);
        rot = m_rotLimiter.calculate(rightX);
        break;
    }

    return new ChassisSpeeds(
        Configs.Chassis.MaximumLinear.times(strafe),
        Configs.Chassis.MaximumLinear.times(forwards),
        Configs.Chassis.MaximumAngularVelocity.times(rot));
  }

  public Command SOTM(Translation2d target) {

    return m_swerve
        .pidPathPose(() -> new Pose2d(target, RobotState.getSOTMRotTarget()))
        .alongWith(shootCmd.get());
  }

  public Command getAuto() {

    return m_autoChooser.getSelected();
  }
}
