// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.o2026.subsystems.Flywheel;
import frc.o2026.subsystems.Flywheel.FlywheelDesiredState;
import frc.o2026.subsystems.Indexer;
import frc.o2026.subsystems.Indexer.IndexerDesiredState;
import frc.o2026.subsystems.Intake;
import frc.o2026.subsystems.Roller;
import frc.o2026.subsystems.Roller.RollerDesiredState;
import frc.o2026.subsystems.drivebase.Swerve;
import frc.o2026.subsystems.drivebase.Swerve.SwerveDesiredState;
import frc.o2026.subsystems.drivebase.SwerveIO;
import frc.o2026.subsystems.drivebase.SwerveIOReal;
import frc.o2026.subsystems.drivebase.SwerveIOSim;
import frc.o2026.subsystems.drivebase.SwerveModule;
import frc.shared.Util;
import frc.shared.hardware.gyro.GyroIOPigeon;
import frc.shared.hardware.motor.MotorIONothing;
import frc.shared.hardware.motor.ctre.MotorIOFlywheelSim;
import frc.shared.hardware.motor.ctre.MotorIOSim;
import frc.shared.hardware.motor.ctre.MotorIOTalonFX;
import frc.shared.hardware.motor.rev.MotorIOSparkMax;
import frc.shared.hardware.vision.objectVision.ObjectCameraIO;
import frc.shared.hardware.vision.objectVision.ObjectCameraIOPhoton;
import frc.shared.hardware.vision.objectVision.ObjectCameraIOSim;
import frc.shared.hardware.vision.poseVision.PoseCameraIOLimelight;
import frc.shared.hardware.vision.poseVision.PoseCameraIOPhoton;
import frc.shared.hardware.vision.poseVision.PoseCameraIOReplay;
import frc.shared.hardware.vision.poseVision.PoseCameraIOSim;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer extends SubsystemBase {

  private final Swerve m_swerve;
  private final Roller m_roller;
  private final Indexer m_indexer;
  private final Intake m_intake;
  private final Flywheel m_flywheel;

  private CommandXboxController m_driver = new CommandXboxController(Constants.Usb.DrivePort);
  private CommandXboxController m_operator = new CommandXboxController(Constants.Usb.OperatorPort);
  //   private GuitarController m_guitar = new GuitarController(Constants.Usb.GuitarPort);
  //   private NONBenevolentSalesman m_creditOrDebit =
  //       new NONBenevolentSalesman(Constants.Usb.CreditPort);

  private SlewRateLimiter m_xLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_yLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.0);

  enum Limiting {
    TrigExp,
    Linear
  }

  private Limiting m_limit = Limiting.TrigExp;

  private AngularVelocity m_trim = RPM.of(0.0);

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {

    switch (Constants.Impl) {
      case Real:
        m_swerve =
            new Swerve(
                new SwerveIOReal(
                    new SwerveModule(
                        "FrontRight",
                        new MotorIOTalonFX(
                            Constants.CanIds.FrontRightDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(
                            Constants.CanIds.FrontRightTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.FrontRightEncoderId,
                        Constants.Chassis.FrontRightForwardsAngle),
                    new SwerveModule(
                        "FrontLeft",
                        new MotorIOTalonFX(
                            Constants.CanIds.FrontLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(
                            Constants.CanIds.FrontLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.FrontLeftEncoderId,
                        Constants.Chassis.FrontLeftForwardsAngle),
                    new SwerveModule(
                        "BackRight",
                        new MotorIOTalonFX(
                            Constants.CanIds.BackRightDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(
                            Constants.CanIds.BackRightTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackRightEncoderId,
                        Constants.Chassis.BackRightForwardsAngle),
                    new SwerveModule(
                        "BackLeft",
                        new MotorIOTalonFX(
                            Constants.CanIds.BackLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOTalonFX(
                            Constants.CanIds.BackLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackLeftEncoderId,
                        Constants.Chassis.BackLeftForwardsAngle),
                    new GyroIOPigeon(Constants.CanIds.PigeonGyroId)),
                new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig, Inches.of(3.0)));

        m_roller =
            new Roller(
                new MotorIOTalonFX(
                    Constants.CanIds.FuelIntakeMotorId, Configs.Roller.RollerConfig));

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
                new SwerveIOReal(
                    new SwerveModule(
                        "FrontRight",
                        new MotorIOTalonFX(
                            Constants.CanIds.FrontRightDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOSparkMax(
                            Constants.CanIds.FrontRightTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.FrontRightEncoderId,
                        Constants.Chassis.FrontRightForwardsAngle),
                    new SwerveModule(
                        "FrontLeft",
                        new MotorIOTalonFX(
                            Constants.CanIds.FrontLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOSparkMax(
                            Constants.CanIds.FrontLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.FrontLeftEncoderId,
                        Constants.Chassis.FrontLeftForwardsAngle),
                    new SwerveModule(
                        "BackRight",
                        new MotorIOTalonFX(
                            Constants.CanIds.BackRightDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOSparkMax(
                            Constants.CanIds.BackRightTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackRightEncoderId,
                        Constants.Chassis.BackRightForwardsAngle),
                    new SwerveModule(
                        "BackLeft",
                        new MotorIOTalonFX(
                            Constants.CanIds.BackLeftDriveId, Configs.Chassis.DriveConfig),
                        new MotorIOSparkMax(
                            Constants.CanIds.BackLeftTurnId, Configs.Chassis.TurnConfig),
                        Constants.CanIds.BackLeftEncoderId,
                        Constants.Chassis.BackLeftForwardsAngle),
                    new GyroIOPigeon(Constants.CanIds.PigeonGyroId)),
                new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig, Inches.of(3.0)),
                new PoseCameraIOPhoton(Constants.Vision.FrontCamConfig),
                new PoseCameraIOPhoton(Constants.Vision.WebCam),
                new PoseCameraIOLimelight(Constants.Vision.LimelightOfDoomAndDespair));

        m_roller = new Roller(new MotorIONothing());

        m_indexer = new Indexer(new MotorIONothing(), new MotorIONothing());

        m_intake = new Intake(new MotorIONothing(), new MotorIONothing());

        m_flywheel = new Flywheel(new MotorIONothing(), new MotorIONothing());
        break;

      case Sim:
        m_swerve =
            new Swerve(
                new SwerveIOSim(),
                new ObjectCameraIOSim(Constants.Vision.BackCamConfig, SimulatedArena.getInstance()),
                new PoseCameraIOSim(Constants.Vision.FrontCamConfig),
                new PoseCameraIOSim(Constants.Vision.WebCam),
                new PoseCameraIOSim(Constants.Vision.LimelightOfDoomAndDespair));

        m_roller =
            new Roller(
                new MotorIOSim(
                    Constants.CanIds.FuelIntakeMotorId,
                    Configs.Roller.RollerConfig,
                    true,
                    0.05,
                    1,
                    1));

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

        // ie replay, but if our robot type is null or something we dont want to run it
      default:
        m_swerve =
            new Swerve(
                new SwerveIO() {},
                new ObjectCameraIO() {},
                new PoseCameraIOReplay(Constants.Vision.FrontCamConfig.name()),
                new PoseCameraIOReplay(Constants.Vision.WebCam.name()),
                new PoseCameraIOReplay(Constants.Vision.LimelightOfDoomAndDespair.name()));

        m_roller = new Roller(new MotorIONothing());

        m_indexer = new Indexer(new MotorIONothing(), new MotorIONothing());

        m_intake = new Intake(new MotorIONothing(), new MotorIONothing());

        // Both sims use 2 motors to simulate the effects of the other motor
        m_flywheel = new Flywheel(new MotorIONothing(), new MotorIONothing());
        break;
    }

    // COMMAND CREATORS

    Supplier<Command> shootCmd =
        () ->
            Commands.parallel(
                m_flywheel.setState(FlywheelDesiredState.autoScore.with(() -> m_trim)),
                m_indexer
                    .setState(IndexerDesiredState.on)
                    .onlyWhile(() -> m_flywheel.isReady() && m_swerve.isAimed()),
                m_intake.stowed());

    Supplier<Command> passCmd =
        () ->
            Commands.parallel(
                m_flywheel.setState(FlywheelDesiredState.autoScore.with(() -> m_trim)),
                m_indexer
                    .setState(IndexerDesiredState.on)
                    .onlyWhile(() -> m_flywheel.isReady() && m_swerve.isAimed()),
                m_intake.stowed());

    // AUTOS

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // DEFAULT COMMANDS

    m_swerve.setDefaultCommand(
        m_swerve.defer(() -> m_swerve.setState(SwerveDesiredState.driveDefault.with(getSpeeds()))));
    m_flywheel.setDefaultCommand(m_flywheel.setState(FlywheelDesiredState.off));
    m_indexer.setDefaultCommand(m_indexer.setState(IndexerDesiredState.off));
    m_roller.setDefaultCommand(m_roller.setState(RollerDesiredState.off));

    // CONTROLLER BINDINGS

    m_driver.a().onTrue(m_swerve.resetGyro());
    m_driver.y().onTrue(m_swerve.toggleFieldCentricity());

    m_driver
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                m_intake.deploy(),
                m_roller.setState(RollerDesiredState.on)));

    m_driver
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                m_intake.deploy(),
                m_roller.setState(RollerDesiredState.on),
                m_swerve.setState(SwerveDesiredState.intakeAssist.with(getSpeeds())).repeatedly()));

    m_driver
        .rightTrigger()
        .onTrue(
            shootCmd
                .get()
                .repeatedly()
                .alongWith(
                    m_swerve.setState(SwerveDesiredState.aimSOTM.with(getSpeeds())).repeatedly()));

    m_driver
        .rightBumper()
        .onTrue(
            passCmd
                .get()
                .repeatedly()
                .alongWith(
                    m_swerve.setState(SwerveDesiredState.aimPass.with(getSpeeds())).repeatedly()));

    m_operator.rightBumper().onTrue(Util.runOnce(() -> m_trim = m_trim.plus(RPM.of(50.0))));

    m_operator.leftBumper().onTrue(Util.runOnce(() -> m_trim = m_trim.minus(RPM.of(50.0))));

    m_operator.a().onTrue(Util.runOnce(() -> m_trim = RPM.of(0.0)));

    SmartDashboard.putData(
        "yUp",
        m_swerve
            .setState(SwerveDesiredState.driveField.with(new ChassisSpeeds(0.5, 0.0, 0.0)))
            .repeatedly()
            .withTimeout(Seconds.of(0.2)));

    SmartDashboard.putData(
        "yDown",
        m_swerve
            .setState(SwerveDesiredState.driveField.with(new ChassisSpeeds(-0.5, 0.0, 0.0)))
            .repeatedly()
            .withTimeout(Seconds.of(0.2)));

    SmartDashboard.putData(
        "xLeft",
        m_swerve
            .setState(SwerveDesiredState.driveField.with(new ChassisSpeeds(0.0, -0.5, 0.0)))
            .repeatedly()
            .withTimeout(Seconds.of(0.2)));

    SmartDashboard.putData(
        "xRight",
        m_swerve
            .setState(SwerveDesiredState.driveField.with(new ChassisSpeeds(0.0, 0.5, 0.0)))
            .repeatedly()
            .withTimeout(Seconds.of(0.2)));

    SmartDashboard.putData(
        "rotLeft",
        m_swerve
            .setState(SwerveDesiredState.driveField.with(new ChassisSpeeds(0.0, 0.0, Math.PI / 2)))
            .repeatedly()
            .withTimeout(Seconds.of(0.2)));

    SmartDashboard.putData(
        "rotRight",
        m_swerve
            .setState(SwerveDesiredState.driveField.with(new ChassisSpeeds(0.0, 0.0, -Math.PI / 2)))
            .repeatedly()
            .withTimeout(Seconds.of(0.2)));

    // m_guitar.A().onTrue(m_swerve.drive(() -> new ChassisSpeeds(-0.5, 0.0, 0.0)));
    // m_guitar.D().onTrue(m_swerve.drive(() -> new ChassisSpeeds(0.5, 0.0, 0.0)));
    // m_guitar.G().onTrue(m_swerve.drive(() -> new ChassisSpeeds(0.0, -0.5, 0.0)));
    // m_guitar.B().onTrue(m_swerve.drive(() -> new ChassisSpeeds(0.0, 0.5, 0.0)));

    // m_creditOrDebit
    //     .swipe()
    //     .onTrue(
    //         m_swerve.defer(
    //             () -> {
    //               return Commands.race(
    //                   m_swerve.setState(
    //                                   Swerve.DesiredState.aim.with(
    //
    // m_swerve.getHeading().plus(Rotation2d.k180deg))).repeatedly())
    //                       .repeatedly(),
    //                   new WaitCommand(10));
    //             }));

    // PATHING COMMANDS & TRIGGERS

    NamedCommands.registerCommand(
        "ShootAll",
        shootCmd
            .get()
            .repeatedly()
            .alongWith(m_swerve.setState(SwerveDesiredState.aimSOTM).repeatedly()));

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

  public Command getAuto() {

    return m_autoChooser.getSelected();
  }
}
