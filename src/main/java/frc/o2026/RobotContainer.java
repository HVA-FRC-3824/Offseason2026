// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.hardware.gyro.GyroIOPigeon;
import frc.lib.hardware.motor.ctre.FlywheelSimIO;
import frc.lib.hardware.motor.ctre.MotorSimIO;
import frc.lib.hardware.motor.ctre.OrchestraOrchestrator;
import frc.lib.hardware.motor.ctre.OrchestraOrchestrator.Song;
import frc.lib.hardware.motor.ctre.TalonIO;
import frc.o2026.subsystems.Flywheel;
import frc.o2026.subsystems.Indexer;
import frc.o2026.subsystems.Intake;
import frc.o2026.subsystems.drivebase.Swerve;
import frc.o2026.subsystems.drivebase.SwerveIOReal;
import frc.o2026.subsystems.drivebase.SwerveIOSim;
import frc.o2026.subsystems.drivebase.objectVision.ObjectCameraIOPhoton;
import frc.o2026.subsystems.drivebase.objectVision.ObjectCameraIOSim;
import frc.o2026.subsystems.drivebase.objectVision.ObjectVision;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOLimelight;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOPhoton;
import frc.o2026.subsystems.drivebase.poseVision.PoseCameraIOSim;
import frc.o2026.subsystems.roller.Roller;
import frc.o2026.subsystems.roller.RollerIOSim;
import frc.o2026.subsystems.roller.RollerIOTalonFX;
import java.util.function.Supplier;

public class RobotContainer extends SubsystemBase {

  private Swerve m_swerve =
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
                  new PoseCameraIOSim(Constants.Vision.LimelightOfDoomAndDespair)));

  private ObjectVision m_odVision =
      new ObjectVision(
          RobotBase.isReal()
              ? new ObjectCameraIOPhoton(Constants.Vision.BackCamConfig)
              : new ObjectCameraIOSim(Constants.Vision.BackCamConfig));

  private Roller m_roller =
      new Roller(RobotBase.isReal() ? new RollerIOTalonFX() : new RollerIOSim());

  private Indexer m_indexer =
      new Indexer(
          RobotBase.isReal()
              ? new TalonIO(Constants.CanIds.IndexerMotorId, Configs.Indexer.BeltConfig)
              : new MotorSimIO(
                  Constants.CanIds.IndexerMotorId,
                  Configs.Indexer.BeltConfig,
                  true,
                  DCMotor.getKrakenX60(1),
                  Constants.SimModels.IndexerMOI,
                  Constants.SimModels.IndexerGearRatio),
          RobotBase.isReal()
              ? new TalonIO(Constants.CanIds.KickerMotorId, Configs.Indexer.KickerConfig)
              : new MotorSimIO(
                  Constants.CanIds.KickerMotorId,
                  Configs.Indexer.KickerConfig,
                  true,
                  DCMotor.getKrakenX60(1),
                  Constants.SimModels.IndexerMOI,
                  Constants.SimModels.IndexerGearRatio));

  private Intake m_intake =
      new Intake(
          RobotBase.isReal()
              ? new TalonIO(
                  Constants.CanIds.IntakePositionLeaderMotorId, Configs.Intake.PivotConfig)
              : new MotorSimIO(
                  Constants.CanIds.IntakePositionLeaderMotorId,
                  Configs.Intake.PivotConfig,
                  true,
                  DCMotor.getKrakenX60(2),
                  Constants.SimModels.IntakeMOI,
                  Constants.SimModels.IntakeGearRatio),
          RobotBase.isReal()
              ? new TalonIO(
                  Constants.CanIds.IntakePositionFollowerMotorId,
                  Configs.Intake.PivotConfig.withInverted(false))
              : new MotorSimIO(
                  Constants.CanIds.IntakePositionFollowerMotorId,
                  Configs.Intake.PivotConfig,
                  true,
                  DCMotor.getKrakenX60(2),
                  Constants.SimModels.IntakeMOI,
                  Constants.SimModels.IntakeGearRatio));

  private Flywheel m_flywheel =
      new Flywheel(
          RobotBase.isReal()
              ? new TalonIO(Constants.CanIds.FlywheelMotorId, Configs.Flywheel.Config)
              : new FlywheelSimIO(
                  Constants.CanIds.FlywheelMotorId,
                  Configs.Flywheel.Config,
                  true,
                  DCMotor.getKrakenX60(2),
                  Constants.SimModels.FlywheelMOI,
                  Constants.SimModels.FlywheelGearRatio),
          RobotBase.isReal()
              ? new TalonIO(Constants.CanIds.FlywheelFollowerMotorId, Configs.Flywheel.Config)
              : new FlywheelSimIO(
                  Constants.CanIds.FlywheelFollowerMotorId,
                  Configs.Flywheel.Config,
                  true,
                  DCMotor.getKrakenX60(2),
                  Constants.SimModels.FlywheelMOI,
                  Constants.SimModels.FlywheelGearRatio));

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

  Limiting m_limit = Limiting.TrigExp;

  private final SendableChooser<Command> m_autoChooser;

  private final Supplier<Command> shootCmd =
      () ->
          Commands.parallel(
              m_flywheel.auto(),
              m_swerve.aimSOTM(),
              m_indexer.on().onlyWhile(this::readyToShoot),
              m_intake.stowed());

  public RobotContainer() {

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    var targetPose = new Pose2d(9, 7.5, Rotation2d.kZero);
    var startPose = new Pose2d(7, 1, Rotation2d.kPi);

    SmartDashboard.putData("bLine to 5,5", m_swerve.bLinePathPose(targetPose));
    SmartDashboard.putData("pp to 5,5", m_swerve.ppPathPose(targetPose));
    SmartDashboard.putData("pid to 5,5", m_swerve.pidPathPose(targetPose));
    SmartDashboard.putData("reset", m_swerve.resetPoseCmd(startPose));

    m_swerve.setDefaultCommand(m_swerve.drive(this::getSpeeds));

    m_driver.a().onTrue(m_swerve.resetGyro());
    m_driver.y().onTrue(m_swerve.fieldCentricityToggle());
    m_driver.x().onTrue(getInit());
    m_driver
        .b()
        .onTrue(new InstantCommand(() -> OrchestraOrchestrator.playSong(Song.GymLeader), m_swerve));

    m_driver
        .leftBumper()
        .whileTrue(
            m_swerve.aimMove(
                () -> new ChassisSpeeds(-0.2, 0.0, 0.0),
                () -> m_odVision.directionToObject(-1),
                true,
                false));

    m_driver.rightTrigger().whileTrue(shootCmd.get());
    m_driver
        .rightTrigger()
        .onFalse(Commands.parallel(m_flywheel.off(), m_intake.stowed(), m_indexer.off()));

    m_driver.rightBumper().whileTrue(m_flywheel.manual(RPM.of(6000.0)));
    m_driver.rightBumper().onFalse(m_flywheel.off());

    m_driver.leftTrigger().whileTrue(Commands.parallel(m_intake.deploy(), m_roller.on()));
    m_driver.leftTrigger().onFalse(m_roller.off());

    m_driver.povUp().onTrue(m_intake.stowed());
    m_driver.povDown().onTrue(m_intake.deploy());

    NamedCommands.registerCommand("ShootRoutine", shootCmd.get().repeatedly());
  }

  private boolean readyToShoot() {

    return m_flywheel.isReady() && m_swerve.isAimed();
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

  public Command getInit() {

    return Commands.parallel(m_swerve.resetSwerveModules()).withName("Init");
  }

  public Command getAuto() {

    return m_autoChooser.getSelected();
  }
}
