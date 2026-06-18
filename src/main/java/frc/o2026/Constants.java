// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.hardware.MotorConfig;
import java.util.ArrayList;
import java.util.List;

public final class Constants {

  public static final class Field {
    /// *** Field Dimensions *** ///
    public static final double FieldLengthMeters = Units.inchesToMeters(652.11); // 16.56 meters
    public static final double FieldWidthMeters = Units.inchesToMeters(317.69); //  8.07 meters

    public static final double AllianceWallToAllianceZoneMeters = Units.inchesToMeters(182.11);

    public static final double HubHeightMeters = Units.inchesToMeters(72.0);

    /// *** Field Locations *** ///

    public static final Pose3d BlueHub =
        new Pose3d(
            AllianceWallToAllianceZoneMeters,
            FieldWidthMeters / 2,
            HubHeightMeters,
            new Rotation3d());
    public static final Pose3d RedHub =
        new Pose3d(
            FieldLengthMeters - AllianceWallToAllianceZoneMeters,
            FieldWidthMeters / 2,
            HubHeightMeters,
            new Rotation3d());

    // For passing we want to aim towards the inside of our alliance zone or towards the neutral
    // zone whichever is closer
    // Either way we want the balls to be going as close to our alliance zone as possible, so aim
    // for that
    // - "Aim for the stars and maybe you'll reach the neutral zone" or something like that...

    public static final Pose2d BlueAllianceZoneClose =
        new Pose2d(AllianceWallToAllianceZoneMeters, FieldWidthMeters / 4, new Rotation2d());
    public static final Pose2d BlueAllianceZoneFar =
        new Pose2d(
            AllianceWallToAllianceZoneMeters,
            FieldWidthMeters - (FieldWidthMeters / 4),
            new Rotation2d());

    public static final Pose2d RedAllianceZoneClose =
        new Pose2d(
            FieldLengthMeters - AllianceWallToAllianceZoneMeters,
            FieldWidthMeters / 4,
            new Rotation2d());
    public static final Pose2d RedAllianceZoneFar =
        new Pose2d(
            FieldLengthMeters - AllianceWallToAllianceZoneMeters,
            FieldWidthMeters - (FieldWidthMeters / 4),
            new Rotation2d());
  }

  public static final class MotorConfigs {
    public static final Current MaxStator = Amps.of(200.0);
    public static final Current MaxSupply = Amps.of(40.0);
  }

  public static final class Vision {

    public static final String kCameraName1 = "SigmaKamera";
    public static final String kCameraName2 = "RightCam";

    public static final Transform3d RobotToCam1 =
        new Transform3d(
            new Translation3d(
                Inches.of(-15.0), // forwards
                Inches.of(0.0), // right
                Inches.of(0.0)), // up
            new Rotation3d(
                Degrees.of(0.0), // roll
                Degrees.of(0.0), // pitch
                Degrees.of(180.0))); // yaw

    // some of these probably need to be flipped
    public static final Transform3d kRobotToCam2 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
            new Rotation3d());

    public static AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N4, N1> kSingleTagStdDevs =
        VecBuilder.fill(
            4,
            4,
            4,
            Double.MAX_VALUE); // TODO: make these NOT MAX_VALUE and find a way to reset gyro off of
    // vision
    public static final Matrix<N4, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, 0.5, Double.MAX_VALUE);
  }

  public static final class Intake {

    public static final MotorConfig PivotConfig = new MotorConfig();

    static {
      PivotConfig.withStatorCurrent(Amps.of(60.0))
          .withInverted(true)
          .withBrakeMode(true)
          .withContinuousWrap(false)
          .withP(0.8)
          .withAccelerationLimit(RotationsPerSecondPerSecond.of(6.0))
          .withVelocityLimit(RotationsPerSecond.of(25.0));
    }

    public static final Angle IntakeStowedTurns = Rotations.of(0.8);
    public static final Angle IntakeDeployedTurns = Rotations.of(10.0);
  }

  public static final class Roller {

    public static final MotorConfig RollerConfig = new MotorConfig();

    static {
      RollerConfig.withStatorCurrent(Amps.of(120.0))
          .withInverted(false)
          .withBrakeMode(true)
          .withContinuousWrap(false)
          .withP(0.2);
    }

    public static final AngularVelocity IntakeDriveTurnsPerSec = RotationsPerSecond.of(600.0);
  }

  public static final class Indexer {

    public static final AngularVelocity BeltTurnsPerSec = RotationsPerSecond.of(120);
    public static final AngularVelocity KickerWheelTurnsPerSec = RotationsPerSecond.of(120);

    public static final MotorConfig BeltConfig = new MotorConfig();

    static {
      BeltConfig.withStatorCurrent(Amps.of(85.0))
          .withInverted(true)
          .withBrakeMode(false)
          .withContinuousWrap(false)
          .withP(0.3)
          .withAccelerationLimit(RotationsPerSecondPerSecond.of(120))
          .withVelocityLimit(RotationsPerSecond.of(30));
    }

    public static final MotorConfig KickerConfig = new MotorConfig();

    static {
      KickerConfig.withStatorCurrent(Amps.of(85.0))
          .withInverted(false)
          .withBrakeMode(false)
          .withContinuousWrap(false)
          .withP(0.3)
          .withAccelerationLimit(RotationsPerSecondPerSecond.of(120))
          .withVelocityLimit(RotationsPerSecond.of(120));
    }
  }

  public static final class Flywheel {

    public static final MotorConfig Config = new MotorConfig();

    static {
      Config.withStatorCurrent(Amps.of(85.0))
          .withInverted(false)
          .withBrakeMode(false)
          .withContinuousWrap(false)
          .withP(0.55)
          .withV(0.13)
          .withA(0.99);
    }

    public static final AngularVelocity CloseSpeed = RotationsPerSecond.of(45.0); // 90 in
    public static final AngularVelocity MiddleSpeed = RotationsPerSecond.of(51.0); // 120 in
    public static final AngularVelocity FieldPassSpeed = RotationsPerSecond.of(85.0);
    public static final AngularVelocity NeutralPassSpeed = RotationsPerSecond.of(65.0);

    // as a percentage of the reference (3tps tolerance at 100tps reference)
    public static final double SpunUpTolerance = 3.0;
  }

  public static final class Chassis {

    public static final MotorConfig DriveConfig = new MotorConfig();

    static {
      DriveConfig
          // .withSupplyLimit(Amps.of(40.0))
          // .withStatorLimit(Amps.of(100.0))
          .withBrakeMode(true)
          .withInverted(false)
          .withContinuousWrap(false)
          .withP(0.1)
          // .withV(0.12)
          .withA(0.0)
          .withVelocityLimit(RotationsPerSecond.of(100)) // Max free speed
          .withAccelerationLimit(RotationsPerSecondPerSecond.of(200)); // reach in 0.5 sec
    }

    public static final MotorConfig TurnConfig = new MotorConfig();

    static {
      TurnConfig.withStatorCurrent(Amps.of(20.0))
          .withSupplyCurrent(Amps.of(40.0))
          .withInverted(true)
          .withBrakeMode(true)
          .withContinuousWrap(true)
          .withP(0.01)
          .withD(0.0002)
          .withSensorToMechanismRatio(21.5)
          .withVelocityLimit(RotationsPerSecond.of(20))
          .withAccelerationLimit(RotationsPerSecondPerSecond.of(200));
    }

    // NOTE: The absolute encoder range is 0.5 to -0.5
    // These are the absolute encoder values that correspond to the wheels facing "forward"
    public static final Angle FrontRightForwardsAngle =
        Rotations.of(-0.1796); // Degrees.of( 0.3824 - 0.5); // WE ARE SO COOKED
    public static final Angle FrontLeftForwardsAngle =
        Rotations.of(-0.4775); // Degrees.of( 0.408691);
    public static final Angle BackRightForwardsAngle =
        Rotations.of(0.05908); // Degrees.of(-0.11377);
    public static final Angle BackLeftForwardsAngle = Rotations.of(0.07); // Degrees.of(-0.025146);

    public static final LinearVelocity MaximumLinear = FeetPerSecond.of(12.0);
    public static final LinearAcceleration MaximumLinearAcceleration =
        FeetPerSecondPerSecond.of(12.0);

    public static final AngularVelocity MaximumAngularVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static final AngularAcceleration MaximumAngularAcceleration =
        RadiansPerSecondPerSecond.of(4 * Math.PI);

    public static final double TranslateExponentialPower = 3.0;
    public static final double AngularExponentialPower = 4.0;

    public static final Distance WheelBaseMeters = Inches.of(30.0);
    public static final Distance TrackWidthMeters = Inches.of(30.0);

    public static final double DriveMotorReduction = 6.75;
    public static final Distance WheelDiameter = Meters.of(0.098022); // meters
    public static final Distance WheelCircumference = WheelDiameter.times(Math.PI);
    public static final double DriveMotorConversion =
        WheelCircumference.div(DriveMotorReduction).in(Meters); // Meters per motor turn

    public static final ArrayList<SwerveModuleState> XishStates =
        new ArrayList<>(
            List.of(
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)), // FL
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)), // FR
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(225.0)), // BL
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(315.0)) // BR
                ));

    public static final Translation2d[] ModulePositions = {
      new Translation2d(WheelBaseMeters.div(2.0), TrackWidthMeters.div(2.0)), // Front Left
      new Translation2d(
          WheelBaseMeters.div(2.0), TrackWidthMeters.div(2.0).times(-1.0)), // Front Right
      new Translation2d(
          WheelBaseMeters.div(2.0).times(-1.0), TrackWidthMeters.div(2.0)), // Back Left
      new Translation2d(
          WheelBaseMeters.div(2.0).times(-1.0), TrackWidthMeters.div(2.0).times(-1.0)) // Back Right
    };

    public static final SwerveDriveKinematics Kinematics =
        new SwerveDriveKinematics(ModulePositions);

    public static final PathConstraints constraints =
        new PathConstraints(
            Constants.Chassis.MaximumLinear,
            Constants.Chassis.MaximumLinearAcceleration,
            Constants.Chassis.MaximumAngularVelocity,
            Constants.Chassis.MaximumAngularAcceleration);
  }

  public static final class CanIds {

    public static final int FrontLeftDriveId = 31; // Kraken X60
    public static final int FrontLeftTurnId = 32; // Kraken X44 / NEO v1.2
    public static final int FrontLeftEncoderId = 33; // CANCoder

    public static final int FrontRightDriveId = 21; // Kraken X60
    public static final int FrontRightTurnId = 22; // Kraken X44 / NEO v1.2
    public static final int FrontRightEncoderId = 23; // CANCoder

    public static final int BackLeftDriveId = 11; // Kraken X60
    public static final int BackLeftTurnId = 12; // Kraken X44 / NEO v1.2
    public static final int BackLeftEncoderId = 13; // CANCoder

    public static final int BackRightDriveId = 04; // Kraken X60
    public static final int BackRightTurnId = 02; // Kraken X44 / NEO v1.2
    public static final int BackRightEncoderId = 03; // CANCoder

    public static final int PigeonGyroId = 05; // CTR Pigeon 2.0

    public static final int IntakePositionFollowerMotorId = 42; // Kraken X44
    public static final int IntakePositionLeaderMotorId = 41; // Kraken X44
    public static final int FuelIntakeMotorId = 40; // Kraken X60

    public static final int IndexerMotorId = 50; // Kraken X60
    public static final int KickerMotorId = 51; // Kraken X44

    public static final int FlywheelMotorId = 52; // Kraken X60
    public static final int FlywheelFollowerMotorId = 53; // Kraken X60

    public static final int PdhId = 60; // PDH
  }

  public static final class Pwm {
    // PWM Ports
    public static final int ActuatorPort = 1;

    public static final int LedUnderGlowPort = 9;
    public static final int LedTurretPort = 7;
  }

  public static final class Usb {
    // drive Input Configurations
    public static final int DrivePort = 0;
    public static final int OperatorPort = 1;
  }
}
