// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.Alliance;
import frc.lib.hardware.vision.VisionConfig;
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

    public static final Translation2d HubCenter =
        Alliance.isRed()
            ? Constants.Field.RedHub.getTranslation().toTranslation2d()
            : Constants.Field.BlueHub.getTranslation().toTranslation2d();
        
    public static final Translation2d HubForward = Alliance.isRed() ? new Translation2d(-1, 0) : new Translation2d(1, 0);

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

  public static final class Vision {

    public static final VisionConfig BackCamConfig =
        new VisionConfig(
            "RealCam",
            new Transform3d(
                new Translation3d(
                    Inches.of(-15.0), // forwards
                    Inches.of(0.0), // right
                    Inches.of(10.0)), // up
                new Rotation3d(
                    Degrees.of(0.0), // roll
                    Degrees.of(10.0), // pitch
                    Degrees.of(180.0)))); // yaw

    public static final VisionConfig LimelightOfDoomAndDespair =
        new VisionConfig(
            "limelight-athreeg",
            new Transform3d(
                new Translation3d(
                    Inches.of(0.0), // forwards
                    Inches.of(-15.0), // right
                    Inches.of(5.0)), // up
                new Rotation3d(
                    Degrees.of(90.0), // roll
                    Degrees.of(0.0), // pitch
                    Degrees.of(90.0)))); // yaw

    public static final VisionConfig FrontCamConfig =
        new VisionConfig(
            "BadCam",
            new Transform3d(
                new Translation3d(
                    Inches.of(15.0), // forwards
                    Inches.of(-3.0), // right
                    Inches.of(5.0)), // up
                new Rotation3d(
                    Degrees.of(90.0), // roll
                    Degrees.of(-20.0), // pitch
                    Degrees.of(0.0)))); // yaw

    public static final VisionConfig WebCam =
        new VisionConfig(
            "LightCam",
            new Transform3d(
                new Translation3d(
                    Inches.of(-1.0), // forwards
                    Inches.of(15.0), // right
                    Inches.of(5.0)), // up
                new Rotation3d(
                    Degrees.of(0.0), // roll
                    Degrees.of(-20.0), // pitch
                    Degrees.of(-90.0)))); // yaw

    public static AprilTagFieldLayout TagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  }

  public static final class SimModels {

    // The follower motors dont need to/shouldn't modify the state of other sim motor models
    public static final DCMotorSim MockFollowerModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));

    public static final int FlywheelGearRatio = 1;
    public static final double FlywheelMOI = 0.02;

    public static final FlywheelSim Flywheel =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), FlywheelMOI, FlywheelGearRatio),
            DCMotor.getKrakenX60(2));

    public static final int IndexerGearRatio = 25;
    public static final double IndexerMOI = 0.021;

    public static final DCMotorSim Indexer =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), IndexerMOI, IndexerGearRatio // MOI from CAD
                ),
            DCMotor.getKrakenX60(1));

    public static final int IntakeGearRatio = 25;
    public static final double IntakeMOI = 0.070408789;

    public static final DCMotorSim Intake =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), IntakeMOI, IntakeGearRatio),
            DCMotor.getKrakenX60(2));

    public static final int RollerGearRatio = 1;
    public static final double RollerMOI = 0.01102666212;
  }

  public static final class Chassis {

    // NOTE: The absolute encoder range is 0.5 to -0.5
    // These are the absolute encoder values that correspond to the wheels facing "forward"
    public static final Angle FrontRightForwardsAngle = Rotations.of(-1.7334);
    public static final Angle FrontLeftForwardsAngle = Rotations.of(-0.4800);
    public static final Angle BackRightForwardsAngle = Rotations.of(0.05908);
    public static final Angle BackLeftForwardsAngle = Rotations.of(0.08325);

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
