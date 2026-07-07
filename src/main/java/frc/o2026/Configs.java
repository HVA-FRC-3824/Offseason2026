// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.hardware.MotorConfig;

public class Configs {

  public static class Vision {

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N4, N1> kSingleTagStdDevs =
        VecBuilder.fill(4, 4, 4, Double.MAX_VALUE);

    public static final Matrix<N4, N1> kMultiTagStdDevs =
        VecBuilder.fill(0.5, 0.5, 0.5, Double.MAX_VALUE);
  }

  public static final class Intake {

    public static final MotorConfig PivotConfig =
        new MotorConfig()
            .withStatorCurrent(Amps.of(60.0))
            .withInverted(true)
            .withBrakeMode(true)
            .withContinuousWrap(false)
            .withP(0.8)
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(6.0))
            .withVelocityLimit(RotationsPerSecond.of(25.0));

    public static final Angle IntakeStowedTurns = Rotations.of(0.8);
    public static final Angle IntakedeployTurns = Rotations.of(10.0);
  }

  public static final class Roller {

    public static final MotorConfig RollerConfig =
        new MotorConfig()
            .withStatorCurrent(Amps.of(120.0))
            .withInverted(false)
            .withBrakeMode(true)
            .withContinuousWrap(false)
            .withP(0.2);

    public static final AngularVelocity IntakeDriveTurnsPerSec = RotationsPerSecond.of(600.0);
  }

  public static final class Indexer {

    public static final AngularVelocity BeltTurnsPerSec = RotationsPerSecond.of(120);
    public static final AngularVelocity KickerWheelTurnsPerSec = RotationsPerSecond.of(120);

    public static final MotorConfig BeltConfig =
        new MotorConfig()
            .withStatorCurrent(Amps.of(85.0))
            .withInverted(true)
            .withBrakeMode(false)
            .withContinuousWrap(false)
            .withP(0.3)
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(120))
            .withVelocityLimit(RotationsPerSecond.of(30));

    public static final MotorConfig KickerConfig =
        new MotorConfig()
            .withStatorCurrent(Amps.of(85.0))
            .withInverted(false)
            .withBrakeMode(false)
            .withContinuousWrap(false)
            .withP(0.3)
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(120))
            .withVelocityLimit(RotationsPerSecond.of(120));
  }

  public static final class Flywheel {

    public static final MotorConfig Config =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(40.0))
            .withStatorCurrent(Amps.of(85.0))
            .withInverted(false)
            .withBrakeMode(false)
            .withContinuousWrap(false)
            .withP(0.55)
            .withV(0.13)
            .withA(0.99);

    public static final AngularVelocity CloseSpeed = RotationsPerSecond.of(45.0); // 90 in
    public static final AngularVelocity MiddleSpeed = RotationsPerSecond.of(51.0); // 120 in
    public static final AngularVelocity FieldPassSpeed = RotationsPerSecond.of(85.0);
    public static final AngularVelocity NeutralPassSpeed = RotationsPerSecond.of(65.0);

    // as a percentage of the reference (10tps tolerance at 100tps reference)
    public static final double SpunUpTolerance = 10.0;
  }

  public static final class Chassis {

    public static final MotorConfig DriveConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(40.0))
            .withStatorCurrent(Amps.of(80.0))
            .withBrakeMode(true)
            .withInverted(false)
            .withContinuousWrap(false)
            .withP(0.1)
            .withVelocityLimit(RotationsPerSecond.of(10))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(10));

    public static final MotorConfig TurnConfig =
        new MotorConfig()
            .withSupplyCurrent(Amps.of(40.0))
            .withStatorCurrent(Amps.of(20.0))
            .withInverted(true)
            .withBrakeMode(true)
            .withContinuousWrap(true)
            .withP(3.5)
            .withD(0.05)
            .withSensorToMechanismRatio(21.5)
            .withVelocityLimit(RotationsPerSecond.of(100))
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(200));

    public static final LinearVelocity MaximumLinear = FeetPerSecond.of(12.0);

    public static final LinearAcceleration MaximumLinearAcceleration =
        FeetPerSecondPerSecond.of(12.0);

    public static final AngularVelocity MaximumAngularVelocity = RadiansPerSecond.of(4 * Math.PI);

    public static final AngularAcceleration MaximumAngularAcceleration =
        RadiansPerSecondPerSecond.of(4 * Math.PI);

    public static final double TranslateExponentialPower = 3.0;
    public static final double AngularExponentialPower = 4.0;

    public static final PathConstraints constraints =
        new PathConstraints(
            Configs.Chassis.MaximumLinear,
            Configs.Chassis.MaximumLinearAcceleration,
            Configs.Chassis.MaximumAngularVelocity,
            Configs.Chassis.MaximumAngularAcceleration);
  }
}
