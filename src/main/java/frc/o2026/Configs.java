// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.o2026;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
import edu.wpi.first.wpilibj.RobotBase;
import frc.o2026.Constants.RobotImpl;
import frc.shared.hardware.motor.MotorConfig;

public class Configs {

  public static class Vision {

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N4, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 4, 4);

    public static final Matrix<N4, N1> kMultiTagStdDevs = VecBuilder.fill(0.4, 0.4, 0.4, 1.0);
  }

  public static final class Intake {

    public static final MotorConfig PivotConfig =
        new MotorConfig()
            .withInverted(true)
            .withBrakeMode(true)
            .withContinuousWrap(false)
            .withP(0.8)
            .withAccelerationLimit(RotationsPerSecondPerSecond.of(6.0))
            .withVelocityLimit(RotationsPerSecond.of(25.0));

    public static final Angle IntakeStowedTurns = Rotations.of(0.8);
    public static final Angle IntakeDeployTurns = Rotations.of(10.0);
  }

  public static final class Roller {

    public static final MotorConfig RollerConfig =
        new MotorConfig()
            .withStatorCurrent(Amps.of(120.0))
            .withInverted(false)
            .withBrakeMode(true)
            .withContinuousWrap(false)
            .withP(0.2);

    public static final AngularVelocity IntakeTurnsPerSec = RotationsPerSecond.of(120.0);
  }

  public static final class Indexer {

    public static final AngularVelocity BeltTurnsPerSec = RotationsPerSecond.of(120);
    public static final AngularVelocity KickerWheelTurnsPerSec = RotationsPerSecond.of(120);

    public static final MotorConfig BeltConfig =
        new MotorConfig()
            .withInverted(true)
            .withBrakeMode(false)
            .withContinuousWrap(false)
            .withP(0.3);

    public static final MotorConfig KickerConfig =
        new MotorConfig()
            .withInverted(false)
            .withBrakeMode(false)
            .withContinuousWrap(false)
            .withP(0.3);
  }

  public static final class Flywheel {

    public static final MotorConfig Config =
        RobotBase.isReal()
            ? new MotorConfig()
                .withInverted(false)
                .withBrakeMode(false)
                .withContinuousWrap(false)
                .withP(0.55)
                .withV(0.13)
                .withA(0.99)
            : new MotorConfig()
                .withInverted(false)
                .withBrakeMode(false)
                .withContinuousWrap(false)
                .withP(10.0);

    public static final AngularVelocity CloseSpeed = RotationsPerSecond.of(45.0); // 90 in
    public static final AngularVelocity MiddleSpeed = RotationsPerSecond.of(51.0); // 120 in
    public static final AngularVelocity FieldPassSpeed = RotationsPerSecond.of(85.0);
    public static final AngularVelocity NeutralPassSpeed = RotationsPerSecond.of(65.0);

    // as a percentage of the reference (10tps tolerance at 100tps reference)
    public static final double SpunUpTolerance = 10.0;
  }

  public static final class Chassis {

    public static final double IntakeAssistRotationPower = 0.9;
    public static final LinearVelocity IntakeAssistSpeed = MetersPerSecond.of(0.2);

    public static final MotorConfig DriveConfig =
        Constants.Impl == RobotImpl.DevBot
            ? new MotorConfig()
                .withSupplyCurrent(Amps.of(40.0))
                .withStatorCurrent(Amps.of(80.0))
                .withBrakeMode(false)
                .withInverted(false)
                .withContinuousWrap(false)
                .withP(0.07)
                .withV(0.0)
                .withVelocityLimit(RotationsPerSecond.of(10))
                .withAccelerationLimit(RotationsPerSecondPerSecond.of(10))
            : new MotorConfig()
                .withSupplyCurrent(Amps.of(40.0))
                .withStatorCurrent(Amps.of(80.0))
                .withBrakeMode(true)
                .withInverted(false)
                .withContinuousWrap(false)
                .withP(0.03)
                .withI(1.5)
                .withV(0.12877);

    public static final MotorConfig TurnConfig =
        Constants.Impl == RobotImpl.DevBot
            ? new MotorConfig()
                .withSupplyCurrent(Amps.of(20.0))
                .withStatorCurrent(Amps.of(40.0))
                .withInverted(true)
                .withBrakeMode(true)
                .withContinuousWrap(true)
                .withP(1.0)
                .withD(0.0)
                .withSensorToMechanismRatio(21.5)
                .withVelocityLimit(RotationsPerSecond.of(150))
                .withAccelerationLimit(RotationsPerSecondPerSecond.of(200))
            : new MotorConfig()
                .withSupplyCurrent(Amps.of(20.0))
                .withStatorCurrent(Amps.of(40.0))
                .withInverted(true)
                .withBrakeMode(true)
                .withContinuousWrap(true)
                .withP(12.0)
                .withD(0.2)
                .withSensorToMechanismRatio(150.0 / 7.0)
                .withVelocityLimit(RotationsPerSecond.of(150))
                .withAccelerationLimit(RotationsPerSecondPerSecond.of(200));

    public static LinearVelocity MaximumLinear = FeetPerSecond.of(12.0);
    public static LinearAcceleration MaximumLinearAcceleration = FeetPerSecondPerSecond.of(12.0);
    public static AngularVelocity MaximumAngularVelocity = RadiansPerSecond.of(4 * Math.PI);
    public static AngularAcceleration MaximumAngularAcceleration =
        RadiansPerSecondPerSecond.of(4 * Math.PI);

    static {
      if (Constants.Impl == RobotImpl.DevBot) {
        MaximumLinear = MaximumLinear.div(2.0);
        MaximumLinearAcceleration = MaximumLinearAcceleration.div(2.0);
        MaximumAngularVelocity = MaximumAngularVelocity.div(2.0);
        MaximumAngularAcceleration = MaximumAngularAcceleration.div(2.0);
      }
    }

    public static final double TranslateExponentialPower = 3.0;
    public static final double AngularExponentialPower = 2.0;

    public static final PathConstraints constraints =
        new PathConstraints(
            Configs.Chassis.MaximumLinear,
            Configs.Chassis.MaximumLinearAcceleration,
            Configs.Chassis.MaximumAngularVelocity,
            Configs.Chassis.MaximumAngularAcceleration);
  }
}
