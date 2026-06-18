// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.lib.hardware.rev.SparkMaxIO;
import frc.o2026.Constants;

public class SwerveModule {
  private final MotorIO m_drivingMotor;
  private final MotorIO m_angleMotor;
  private final CANcoder angleAbsoluteEncoder;

  public SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) {

    m_drivingMotor = new TalonIO(driveMotorCanId);
    m_angleMotor = new SparkMaxIO(angleMotorCanId);
    angleAbsoluteEncoder = new CANcoder(angleEncoderCanId);

    m_drivingMotor.config(Constants.Chassis.DriveConfig);

    m_angleMotor.config(Constants.Chassis.TurnConfig);

    angleAbsoluteEncoder
        .getConfigurator()
        .apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withMagnetOffset(0.0)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.optimize(getPosition().angle);

    m_drivingMotor.setVelocity(
        RotationsPerSecond.of(
            desiredState.speedMetersPerSecond * Constants.Chassis.DriveMotorConversion));
    m_angleMotor.setPosition(desiredState.angle.getMeasure());
  }

  public SwerveModuleState getState() {

    return new SwerveModuleState(
        m_drivingMotor.getVelocity().in(RotationsPerSecond)
            * Constants.Chassis.DriveMotorConversion,
        new Rotation2d(m_angleMotor.getPos()));
  }

  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        m_drivingMotor.getPos().in(Rotations) * Constants.Chassis.DriveMotorConversion,
        new Rotation2d(m_angleMotor.getPos()));
  }

  public void resetEncoders() {
    m_drivingMotor.resetEncoder(Degrees.of(0.0));
  }

  public void setWheelAngleToForward(double forwardAngleDeg) {

    double moveDegrees = getAbsoluteEncoderAngle() - forwardAngleDeg;

    m_angleMotor.resetEncoder(Degrees.of(moveDegrees));
    m_angleMotor.setPosition(Rotations.of(0.0));
  }

  private double getAbsoluteEncoderAngle() {
    return angleAbsoluteEncoder.getAbsolutePosition().getValue().in(Degrees);
  }
}
