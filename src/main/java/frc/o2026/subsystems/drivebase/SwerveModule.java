// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase;

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
import edu.wpi.first.units.measure.Angle;
import frc.lib.hardware.motor.MotorIO;
import frc.lib.hardware.motor.ctre.TalonIO;
import frc.lib.hardware.motor.rev.SparkMaxIO;
import frc.o2026.Configs;
import frc.o2026.Constants;

public class SwerveModule {
  private final MotorIO m_drivingMotor;
  private final MotorIO m_angleMotor;
  private final CANcoder angleAbsoluteEncoder;

  public SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) {

    m_drivingMotor = new TalonIO(driveMotorCanId);
    m_angleMotor = new SparkMaxIO(angleMotorCanId);
    angleAbsoluteEncoder = new CANcoder(angleEncoderCanId);

    m_drivingMotor.config(Configs.Chassis.DriveConfig);

    m_angleMotor.config(Configs.Chassis.TurnConfig);

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

    m_drivingMotor.setVelocity(
        RotationsPerSecond.of(
            desiredState.speedMetersPerSecond / Constants.Chassis.DriveMotorConversion));

    m_angleMotor.setPosition(desiredState.angle.getMeasure());

    // if (desiredState.angle.getMeasure() == m_angleMotor.getPos()) {
    //   m_angleMotor.brake();
    // }

    // if (desiredState.speedMetersPerSecond == 0.0) {
    //   m_drivingMotor.brake();
    // }
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

  public void setWheelAngleToForward(Angle forwardAngle) {

    m_angleMotor.resetEncoder(
        angleAbsoluteEncoder.getAbsolutePosition().getValue().minus(forwardAngle));
  }
}
