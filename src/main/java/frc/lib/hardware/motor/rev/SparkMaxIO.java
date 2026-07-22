// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor.rev;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.motor.MotorConfig;
import frc.lib.hardware.motor.MotorIO;
import org.littletonrobotics.junction.Logger;

public final class SparkMaxIO implements MotorIO {

  private SparkMax m_motor;
  private SparkMaxConfig m_motorConfig;
  private RelativeEncoder m_encoder;
  private SparkClosedLoopController m_controller;

  private double m_lastRef = 0.0;

  public SparkMaxIO(int id) {

    m_motor = new SparkMax(id, MotorType.kBrushless);
    m_motorConfig = new SparkMaxConfig();
    m_encoder = m_motor.getEncoder();
    m_controller = m_motor.getClosedLoopController();
  }

  public SparkMaxIO(int id, MotorConfig config) {

    this(id);

    config(config);
  }

  public void config(MotorConfig config) {

    m_motorConfig
        .closedLoop
        .pid(config.getP(), config.getI(), config.getD())
        .positionWrappingEnabled(config.isContinuousWrap());

    m_motorConfig
        .inverted(config.isInverted())
        .idleMode(config.isBrakeMode() ? IdleMode.kBrake : IdleMode.kCoast);

    m_motorConfig.smartCurrentLimit((int) config.getStatorCurrent().in(Amps));

    m_motorConfig
        .encoder
        .positionConversionFactor(1.0 / config.getSensorToMechanismRatio())
        .velocityConversionFactor((1.0 / config.getSensorToMechanismRatio()) / 60.0);

    m_motorConfig
        .absoluteEncoder
        .positionConversionFactor((1 / config.getSensorToMechanismRatio()))
        .velocityConversionFactor((1 / config.getSensorToMechanismRatio()) / 60.0);

    // Write the configuration to the motor controller
    m_motor.configure(
        m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  @Override
  public int getId() {

    return m_motor.getDeviceId();
  }

  public void follow(int id, boolean inverted) {

    m_motorConfig.follow(id, inverted);

    m_motor.configure(
        m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void brake() {

    // m_motor.set(0.0);
    m_lastRef = 0.0;

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void setPosition(Angle angle) {

    m_controller.setSetpoint(angle.in(Rotations), ControlType.kPosition);

    m_lastRef = angle.in(Rotations);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void setVelocity(AngularVelocity angleVel) {

    m_controller.setSetpoint(angleVel.in(RotationsPerSecond), ControlType.kVelocity);

    m_lastRef = angleVel.in(RotationsPerSecond);

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public void resetEncoder(Angle angle) {

    m_encoder.setPosition(angle.in(Rotations));

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().name());
  }

  public Voltage getAppliedVoltage() {

    return Voltage.ofRelativeUnits(m_motor.getAppliedOutput() * m_motor.getBusVoltage(), Volt);
  }

  public Voltage getSupplyVoltage() {

    return Voltage.ofRelativeUnits(m_motor.getBusVoltage(), Volt);
  }

  public Angle getPos() {

    return Rotations.of(m_encoder.getPosition());
  }

  public AngularVelocity getVelocity() {

    return RotationsPerSecond.of(m_encoder.getVelocity());
  }

  @Override
  public double getLastReference() {

    return m_lastRef;
  }
}
