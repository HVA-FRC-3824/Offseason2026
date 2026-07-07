// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.rev;

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
import frc.lib.hardware.MotorConfig;
import frc.lib.hardware.MotorIO;
import org.littletonrobotics.junction.Logger;

public final class SparkMaxIO implements MotorIO {

  private SparkMax m_motor;
  private SparkMaxConfig m_motorConfig;
  private RelativeEncoder m_encoder;
  private SparkClosedLoopController m_controller;

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
        .velocityConversionFactor(
            (1.0 / config.getSensorToMechanismRatio())
                / 60.0); // Div by 60 because its in RPM by default

    m_motorConfig
        .absoluteEncoder
        .positionConversionFactor((1 / config.getSensorToMechanismRatio()))
        .velocityConversionFactor(
            (1 / config.getSensorToMechanismRatio())
                / 60.0); // Div by 60 because its in RPM by default

    // Write the configuration to the motor controller
    m_motor.configure(
        m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public int getId() {

    return m_motor.getDeviceId();
  }

  @Override
  public void periodic() {

    Logger.recordOutput(
        "MotorErr/SparkMax " + m_motor.getDeviceId(), m_motor.getLastError().toString());
  }

  public void follow(int id, boolean inverted) {

    m_motorConfig.follow(id, inverted);

    m_motor.configure(
        m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void brake() {

    m_motor.set(0.0);
  }

  @Override
  public void setPercent(double percent) {

    m_motor.set(percent);
  }

  public void setPosition(Angle angle) {

    m_controller.setSetpoint(angle.in(Rotations), ControlType.kPosition);
  }

  public void setVelocity(AngularVelocity angleVel) {

    m_controller.setSetpoint(angleVel.in(RotationsPerSecond), ControlType.kVelocity);
  }

  public void resetEncoder(Angle angle) {

    m_encoder.setPosition(angle.in(Rotations));
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
}
