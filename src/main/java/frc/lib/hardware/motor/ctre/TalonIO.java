// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor.ctre;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.hardware.motor.MotorConfig;
import frc.lib.hardware.motor.MotorIO;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TalonIO implements MotorIO {

  final TalonFX m_motor;

  final Supplier<Angle> m_posSupplier;
  final Supplier<AngularVelocity> m_velSupplier;

  private double m_lastRef = 0.0;

  public TalonIO(int id) {

    m_motor = new TalonFX(id);
    m_posSupplier = m_motor.getPosition().asSupplier();
    m_velSupplier = m_motor.getVelocity().asSupplier();

    OrchestraOrchestrator.addInstrument(m_motor);
  }

  public TalonIO(int id, MotorConfig config) {

    this(id);

    config(config);
  }

  public void config(MotorConfig config) {

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    talonConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrent().in(Amps);
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.getSupplyCurrent().in(Amps) != 0;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrent().in(Amps);
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.getStatorCurrent().in(Amps) != 0;

    talonConfig.MotorOutput.Inverted =
        config.isInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode =
        config.isBrakeMode() ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    talonConfig.Feedback.SensorToMechanismRatio = config.getSensorToMechanismRatio();

    talonConfig.ClosedLoopGeneral.ContinuousWrap = config.isContinuousWrap();

    talonConfig.Slot0.kP = config.getP();
    talonConfig.Slot0.kI = config.getI();
    talonConfig.Slot0.kD = config.getD();
    talonConfig.Slot0.kS = config.getS();
    talonConfig.Slot0.kV = config.getV();
    talonConfig.Slot0.kA = config.getA();

    talonConfig.MotionMagic.MotionMagicCruiseVelocity =
        config.getVelocityLimit().in(RotationsPerSecond);
    talonConfig.MotionMagic.MotionMagicAcceleration =
        config.getAccelerationLimit().in(RotationsPerSecondPerSecond);

    talonConfig.Audio.AllowMusicDurDisable = true;

    var status = m_motor.getConfigurator().apply(talonConfig);

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public int getId() {

    return m_motor.getDeviceID();
  }

  @Override
  public Voltage getAppliedVoltage() {

    return m_motor.getMotorVoltage().getValue();
  }

  @Override
  public Voltage getSupplyVoltage() {

    return m_motor.getSupplyVoltage().getValue();
  }

  @Override
  public Angle getPos() {

    return m_posSupplier.get();
  }

  @Override
  public AngularVelocity getVelocity() {

    return m_velSupplier.get();
  }

  @Override
  public void setVelocity(AngularVelocity angleVel) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status = m_motor.setControl(new MotionMagicVelocityVoltage(angleVel));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    m_lastRef = angleVel.in(RotationsPerSecond);
  }

  @Override
  public void setPosition(Angle angle) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status = m_motor.setControl(new MotionMagicVoltage(angle));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    m_lastRef = angle.in(Rotations);
  }

  @Override
  public void follow(int id, boolean inverted) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status =
        m_motor.setControl(
            new Follower(id, inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public void brake() {

    var status = m_motor.setControl(new NeutralOut());
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    OrchestraOrchestrator.addInstrument(m_motor);

    m_lastRef = 0.0;
  }

  @Override
  public void resetEncoder(Angle angle) {

    var status = m_motor.setPosition(angle);
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public double getLastReference() {

    return m_lastRef;
  }
}
