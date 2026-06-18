// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.ctre;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.hardware.MotorConfig;
import frc.lib.hardware.MotorIO;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TalonIO implements MotorIO {

  final TalonFX m_motor;

  final Supplier<Angle> m_posSupplier;
  final Supplier<AngularVelocity> m_velSupplier;

  final TalonFXSimState m_motorSim;

  Angle m_simPos;

  public TalonIO(int id, boolean isX60) {

    m_motor = new TalonFX(id);
    m_posSupplier = m_motor.getPosition().asSupplier();
    m_velSupplier = m_motor.getVelocity().asSupplier();

    if (RobotBase.isSimulation()) {

      m_motorSim = m_motor.getSimState();
      m_motorSim.setMotorType(isX60 ? MotorType.KrakenX60 : MotorType.KrakenX44);

      m_simPos = Rotations.of(0.0);

    } else {

      m_motorSim = null;
    }

    OrchestraOrchestrator.addInstrument(m_motor);
  }

  public TalonIO(int id, MotorConfig config, boolean isX60) {

    this(id, isX60);

    config(config);
  }

  public TalonIO(int id, MotorConfig config) {

    this(id, config, true);
  }

  public TalonIO(int id) {

    this(id, true);
  }

  public void config(MotorConfig config) {

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrent().in(Amps);
    // talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.supplyCurrent().in(Amps) == 0.0;
    // talonConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrent().in(Amps);
    // talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.statorCurrent().in(Amps) == 0.0;

    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = false;

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

    var status = m_motor.getConfigurator().apply(talonConfig);

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  public void simPeriodic(AngularVelocity velocity) {

    m_simPos = m_simPos.plus(Rotations.of(velocity.in(RotationsPerSecond) * 0.02));
    m_motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_motorSim.setRawRotorPosition(m_simPos);
    m_motorSim.setRotorVelocity(velocity.in(RotationsPerSecond));
  }

  public Voltage getAppliedVoltage() {

    return RobotBase.isSimulation()
        ? m_motorSim.getMotorVoltageMeasure()
        : m_motor.getMotorVoltage().getValue();
  }

  public Voltage getSupplyVoltage() {

    return RobotBase.isSimulation()
        ? Volts.of(RobotController.getBatteryVoltage())
        : m_motor.getSupplyVoltage().getValue();
  }

  public Angle getPos() {

    return m_posSupplier.get();
  }

  public AngularVelocity getVelocity() {

    return m_velSupplier.get();
  }

  public void setVelocity(AngularVelocity angleVel) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status = m_motor.setControl(new MotionMagicVelocityVoltage(angleVel));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  public void setPosition(Angle angle) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status = m_motor.setControl(new MotionMagicVoltage(angle));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  public void follow(int id, boolean inverted) {

    OrchestraOrchestrator.removeInstrument(m_motor.getDeviceID());

    var status =
        m_motor.setControl(
            new Follower(id, inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  public void brake() {

    var status = m_motor.setControl(new VelocityDutyCycle(0.0));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    OrchestraOrchestrator.addInstrument(m_motor);
  }

  public void resetEncoder(Angle angle) {

    var status = m_motor.setPosition(angle);
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());

    if (RobotBase.isSimulation()) m_simPos = angle;
  }
}
