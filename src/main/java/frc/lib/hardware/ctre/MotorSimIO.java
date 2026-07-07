// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.ctre;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
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
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.MotorConfig;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.SimBattery;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MotorSimIO implements MotorIO {

  private final TalonFX m_motor;

  private final DCMotorSim m_motorSimModel;

  private final Supplier<Angle> m_posSupplier;
  private final Supplier<AngularVelocity> m_velSupplier;

  private int m_gearRatio;

  private MotorType m_simType;

  public MotorSimIO(
      int id, MotorConfig config, boolean isX60, DCMotorSim motorSimModel, int gearRatio) {

    m_motor = new TalonFX(id);

    m_motorSimModel = motorSimModel;

    m_posSupplier = m_motor.getPosition().asSupplier();
    m_velSupplier = m_motor.getVelocity().asSupplier();

    m_simType = isX60 ? MotorType.KrakenX60 : MotorType.KrakenX44;

    m_gearRatio = gearRatio;

    SimBattery.registerDevice(m_motor.getSupplyCurrent().asSupplier());

    config(config);
  }

  public void config(MotorConfig config) {

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    talonConfig.CurrentLimits.SupplyCurrentLimit = config.getSupplyCurrent().in(Amps);
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.getStatorCurrent().in(Amps);
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

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

  @Override
  public int getId() {

    return m_motor.getDeviceID();
  }

  @Override
  public void periodic() {

    var talonFXSim = m_motor.getSimState();
    talonFXSim.setMotorType(m_simType);

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(SimBattery.getSupplyVoltage().in(Volts));

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInput(
        MathUtil.clamp(
            motorVoltage.in(Volts),
            -SimBattery.getSupplyVoltage().in(Volts),
            SimBattery.getSupplyVoltage().in(Volts)));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(m_gearRatio));
    talonFXSim.setRotorVelocity(m_motorSimModel.getOutput(0) * m_gearRatio);
  }

  @Override
  public Voltage getAppliedVoltage() {

    return Volts.of(m_motorSimModel.getInputVoltage());
  }

  @Override
  public Voltage getSupplyVoltage() {

    return Volts.of(RobotController.getBatteryVoltage());
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

    var status = m_motor.setControl(new MotionMagicVelocityVoltage(angleVel));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public void setPosition(Angle angle) {

    var status = m_motor.setControl(new MotionMagicVoltage(angle));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public void follow(int id, boolean inverted) {

    var status =
        m_motor.setControl(
            new Follower(id, inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public void brake() {

    var status = m_motor.setControl(new VelocityDutyCycle(0.0));
    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }

  @Override
  public void resetEncoder(Angle angle) {

    m_motorSimModel.setAngle(angle.in(Radians));
    var status = m_motor.setPosition(angle);

    Logger.recordOutput("MotorErr/Talon " + m_motor.getDeviceID(), status.toString());
  }
}
