// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

// It shares a name with the wpilibj name FlywheelSim... we fix this
public class FlywheelIOSim implements FlywheelIO {
  private AngularVelocity m_lastinput = RotationsPerSecond.of(0.0);

  private Angle m_simPosTurns = Rotations.of(0.0);

  private int m_offsetIndex = 0;

  public final MotorIO m_motor;
  public final FlywheelSim m_motorModel;

  public SwerveDriveSimulation m_simDrive;
  public IntakeSimulation m_intakeSimulation;

  public FlywheelIOSim(SwerveDriveSimulation simDrive, IntakeSimulation intakeSimulation) {
    m_motor = new TalonIO(Constants.CanIds.FlywheelMotorId, Constants.Flywheel.Config, true);

    m_motorModel =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.021, 1.0),
            DCMotor.getKrakenX60(2),
            0.2);

    m_simDrive = simDrive;
    m_intakeSimulation = intakeSimulation;
  }

  @Override
  public void setFlywheel(AngularVelocity velocity) {
    m_motor.setVelocity(velocity);

    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(m_motorModel.getAngularVelocity());
  }

  public void stopFlywheel() {
    m_motor.brake();

    m_motor.setPosition(m_simPosTurns);

    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(RadiansPerSecond.of(m_motorModel.getAngularVelocityRadPerSec()));
  }

  public AngularVelocity getReference() {
    return m_lastinput;
  }

  public AngularVelocity getMeasured() {
    return m_motor.getVelocity();
  }
}
