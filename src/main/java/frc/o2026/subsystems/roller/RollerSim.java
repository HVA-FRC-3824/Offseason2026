// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.roller;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.TalonIO;
import frc.o2026.Configs;
import frc.o2026.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class RollerSim implements RollerIO {
  public MotorIO m_motor;

  public DCMotorSim m_motorModel;

  private final IntakeSimulation m_intakeSimulation;

  public RollerSim(AbstractDriveTrainSimulation driveTrain) {

    m_motor =
        new TalonIO(
            Constants.CanIds.FuelIntakeMotorId, Configs.Roller.RollerConfig, true); // is an X60

    m_motorModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.01102666212, 1.0 // MOI from CAD
                ),
            DCMotor.getKrakenX60(1));

    m_intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrain,
            Inches.of(30 - (1.5 * 2)), // Width
            Inches.of(11.5), // Extension
            IntakeSimulation.IntakeSide.FRONT,
            40 // Capacity
            );

    m_intakeSimulation.addGamePiecesToIntake(8); // Preloads
  }

  @Override
  public void setRoller(AngularVelocity velocity) {
    m_motor.setVelocity(velocity);

    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(m_motorModel.getAngularVelocity());

    if (velocity.gt(RotationsPerSecond.of(1.0))) {
      m_intakeSimulation.startIntake();
    } else {
      m_intakeSimulation.stopIntake();
    }
  }

  @Override
  public void brakeRoller() {
    m_motor.brake();

    m_motorModel.setInputVoltage(m_motor.getAppliedVoltage().in(Volts));
    m_motorModel.update(0.02);

    m_motor.simPeriodic(m_motorModel.getAngularVelocity());
  }

  @Override
  public AngularVelocity getVelocity() {
    return m_motor.getVelocity();
  }
}
