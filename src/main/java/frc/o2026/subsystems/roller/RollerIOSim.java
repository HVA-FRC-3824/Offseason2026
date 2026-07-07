// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.roller;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.hardware.MotorIO;
import frc.lib.hardware.ctre.MotorSimIO;
import frc.o2026.Configs;
import frc.o2026.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class RollerIOSim implements RollerIO {
  public MotorIO m_motor;

  public DCMotorSim m_motorModel;

  private final IntakeSimulation m_intakeSimulation;

  public RollerIOSim(AbstractDriveTrainSimulation driveTrain) {

    m_motor =
        new MotorSimIO(
            Constants.CanIds.FuelIntakeMotorId,
            Configs.Roller.RollerConfig,
            true,
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1), 0.01102666212, 1.0 // MOI from CAD
                    ),
                DCMotor.getKrakenX60(1)),
            1); // is an X60

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
  public void periodic() {

    m_motor.periodic();
  }

  @Override
  public void setRoller(AngularVelocity velocity) {
    m_motor.setVelocity(velocity);

    if (velocity.gt(RotationsPerSecond.of(1.0))) {
      m_intakeSimulation.startIntake();
    } else {
      m_intakeSimulation.stopIntake();
    }
  }

  @Override
  public void brakeRoller() {
    m_motor.brake();
  }

  @Override
  public AngularVelocity getVelocity() {
    return m_motor.getVelocity();
  }
}
