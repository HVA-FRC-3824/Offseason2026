// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.hardware.motor.ctre.MotorSimIO;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import org.littletonrobotics.junction.Logger;

public class RollerIOSim extends MotorSimIO implements RollerIO {

  public RollerIOSim() {

    super(
        Constants.CanIds.FuelIntakeMotorId,
        Configs.Roller.RollerConfig,
        true,
        DCMotor.getKrakenX60(1),
        Constants.SimModels.RollerMOI,
        Constants.SimModels.RollerGearRatio);
  }

  @Override
  public void periodic() {

    super.periodic();

    if (getLastReference() >= 10.0) {
      RobotState.setSimIntaking(true);
    } else {
      RobotState.setSimIntaking(false);
    }

    Logger.recordOutput("Sim/intaking", RobotState.isSimIntaking());
  }
}
