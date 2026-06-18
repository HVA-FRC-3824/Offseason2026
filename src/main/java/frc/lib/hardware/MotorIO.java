// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {

  // Call this in your updateHardwareInputs function
  // Get the velocity from your motor model (eg FlywheelSim)
  default void simPeriodic(AngularVelocity velocity) {}

  void config(MotorConfig config);

  void follow(int id, boolean inverted);

  void brake();

  void setPosition(Angle angle);

  void setVelocity(AngularVelocity angleVel);

  void resetEncoder(Angle angle);

  Voltage getAppliedVoltage();

  Voltage getSupplyVoltage();

  Angle getPos();

  AngularVelocity getVelocity();
}
