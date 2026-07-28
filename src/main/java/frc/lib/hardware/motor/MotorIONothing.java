// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class MotorIONothing implements MotorIO {

  public MotorIONothing() {}

  public void periodic() {}

  public void config(MotorConfig config) {}

  public int getId() {

    return 0;
  }

  public void follow(int id, boolean inverted) {}

  public void brake() {}

  public void setPosition(Angle angle) {}

  public void setVelocity(AngularVelocity angleVel) {}

  public void resetEncoder(Angle angle) {}

  public Voltage getAppliedVoltage() {

    return Volts.of(0.0);
  }

  public Voltage getSupplyVoltage() {

    return Volts.of(0.0);
  }

  public Angle getPos() {

    return Degrees.of(0.0);
  }

  public AngularVelocity getVelocity() {

    return DegreesPerSecond.of(0.0);
  }

  public double getLastReference() {

    return 0.0;
  }
}
