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

  public default void periodic() {}

  public void config(MotorConfig config);

  public int getId();

  public void follow(int id, boolean inverted);

  public void brake();

  public default void setPercent(double percent) {}

  public void setPosition(Angle angle);

  public void setVelocity(AngularVelocity angleVel);

  public void resetEncoder(Angle angle);

  public Voltage getAppliedVoltage();

  public Voltage getSupplyVoltage();

  public Angle getPos();

  public AngularVelocity getVelocity();
}
