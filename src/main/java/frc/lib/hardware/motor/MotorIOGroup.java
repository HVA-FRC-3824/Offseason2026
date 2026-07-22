// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;

public class MotorIOGroup implements MotorIO {

  private ArrayList<MotorIO> m_motors;

  private MotorIO m_leadMotor;

  private boolean m_isfollowNotCopy = false;

  // Generally always use follow, but copy might be more convenient
  public MotorIOGroup(boolean isfollowNotCopy, MotorIO... motors) {

    m_isfollowNotCopy = isfollowNotCopy;

    m_leadMotor = motors[0];

    for (int i = 1; i < motors.length; i++) {
      m_motors.add(motors[i]);
    }

    if (isfollowNotCopy) initFollow(null);
  }

  public ArrayList<MotorIO> getMotors() {

    return m_motors;
  }

  public void initFollow(boolean[] inverts) {

    if (inverts == null) {
      inverts = new boolean[m_motors.size()];
      for (int i = 0; i < inverts.length; i++) {
        inverts[i] = false;
      }
    }

    for (int i = 0; i < m_motors.size(); i++) {

      m_motors.get(i).follow(m_leadMotor.getId(), inverts[i]);
    }
  }

  @Override
  public void periodic() {

    for (MotorIO motor : m_motors) {
      motor.periodic();
    }
  }

  @Override
  public void config(MotorConfig config) {

    for (MotorIO motor : m_motors) {
      motor.config(config);
    }
  }

  @Override
  public int getId() {

    return m_leadMotor.getId();
  }

  @Override
  public void follow(int id, boolean inverted) {

    m_leadMotor.follow(id, inverted);

    if (m_isfollowNotCopy) {
      for (MotorIO motor : m_motors) {
        motor.follow(id, inverted);
      }
    }
  }

  @Override
  public void brake() {

    m_leadMotor.brake();

    if (m_isfollowNotCopy) {
      for (MotorIO motor : m_motors) {
        motor.brake();
      }
    }
  }

  @Override
  public void setPosition(Angle angle) {

    m_leadMotor.setPosition(angle);

    if (m_isfollowNotCopy) {
      for (MotorIO motor : m_motors) {
        motor.setPosition(angle);
      }
    }
  }

  @Override
  public void setVelocity(AngularVelocity angleVel) {

    m_leadMotor.setVelocity(angleVel);

    if (m_isfollowNotCopy) {
      for (MotorIO motor : m_motors) {
        motor.setVelocity(angleVel);
      }
    }
  }

  @Override
  public void resetEncoder(Angle angle) {

    m_leadMotor.resetEncoder(angle);

    if (m_isfollowNotCopy) {
      for (MotorIO motor : m_motors) {
        motor.resetEncoder(angle);
      }
    }
  }

  @Override
  public Voltage getAppliedVoltage() {

    return m_leadMotor.getAppliedVoltage();
  }

  @Override
  public Voltage getSupplyVoltage() {

    return m_leadMotor.getSupplyVoltage();
  }

  @Override
  public Angle getPos() {

    return m_leadMotor.getPos();
  }

  @Override
  public AngularVelocity getVelocity() {

    return m_leadMotor.getVelocity();
  }

  @Override
  public double getLastReference() {

    return m_leadMotor.getLastReference();
  }
}
