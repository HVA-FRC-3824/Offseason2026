// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware;

import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ConnectionOrchestrator {

  private HashMap<String, Supplier<Boolean>> m_devices;

  public void registerDevice(String name, Supplier<Boolean> isUp) {

    m_devices.put(name, isUp);
  }

  public void logDevices() {

    m_devices.forEach(
        (name, isUp) -> {
          Logger.recordOutput("Connected/" + name, isUp.get());
        });
  }
}
