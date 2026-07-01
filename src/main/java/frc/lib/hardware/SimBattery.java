// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import java.util.HashSet;
import java.util.function.Supplier;

public class SimBattery {

  private static HashSet<Supplier<Current>> m_currents = new HashSet<Supplier<Current>>();

  private static Voltage m_supplyVoltage = Volts.of(12.0);

  public static void registerDevice(Supplier<Current> supplier) {

    m_currents.add(supplier);
  }

  public static void calculateSupplyVoltage() {

    m_supplyVoltage =
        Volts.of(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                m_currents.stream().mapToDouble(curr -> curr.get().in(Amps)).toArray()));
  }

  public static Voltage getSupplyVoltage() {

    return m_supplyVoltage;
  }
}
