// Copyright (c) 2026-2027 Jackson Case
// http://github.com/NO-skcaj
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;

// This logic is always so ugly and cluttering
// Abstract to here.
public class Alliance {
  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
        == DriverStation.Alliance.Red;
  }
}
