// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.pathing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.FollowPath;
import java.util.concurrent.atomic.AtomicBoolean;

// Uses two event triggers to specify a region where a command will run
public class BLineRegionCommand extends SequentialCommandGroup {

  AtomicBoolean shouldIntake = new AtomicBoolean(false);

  public BLineRegionCommand(String eventTrigger, Command cmd) {

    FollowPath.registerEventTrigger(eventTrigger + "start", () -> shouldIntake.set(true));
    FollowPath.registerEventTrigger(eventTrigger + "end", () -> shouldIntake.set(false));

    addCommands(Commands.waitUntil(shouldIntake::get), cmd.until(() -> !shouldIntake.get()));
  }

  // Call this as a ".finallyDo(cmd::falsifyTrigger)"
  // on your auto command
  public void falsifyTrigger(boolean b) {

    shouldIntake.set(false);
  }
}
