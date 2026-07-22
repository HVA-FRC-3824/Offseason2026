// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.rebuilt.firecontrol.FuelPhysicsSim;
import frc.o2026.Constants;
import frc.o2026.RobotState;

public class BallSim {
    
  private FuelPhysicsSim ballSim;

  private static BallSim m_inst;

  public static BallSim getInstance() {

    if (m_inst == null) m_inst = new BallSim();

    return m_inst;
  }

  private BallSim() {

    // Larp advantagekit
    ballSim = new FuelPhysicsSim("AdvantageKit/RealOutputs/Fuel");

    ballSim.enable();
    ballSim.placeFieldBalls();  // spawns all the game pieces

    // tell it about your robot
    ballSim.configureRobot(
      Constants.Chassis.TrackWidthMeters.in(Meters), 
      Constants.Chassis.WheelBaseMeters.in(Meters), 
      Inches.of(4.5).in(Meters),
      () -> RobotState.getSimRealPose().toPose2d(), () -> RobotState.getLastMeasuredSpeeds());

    ballSim.addIntakeZone(-0.85 / 2 - 0.2, -0.85 / 2, 
      Constants.Chassis.TrackWidthMeters.in(Meters) / -2, 
      Constants.Chassis.TrackWidthMeters.in(Meters) / 2, 
      () -> RobotState.isSimIntaking(), 
      () -> RobotState.incrementFuel());
  }

  public void update() {

    ballSim.tick();
  }

  public void launchAtRPM(Pose2d robotPose, double shooterRPM) {
    
    Translation3d launchPos = new Translation3d(robotPose.getX(), robotPose.getY(), Units.inchesToMeters(20.0));

    double launchAngleRad = Math.toRadians(90-27);
    double exitSpeed = 0.7 * shooterRPM * Math.PI * Units.inchesToMeters(5.0) / 60.0;
    double vHorizontal = exitSpeed * Math.cos(launchAngleRad);
    double vVertical = exitSpeed * Math.sin(launchAngleRad);

    double vx = vHorizontal * Math.cos(robotPose.getRotation().getRadians());
    double vy = vHorizontal * Math.sin(robotPose.getRotation().getRadians());

    Translation3d launchVel = new Translation3d(vx, vy, vVertical);
    ballSim.launchBall(launchPos, launchVel, 2000.0);
  }
}
