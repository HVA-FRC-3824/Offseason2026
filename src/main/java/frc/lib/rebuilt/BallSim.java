package frc.lib.rebuilt;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation3d;
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
    ballSim.configureRobot(Constants.Chassis.TrackWidthMeters.in(Meters), Constants.Chassis.WheelBaseMeters.in(Meters), Inches.of(4.5).in(Meters),
      () -> RobotState.getInstance().getPoseEst().toPose2d(), () -> RobotState.getInstance().getLastMeasuredSpeeds());

    ballSim.addIntakeZone(-0.85 / 2 - 0.2, -0.85 / 2, 
      Constants.Chassis.TrackWidthMeters.in(Meters) / -2, 
      Constants.Chassis.TrackWidthMeters.in(Meters) / 2, 
      RobotState.getInstance()::isSimIntaking, 
      RobotState.getInstance()::incrementFuel);
  }

  public void update() {

    ballSim.tick();
  }

  public void shoot(Translation3d pos, Translation3d vel, Translation3d omega) {

    ballSim.launchBall(pos, vel, omega);
  }
}
