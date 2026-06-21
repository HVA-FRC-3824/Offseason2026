// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alliance;
import frc.lib.rebuilt.BallSim;
import frc.lib.rebuilt.firecontrol.ProjectileSimulator;
import frc.lib.rebuilt.firecontrol.ShotCalculator;
import frc.o2026.Constants;
import frc.o2026.RobotState;

public class Flywheel extends SubsystemBase {

  private AngularVelocity m_lastInput = RotationsPerSecond.of(0.0);

  private FlywheelIO m_io;

  private ShotCalculator m_shotCalc;

  public Flywheel(FlywheelIO io) {

    m_io = io;

    ProjectileSimulator.SimParameters params =
        new ProjectileSimulator.SimParameters(
            0.215, // ball mass kg
            0.1501, // ball diameter m
            0.47, // drag coeff (smooth sphere)
            0.2, // Magnus coeff
            1.225, // air density
            0.43, // exit height (m), floor to where the ball leaves the shooter
            0.1016, // flywheel diameter (m), measure with calipers
            1.83, // target height (m), from game manual
            0.5, // slip factor (0=no grip, 1=perfect), tune this on the real robot
            45.0, // launch angle from horizontal, measure from CAD
            0.001, // sim timestep
            1500,
            6000,
            25,
            5.0 // RPM search range, iterations, max sim time
            );

    ProjectileSimulator sim = new ProjectileSimulator(params);
    ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();

    // print it out
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        System.out.printf(
            "%.2fm -> %.0f RPM, %.3fs TOF%n", entry.distanceM(), entry.rpm(), entry.tof());
      }
    }

    // in RobotContainer or wherever you set stuff up
    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = 0.23; // how far forward the launcher is from robot center (m)
    config.launcherOffsetY = 0.0; // how far left, 0 if centered
    config.phaseDelayMs = 30.0; // your vision pipeline latency
    config.mechLatencyMs = 20.0; // how long the mechanism takes to respond
    config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
    config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
    config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub

    m_shotCalc = new ShotCalculator(config);

    // load the LUT you generated
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        m_shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
      }
    }
  }

  public Command off() {

    return runOnce(
        () -> {
          m_io.stopFlywheel();
          m_lastInput = RotationsPerSecond.of(0.0);
        });
  }

  public Command manual(AngularVelocity velocity) {

    return runOnce(
        () -> {
          m_io.setFlywheel(velocity);
          m_lastInput = velocity;
        });
  }

  public Command set(Setpoints setpoint) {

    return runOnce(
        () -> {
          m_io.setFlywheel(setpoint.m_velocity);
          m_lastInput = setpoint.m_velocity;
        });
  }

  public Command auto() {

    // Blue only
    // Translation2d hubCenter  = new Translation2d(4.6, 4.0);  // your target
    // Translation2d hubForward = new Translation2d(1, 0);       // which way the hub faces

    var hubCenter =
        Alliance.isRed()
            ? Constants.Field.RedHub.getTranslation().toTranslation2d()
            : Constants.Field.BlueHub.getTranslation().toTranslation2d();

    var hubForward = Alliance.isRed() ? new Translation2d(-1, 0) : new Translation2d(1, 0);

    return run(
        () -> {
          var pose = RobotState.getInstance().getPoseEst();
          ShotCalculator.ShotInputs inputs =
              new ShotCalculator.ShotInputs(
                  pose.toPose2d(),
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      RobotState.getInstance().getLastMeasuredSpeeds(),
                      pose.toPose2d().getRotation()),
                  RobotState.getInstance().getLastMeasuredSpeeds(),
                  hubCenter,
                  hubForward,
                  0.9, // vision confidence, 0 to 1
                  pose.getRotation().getMeasureY().in(Degrees),
                  pose.getRotation().getMeasureX().in(Degrees));

          ShotCalculator.LaunchParameters shot = m_shotCalc.calculate(inputs);
          if (shot.isValid() && shot.confidence() > 50) {
            m_io.setFlywheel(RPM.of(shot.rpm()));
            RobotState.getInstance().setSOTMRotTarget(shot.driveAngle());

            if (RobotBase.isSimulation() && RobotState.getInstance().isSimIndexing()) {
              BallSim.getInstance()
                  .shoot(
                      pose.getTranslation(),
                      new Translation3d(
                              ProjectileSimulator.rpmToExitVelocity(
                                  shot.rpm(), Units.inchesToMeters(5.0), 0.5),
                              0,
                              0)
                          .rotateBy(
                              new Rotation3d(
                                  Degrees.of(0.0),
                                  Degrees.of(27.0),
                                  pose.getRotation().getMeasureZ())),
                      new Translation3d(0.0, shot.driveAngularVelocityRadPerSec(), 0.0));
            }
          }
        });
  }

  public boolean isSpunUp() {

    return getReference().isNear(m_io.getMeasured(), Constants.Flywheel.SpunUpTolerance)
        && getReference().in(RotationsPerSecond) != 0.0;
  }

  public AngularVelocity getReference() {

    return m_lastInput;
  }

  public enum Setpoints {
    Backwards(Constants.Flywheel.CloseSpeed.times(-1.0)),
    Low(Constants.Flywheel.CloseSpeed),
    Mid(Constants.Flywheel.MiddleSpeed),
    Neutral(Constants.Flywheel.NeutralPassSpeed),
    Field(Constants.Flywheel.FieldPassSpeed);

    AngularVelocity m_velocity;

    private Setpoints(AngularVelocity vel) {
      m_velocity = vel;
    }
  }
}
