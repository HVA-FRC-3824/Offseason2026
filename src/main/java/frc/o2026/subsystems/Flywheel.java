// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Alliance;
import frc.lib.hardware.motor.MotorIO;
import frc.lib.rebuilt.BallSim;
import frc.lib.rebuilt.firecontrol.ProjectileSimulator;
import frc.lib.rebuilt.firecontrol.ShotCalculator;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private boolean m_validShot = false;

  private MotorIO m_teacherIO;
  private MotorIO m_studentIO;

  private ShotCalculator m_shotCalc;

  private ShotCalculator.LaunchParameters shot;

  public Flywheel(MotorIO teacherIO, MotorIO studentIO) {

    m_teacherIO = teacherIO;
    m_studentIO = studentIO;

    ProjectileSimulator.SimParameters params =
        new ProjectileSimulator.SimParameters(
            0.215, // ball mass kg
            0.1501, // ball diameter m
            0.47, // drag coeff (smooth sphere)
            0.2, // Magnus coeff
            1.225, // air density
            0.43, // exit height (m), floor to where the ball leaves the shooter
            Units.inchesToMeters(5.0), // flywheel diameter (m), measure with calipers
            1.83, // target height (m), from game manual
            0.8, // slip factor (0=no grip, 1=perfect), tune this on the real robot
            90.0 - 27.0, // launch angle from horizontal, measure from CAD
            0.02, // sim timestep
            1500,
            7000,
            40,
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
    config.mechLatencyMs = 20.0; // how long the mechanism takes to respond
    // config.maxTiltDeg = 5.0; // suppress firing when chassis tilts past this (bumps/ramps)
    // config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to
    // disable)
    // config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub

    m_shotCalc = new ShotCalculator(config);

    // load the LUT you generated
    for (var entry : lut.entries()) {
      if (entry.reachable()) {
        m_shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
      }
    }

    m_studentIO.follow(m_teacherIO.getId(), true);
  }

  @Override
  public void periodic() {

    m_teacherIO.periodic();
    m_studentIO.periodic();

    var hubCenter =
        Alliance.isRed()
            ? Constants.Field.RedHub.getTranslation().toTranslation2d()
            : Constants.Field.BlueHub.getTranslation().toTranslation2d();

    var hubForward = Alliance.isRed() ? new Translation2d(-1, 0) : new Translation2d(1, 0);

    var pose = RobotState.getPoseEst();
    ShotCalculator.ShotInputs inputs =
        new ShotCalculator.ShotInputs(
            pose.toPose2d(),
            ChassisSpeeds.fromRobotRelativeSpeeds(
                RobotState.getLastMeasuredSpeeds(), pose.toPose2d().getRotation()),
            RobotState.getLastMeasuredSpeeds(),
            hubCenter,
            hubForward,
            0.9, // vision confidence, 0 to 1
            pose.getRotation().getMeasureY().in(Degrees),
            pose.getRotation().getMeasureX().in(Degrees));

    shot = m_shotCalc.calculate(inputs);

    if (shot.driveAngle() != Rotation2d.kZero) RobotState.setSOTMRotTarget(shot.driveAngle());

    if (RobotBase.isSimulation())
      Logger.runEveryN(
          5,
          () -> {
            if (m_validShot && RobotState.isSimIndexing() && RobotState.getSimFuelCount() > 0) {

              RobotState.decrementFuel();

              BallSim.getInstance().launchAtRPM(RobotState.getSimRealPose().toPose2d(), shot.rpm());
            }
          });

    Logger.recordOutput("flywheel/isReady", isReady());
    Logger.recordOutput("flywheel/m-velocity", m_teacherIO.getVelocity().in(RotationsPerSecond));
    Logger.recordOutput("flywheel/d-velocity", m_teacherIO.getLastReference());
    Logger.recordOutput("flywheel/m-fuelCount", RobotState.getSimFuelCount());
  }

  public Command off() {

    return runOnce(
        () -> {
          m_teacherIO.brake();
        });
  }

  public Command manual(AngularVelocity velocity) {

    return runOnce(
        () -> {
          m_teacherIO.setVelocity(velocity);
        });
  }

  public Command set(Setpoints setpoint) {

    return runOnce(
        () -> {
          m_teacherIO.setVelocity(setpoint.m_velocity);
        });
  }

  public Command auto() {

    // Blue only
    // Translation2d hubCenter  = new Translation2d(4.6, 4.0);  // your target
    // Translation2d hubForward = new Translation2d(1, 0);       // which way the hub faces

    return run(() -> {
          m_validShot = shot.isValid() && shot.confidence() > 50;

          m_teacherIO.setVelocity(RPM.of(shot.rpm()));
        })
        .withName("FlywheelAutoAim");
  }

  public boolean isReady() {

    return Math.abs(
            m_teacherIO.getLastReference() - m_teacherIO.getVelocity().in(RotationsPerSecond))
        <= Configs.Flywheel.SpunUpTolerance;
  }

  public enum Setpoints {
    Backwards(Configs.Flywheel.CloseSpeed.times(-1.0)),
    Low(Configs.Flywheel.CloseSpeed),
    Mid(Configs.Flywheel.MiddleSpeed),
    Neutral(Configs.Flywheel.NeutralPassSpeed),
    Field(Configs.Flywheel.FieldPassSpeed);

    AngularVelocity m_velocity;

    private Setpoints(AngularVelocity vel) {
      m_velocity = vel;
    }
  }
}
