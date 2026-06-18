// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Constants;
import frc.o2026.Robot;
import frc.o2026.RobotState;
import java.util.ArrayList;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {

  public static AprilTagFieldLayout fieldAprilTags =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private VisionSystemSim visionSim;

  private SwerveDriveSimulation m_swerveSim;

  private Pose3d m_lastSeenTag = new Pose3d();

  public Camera[] cameras =
      new Camera[] {new Camera(Constants.Vision.kCameraName1, Constants.Vision.RobotToCam1)};

  public Vision() {

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(fieldAprilTags);

      for (Camera m_camera : cameras) {
        visionSim.addCamera(m_camera.getSimCamera(), m_camera.getOffset());
      }
    }
  }

  @Override
  public void periodic() {

    for (Camera m_camera : cameras)
      for (VisionData data : m_camera.getMeasurements())
        RobotState.getInstance().addVisionData(data);

    if (Robot.isSimulation()) visionSim.update(RobotState.getInstance().getSimActualPose());
  }

  public static Pose3d getTagPose(int fiduciary) {

    return Constants.Vision.kTagLayout.getTagPose(fiduciary).orElse(new Pose3d());
  }

  public static record Outputs(ArrayList<VisionData> measurements, Pose2d lastSeenTag) {

    public static Outputs zeroed() {

      return new Outputs(new ArrayList<VisionData>(0), new Pose2d());
    }

    public void log() {

      if (measurements.size() > 0)
        Logger.recordOutput(
            "Vision/measurement", measurements.get(measurements.size() - 1).visionMeasurement());
      Logger.recordOutput("Vision/Last Seen Tag", lastSeenTag);
    }
  }

  public record VisionData(
      Pose3d visionMeasurement, double timestampSeconds, Matrix<N4, N1> stdDevs, int[] target) {}
}
