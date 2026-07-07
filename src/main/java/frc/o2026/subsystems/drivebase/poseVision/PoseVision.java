// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.poseVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.Robot;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class PoseVision extends SubsystemBase {

  public static AprilTagFieldLayout fieldAprilTags =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private VisionSystemSim visionSim;

  private Consumer<VisionData> m_poseEstimatorConsumer;

  private int m_lastSeenTag = 0;

  public PoseCameraIO[] m_cameras;

  public PoseVision(Consumer<VisionData> poseEstimatorConsumer, PoseCameraIO... cameras) {

    m_poseEstimatorConsumer = poseEstimatorConsumer;
    m_cameras = cameras;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(fieldAprilTags);

      for (PoseCameraIO m_camera : cameras) {
        visionSim.addCamera(m_camera.getSimCamera(), m_camera.getOffset());
      }
    }
  }

  public void update() {

    update(null);
  }

  public void update(Pose2d simRealPos) {

    for (PoseCameraIO camera : m_cameras) {
      for (VisionData data : camera.getMeasurements()) {
        m_poseEstimatorConsumer.accept(data);
      }
    }

    try {

      m_lastSeenTag = m_cameras[0].getMeasurements().get(0).target()[0];
    } catch (Exception e) {

    }

    Logger.recordOutput("lastSeenTag", PoseCameraIO.getTagPose(m_lastSeenTag));

    if (simRealPos != null) visionSim.update(simRealPos);
  }

  public int getLastSeenTag() {

    return m_lastSeenTag;
  }

  public record VisionData(
      Pose3d visionMeasurement, double timestampSeconds, Matrix<N4, N1> stdDevs, int[] target) {

    public Matrix<N3, N1> get2dStdDevs() {

      return VecBuilder.fill(stdDevs().get(0, 0), stdDevs().get(1, 0), stdDevs().get(3, 0));
    }
  }
}
