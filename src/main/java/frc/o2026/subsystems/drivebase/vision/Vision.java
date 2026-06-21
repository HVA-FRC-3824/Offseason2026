// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.o2026.Constants;
import frc.o2026.Robot;
import java.util.function.Consumer;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {

  public static AprilTagFieldLayout fieldAprilTags =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private VisionSystemSim visionSim;

  private Consumer<VisionData> m_poseEstimatorConsumer;

  private int m_lastSeenTag = 0;

  public Camera[] cameras =
      new Camera[] {new Camera(Constants.Vision.kCameraName1, Constants.Vision.RobotToCam1)};

  public Vision(Consumer<VisionData> poseEstimatorConsumer) {

    m_poseEstimatorConsumer = poseEstimatorConsumer;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(fieldAprilTags);

      for (Camera m_camera : cameras) {
        visionSim.addCamera(m_camera.getSimCamera(), m_camera.getOffset());
      }
    }
  }

  public void periodic(Pose2d simRealPos) {

    for (Camera camera : cameras)
      for (VisionData data : camera.getMeasurements()) m_poseEstimatorConsumer.accept(data);

    // This just checks if the there is a measurement,
    // then if there are any tags in that measurement
    // then it takes the first tag of that measurement
    // and makes that the last seen tag
    if (cameras[0].getMeasurements().size() > 0) {
      if (cameras[0].getMeasurements().get(cameras[0].getMeasurements().size() - 1).target().length
          > 0) {

        m_lastSeenTag =
            cameras[0].getMeasurements().get(cameras[0].getMeasurements().size() - 1).target()[0];
      }
    }

    if (simRealPos != null) visionSim.update(simRealPos);
  }

  public int getLastSeenTag() {

    return m_lastSeenTag;
  }

  public static Pose3d getTagPose(int fiduciary) {

    return Constants.Vision.kTagLayout.getTagPose(fiduciary).orElse(new Pose3d());
  }

  public record VisionData(
      Pose3d visionMeasurement, double timestampSeconds, Matrix<N4, N1> stdDevs, int[] target) {}
}
