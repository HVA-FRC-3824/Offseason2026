// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.objectVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.o2026.subsystems.drivebase.objectVision.ObjectVision.ObjectTargetData;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class ObjectCameraIOPhoton implements ObjectCameraIO {

  private PhotonCamera m_camera;

  private Transform3d m_offset;

  public ObjectCameraIOPhoton(String name, Transform3d offset) {

    m_camera = new PhotonCamera(name);

    m_offset = offset;
  }

  @Override
  public Set<ObjectTargetData> getObjects() {

    Logger.recordOutput("odBeingCalled", true);

    Set<ObjectTargetData> dataSet =
        new HashSet<ObjectTargetData>(
            m_camera.getAllUnreadResults().stream().reduce((first, second) -> second).stream()
                .flatMap(result -> result.getTargets().stream())
                .map(
                    target ->
                        new ObjectTargetData(
                            target.objDetectId,
                            target.objDetectConf,
                            target
                                .bestCameraToTarget
                                .getTranslation()
                                .plus(m_offset.getTranslation())))
                .toList());

    ArrayList<Pose2d> poses =
        new ArrayList<>(
            dataSet.stream()
                .map((data) -> new Pose2d(data.translation().toTranslation2d(), new Rotation2d()))
                .toList());

    if (poses.size() >= 1) Logger.recordOutput("odObject", poses.toArray(new Pose2d[1]));

    dataSet.forEach((data) -> Logger.recordOutput("odObject", data.translation()));
    Logger.recordOutput("odDataEmpty", dataSet.isEmpty());

    return dataSet;
  }

  public Transform3d getOffset() {
    return m_offset;
  }
}
