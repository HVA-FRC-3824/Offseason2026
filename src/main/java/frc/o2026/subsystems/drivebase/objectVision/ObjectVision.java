// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.objectVision;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.Comparator;
import java.util.List;

public class ObjectVision {

  private static Distance FurthestDistanceToTrack = Feet.of(3.0);

  private ObjectCameraIO m_io;

  public ObjectVision(ObjectCameraIO io) {

    m_io = io;
  }

  public Rotation2d directionToObject(int objectId, Pose2d robot) {

    return getClosestObject(objectId, robot).minus(robot.getTranslation()).getAngle();
  }

  public Translation2d getClosestObject(int objectId, Pose2d robot) {

    // The max val/max val case should never happen because you should be a good little programmer
    // and check hasObjects(...) first

    return objectPoses(objectId, robot).stream()
        .min(Comparator.comparingDouble(object -> object.getDistance(robot.getTranslation())))
        .orElse(new Translation2d(Double.MAX_VALUE, Double.MAX_VALUE));
  }

  public List<Translation2d> objectPoses(int objectId, Pose2d robot) {

    return m_io.getObjects().stream()
        .filter(object -> object.objectId() == objectId)
        .filter(object -> FurthestDistanceToTrack.gte(Meters.of(object.translation.getNorm())))
        .map(
            object ->
                robot
                    .getTranslation()
                    .plus(object.translation.toTranslation2d().rotateBy(robot.getRotation())))
        .toList();
  }

  public boolean hasObjects(int objectId) {

    return m_io.getObjects().stream().anyMatch(object -> object.objectId() == objectId);
  }

  // Transform is the transform from the robot center, using robot relative coordinates
  public static record ObjectTargetData(
      int objectId, double confidence, Translation3d translation) {}
}
