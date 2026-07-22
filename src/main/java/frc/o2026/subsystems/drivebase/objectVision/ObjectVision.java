// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.objectVision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ObjectVision {

  private ObjectCameraIO m_io;

  public ObjectVision(ObjectCameraIO io) {

    m_io = io;
  }

  public Optional<Rotation2d> directionToObject(int objectId) {

    var closest = m_io.getRotToBestObject();

    if (closest.isEmpty()) return Optional.empty();

    Logger.recordOutput("objectYaw", closest.get().in(Degrees));

    return Optional.of(new Rotation2d(closest.get()));
  }

  public Optional<Translation2d> getClosestObject(int objectId, Pose2d robot) {

    var poses = objectPoses(objectId, robot);

    if (poses.isEmpty()) return Optional.empty();

    // Compare distance to the robot's position, not the field origin.
    return poses.get().stream()
        .min(Comparator.comparingDouble(object -> object.getDistance(robot.getTranslation())));
  }

  public Optional<List<Translation2d>> objectPoses(int objectId, Pose2d robot) {

    var objectsOpt = m_io.getObjects();

    if (objectsOpt.isEmpty()) {
      // Logger.recordOutput("odObject", new Pose2d[0]);
      return Optional.empty();
    }
    var objects = objectsOpt.get();

    List<Translation2d> fieldTranslations =
        objects.stream()
            // .filter(object -> object.objectId() == objectId)
            // .filter(object ->
            // FurthestDistanceToTrack.gte(Meters.of(object.translation.getNorm())))
            .map(
                object -> {
                  // transformation() is robot-relative (robot-to-target).
                  // Rotate by the robot's field heading to convert to field-relative,
                  // then translate by the robot's field position.
                  Translation2d robotRelative = object.translation().toTranslation2d();

                  return robotRelative
                      .rotateBy(robot.getRotation()) // was incorrectly negated before
                      .plus(robot.getTranslation());
                })
            .toList();

    Pose2d[] poseArray =
        fieldTranslations.stream().map(t -> new Pose2d(t, new Rotation2d())).toArray(Pose2d[]::new);
    Logger.recordOutput("odObject", poseArray);

    return Optional.of(fieldTranslations);
  }

  public boolean hasObjects(int objectId) {

    return objectPoses(objectId, new Pose2d()).isPresent(); // object.objectId() == objectId);
  }

  // Transform is the transform from the robot center, using robot relative coordinates
  public static record ObjectTargetData(
      int objectId, double confidence, Translation3d translation) {}
}
