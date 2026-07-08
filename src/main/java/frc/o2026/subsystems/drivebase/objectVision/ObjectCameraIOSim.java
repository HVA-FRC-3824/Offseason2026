// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.objectVision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.o2026.Constants;
import frc.o2026.subsystems.drivebase.objectVision.ObjectVision.ObjectTargetData;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class ObjectCameraIOSim implements ObjectCameraIO {

  private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75),
      CAMERA_VERTICAL_FOV = Rotation2d.fromDegrees(45);

  private Transform3d m_offset;

  private Supplier<Pose2d> m_poseSupplier;

  public ObjectCameraIOSim(Transform3d offset, Supplier<Pose2d> poseSupplier) {

    m_offset = offset;
    m_poseSupplier = poseSupplier;
  }

  @Override
  public Set<ObjectTargetData> getObjects() {

    Set<ObjectTargetData> objects = Set.of();

    Constants.Field.GamepieceNames.forEach(
        (id, key) -> {
          objects.addAll(
              calculateVisibleGamePieces(new Pose3d(m_poseSupplier.get()).plus(m_offset), id)
                  .stream()
                  .map(
                      data ->
                          new ObjectTargetData(
                              id,
                              1,
                              data.minus(new Translation3d(m_poseSupplier.get().getTranslation()))))
                  .toList());
        });

    return objects;
  }

  private List<Translation3d> calculateVisibleGamePieces(Pose3d cameraPose, int objectId) {

    List<Pose3d> gamePiecesOnField =
        SimulatedArena.getInstance()
            .getGamePiecesPosesByType(Constants.Field.GamepieceNames.get(objectId));

    var pieces =
        gamePiecesOnField.stream()
            .map(Pose3d::getTranslation)
            .map(
                translation ->
                    new Pair<>(translation, calculateCameraAngleToObject(translation, cameraPose)))
            .filter(pair -> isObjectWithinFOV(pair.getSecond()));

    return pieces.map(pair -> pair.getFirst()).toList();
  }

  private Rotation3d calculateCameraAngleToObject(Translation3d objectPosition, Pose3d cameraPose) {
    final Translation3d cameraPosition = cameraPose.getTranslation();

    final Translation3d difference = cameraPosition.minus(objectPosition);
    final Rotation3d differenceAsAngle =
        new Rotation3d(
            0,
            new Rotation2d(
                    Math.atan2(difference.getZ(), Math.hypot(difference.getX(), difference.getY())))
                .getRadians(),
            new Rotation2d(Math.atan2(-difference.getY(), -difference.getX())).getRadians());

    return differenceAsAngle.minus(cameraPose.getRotation());
  }

  private boolean isObjectWithinFOV(Rotation3d objectRotation) {
    return Math.abs(objectRotation.getZ()) <= CAMERA_HORIZONTAL_FOV.getRadians() / 2
        && Math.abs(objectRotation.getY()) <= CAMERA_VERTICAL_FOV.getRadians() / 2;
  }
}
