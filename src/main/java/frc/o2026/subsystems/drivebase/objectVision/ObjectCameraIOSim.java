// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.o2026.subsystems.drivebase.objectVision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.hardware.vision.VisionConfig;
import frc.o2026.RobotState;
import frc.o2026.subsystems.drivebase.objectVision.ObjectVision.ObjectTargetData;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class ObjectCameraIOSim implements ObjectCameraIO {

  private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75),
      CAMERA_VERTICAL_FOV = Rotation2d.fromDegrees(45);

  private VisionConfig m_config;

  public ObjectCameraIOSim(VisionConfig config) {

    m_config = config;
  }

  @Override
  public Optional<Set<ObjectTargetData>> getObjects() {

    Set<ObjectTargetData> objects = Set.of();

    objects.addAll(
        calculateVisibleGamePieces(RobotState.getPoseEst().transformBy(m_config.offset())).stream()
            .map(
                data ->
                    new ObjectTargetData(
                        0, 1, data.minus(RobotState.getPoseEst().getTranslation())))
            .toList());

    if (objects.isEmpty()) return Optional.empty();
    return Optional.of(objects);
  }

  private List<Translation3d> calculateVisibleGamePieces(Pose3d cameraPose) {

    List<Pose3d> gamePiecesOnField =
        SimulatedArena.getInstance().gamePiecesOnField().stream()
            // .filter(piece -> piece.getType() == "Fuel")
            .map(GamePieceOnFieldSimulation::getPose3d)
            .toList();

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
