// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionIO.PoseObservationType;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final String[] cameraEnabledKeys;
  private final String[] cameraLogPaths;
  private final String[] cameraEnabledLogPaths;
  private final String[] cameraTagPosePaths;
  private final String[] cameraRobotPosePaths;
  private final String[] cameraRobotPoseAcceptedPaths;
  private final String[] cameraRobotPoseRejectedPaths;

  // Pre-allocated lists to avoid GC pressure (reused each cycle)
  private final List<Pose3d> allTagPoses = new LinkedList<>();
  private final List<Pose3d> allRobotPoses = new LinkedList<>();
  private final List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
  private final List<Pose3d> allRobotPosesRejected = new LinkedList<>();
  private final List<CameraObservation> allValidObservations = new ArrayList<>();
  private final List<Pose3d> tagPoses = new LinkedList<>();
  private final List<Pose3d> robotPoses = new LinkedList<>();
  private final List<Pose3d> robotPosesAccepted = new LinkedList<>();
  private final List<Pose3d> robotPosesRejected = new LinkedList<>();

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Initialize SmartDashboard toggles and cached log paths for each camera
    this.cameraEnabledKeys = new String[io.length];
    this.cameraLogPaths = new String[io.length];
    this.cameraEnabledLogPaths = new String[io.length];
    this.cameraTagPosePaths = new String[io.length];
    this.cameraRobotPosePaths = new String[io.length];
    this.cameraRobotPoseAcceptedPaths = new String[io.length];
    this.cameraRobotPoseRejectedPaths = new String[io.length];

    for (int i = 0; i < io.length; i++) {
      cameraEnabledKeys[i] = "Overrides/Vision Camera " + i + " Enabled";
      SmartDashboard.putBoolean(cameraEnabledKeys[i], true);  // Force true on startup

      // Cache log paths to avoid string concatenation every cycle
      String cameraBase = "Subsystems/Vision/Camera" + Integer.toString(i);
      cameraLogPaths[i] = cameraBase;
      cameraEnabledLogPaths[i] = cameraBase + "/Enabled";
      cameraTagPosePaths[i] = cameraBase + "/TagPoses";
      cameraRobotPosePaths[i] = cameraBase + "/RobotPoses";
      cameraRobotPoseAcceptedPaths[i] = cameraBase + "/RobotPosesAccepted";
      cameraRobotPoseRejectedPaths[i] = cameraBase + "/RobotPosesRejected";
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(cameraLogPaths[i], inputs[i]);
    }

    // Clear pre-allocated lists for reuse (avoids GC pressure)
    allTagPoses.clear();
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();
    allValidObservations.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Check if camera is enabled via dashboard
      boolean cameraEnabled = SmartDashboard.getBoolean(cameraEnabledKeys[cameraIndex], true);
      Logger.recordOutput(cameraEnabledLogPaths[cameraIndex], cameraEnabled);

      // Clear per-camera lists for reuse
      tagPoses.clear();
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            !cameraEnabled // Camera disabled via dashboard
                || observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        if (observation.type() == PoseObservationType.MEGATAG_1) {
          linearStdDev *= Double.MAX_VALUE;
        }

        // Store observation for later processing
        allValidObservations.add(
            new CameraObservation(
                cameraIndex, observation, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(cameraTagPosePaths[cameraIndex], tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraRobotPosePaths[cameraIndex], robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraRobotPoseAcceptedPaths[cameraIndex], robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(cameraRobotPoseRejectedPaths[cameraIndex], robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Find the camera with the closest observation this cycle
    int selectedCameraIndex = -1;
    double closestDistance = Double.MAX_VALUE;
    for (var cameraObs : allValidObservations) {
      if (cameraObs.observation.averageTagDistance() < closestDistance) {
        closestDistance = cameraObs.observation.averageTagDistance();
        selectedCameraIndex = cameraObs.cameraIndex;
      }
    }

    // Send only observations from the selected camera to the consumer (max 10 per cycle)
    int observationCount = 0;
    if (selectedCameraIndex >= 0) {
      for (var cameraObs : allValidObservations) {
        if (observationCount >= 10) break;
        if (cameraObs.cameraIndex == selectedCameraIndex) {
          consumer.accept(
              cameraObs.observation.pose().toPose2d(),
              cameraObs.observation.timestamp(),
              VecBuilder.fill(
                  cameraObs.linearStdDev, cameraObs.linearStdDev, cameraObs.angularStdDev));
          observationCount++;
        }
      }
    }

    // Log which camera was selected
    String selectedCameraName;
    switch (selectedCameraIndex) {
      case 0:
        selectedCameraName = "LEFT";
        break;
      case 1:
        selectedCameraName = "FRONT";
        break;
      default:
        selectedCameraName = "NONE";
        break;
    }
    Logger.recordOutput("Subsystems/Vision/SelectedCamera", selectedCameraName);
    Logger.recordOutput("Subsystems/Vision/SelectedCameraDistance", closestDistance);
    Logger.recordOutput("Subsystems/Vision/SelectedCameraObservationCount", observationCount);

    // Log summary data
    Logger.recordOutput("Subsystems/Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Subsystems/Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Subsystems/Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Subsystems/Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** Helper class to store observation data from a camera. */
  private static record CameraObservation(
      int cameraIndex,
      VisionIO.PoseObservation observation,
      double linearStdDev,
      double angularStdDev) {}
}
