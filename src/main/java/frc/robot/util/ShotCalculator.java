package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.TurretConstants;

/**
 * Centralized launch calculator that provides velocity-compensated virtual targets.
 * Drop-in replacement for the old getVirtualTarget() — subsystems still handle their
 * own auto-goal selection and tracking, but pass targets through here for compensation.
 */
public class ShotCalculator {
    private static ShotCalculator instance;

    private CommandSwerveDrivetrain m_swerveSubsystem;

    // Configuration
    private static final double PHASE_DELAY = 0.02; // seconds of processing latency compensation

    private ShotCalculator() {}

    public static ShotCalculator getInstance() {
        if (instance == null) instance = new ShotCalculator();
        return instance;
    }

    public void init(CommandSwerveDrivetrain swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
    }

    /**
     * Takes a raw target pose and returns a velocity-compensated virtual target.
     * Accounts for:
     * - Phase delay (projects robot pose forward to compensate for processing latency)
     * - Robot velocity (iteratively offsets aim to lead the target based on air time)
     * - Turret offset from robot center (rotational velocity cross-term)
     *
     * @param rawTarget The actual target position on the field
     * @return A compensated virtual target Pose2d that the turret should track
     */
    public Pose2d getVirtualTarget(Pose2d rawTarget) {
        if (m_swerveSubsystem == null) {
            return rawTarget;
        }

        Pose2d robotPose = m_swerveSubsystem.getState().Pose;
        ChassisSpeeds robotRelativeVelocity = m_swerveSubsystem.getState().Speeds;

        // Phase delay compensation — estimate where robot will be
        Pose2d estimatedPose = robotPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * PHASE_DELAY,
                robotRelativeVelocity.vyMetersPerSecond * PHASE_DELAY,
                robotRelativeVelocity.omegaRadiansPerSecond * PHASE_DELAY));

        // Field-relative velocity
        ChassisSpeeds fieldVelocity = m_swerveSubsystem.getAsFieldRelativeSpeeds();

        // Calculate field-relative turret velocity including rotational cross-term
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretOffsetX = Units.inchesToMeters(TurretConstants.kTurretTransformInchesX);
        double turretOffsetY = Units.inchesToMeters(TurretConstants.kTurretTransformInchesY);

        double turretVelocityX = fieldVelocity.vxMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond
                * (turretOffsetY * Math.cos(robotAngle) - turretOffsetX * Math.sin(robotAngle));
        double turretVelocityY = fieldVelocity.vyMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond
                * (turretOffsetX * Math.cos(robotAngle) - turretOffsetY * Math.sin(robotAngle));

        Translation2d target = rawTarget.getTranslation();
        Translation2d turretPosition = estimatedPose.getTranslation();

        // Iterative convergence — each iteration updates distance, which updates air time
        double distanceToTarget = target.getDistance(turretPosition);
        Translation2d virtualTarget = target;

        for (int i = 0; i < 20; i++) {
            double airTime = TurretConstants.getAirTime(distanceToTarget);

            // Offset the target to compensate for robot movement during flight
            virtualTarget = target.minus(new Translation2d(
                turretVelocityX * airTime,
                turretVelocityY * airTime));

            // Recalculate distance with the new virtual target
            distanceToTarget = virtualTarget.getDistance(turretPosition);
        }

        Pose2d virtualTargetPose = new Pose2d(virtualTarget, Rotation2d.kZero);

        // Log
        Logger.recordOutput("LaunchCalculator/VirtualTargetPose", virtualTargetPose);
        Logger.recordOutput("LaunchCalculator/CompensatedDistance", distanceToTarget);
        Logger.recordOutput("LaunchCalculator/TurretVelocityX", turretVelocityX);
        Logger.recordOutput("LaunchCalculator/TurretVelocityY", turretVelocityY);

        return virtualTargetPose;
    }

    /**
     * Returns the compensated distance from the turret to a target,
     * accounting for phase delay and velocity. Useful for shooter speed lookups.
     *
     * @param rawTarget The actual target position on the field
     * @return The compensated distance in meters
     */
    public double getCompensatedDistance(Pose2d rawTarget) {
        if (m_swerveSubsystem == null) {
            return 0.0;
        }

        Pose2d virtualTarget = getVirtualTarget(rawTarget);
        Pose2d robotPose = m_swerveSubsystem.getState().Pose;
        return virtualTarget.getTranslation().getDistance(robotPose.getTranslation());
    }
}
