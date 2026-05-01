// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.util.ShiftHelpers;
import frc.robot.util.Zones;
import frc.robot.subsystems.Vision.VisionConstants;

import static edu.wpi.first.units.Units.Meters;

/** Default drive command that handles normal driving plus trench/bump auto-alignment */
public class teleopDrive extends Command {
    private final CommandSwerveDrivetrain m_swerveSubsystem;
    private final CommandXboxController m_driverController;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.kMaxSpeed * DriveConstants.kTranslationDeadband)
            .withRotationalDeadband(DriveConstants.kMaxAngularRate * DriveConstants.kRotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Trigger inTrenchZoneTrigger;
    private final Trigger inBumpZoneTrigger;

    private final PIDController bumpYController = new PIDController(
            ZoneConstants.getBumpYkP(),
            ZoneConstants.getBumpYkI(),
            ZoneConstants.getBumpYkD());
    private final PIDController rotationController = new PIDController(
            ZoneConstants.getRotationkP(),
            ZoneConstants.getRotationkI(),
            ZoneConstants.getRotationkD());
    private DriveMode currentDriveMode = DriveMode.NORMAL;

    public teleopDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController) {
        this.m_driverController = driverController;
        this.m_swerveSubsystem = drivetrain;

        SmartDashboard.putBoolean("Overrides/Bump Trench Assist Enabled", true);

        bumpYController.setTolerance(ZoneConstants.BUMP_Y_TOLERANCE);
        rotationController.setTolerance(ZoneConstants.ROTATION_TOLERANCE);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        inTrenchZoneTrigger = Zones.TRENCH_ZONES
                .willContain(
                        () -> drivetrain.getState().Pose,
                        drivetrain::getAsFieldRelativeSpeeds,
                        Seconds.of(ZoneConstants.TRENCH_ALIGN_TIME_SECONDS))
                .debounce(0.1);

        inBumpZoneTrigger = Zones.BUMP_ZONES
                .willContain(
                        () -> drivetrain.getState().Pose,
                        drivetrain::getAsFieldRelativeSpeeds,
                        Seconds.of(ZoneConstants.BUMP_ALIGN_TIME_SECONDS))
                .debounce(0.1);

      //inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_SLOWDOWN));
        //inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
        m_driverController.rightTrigger().onTrue(updateDriveMode(DriveMode.SHOOTING));
        m_driverController.y().onTrue(updateDriveMode(DriveMode.BUMP_TIGHT_SWEEP));
        m_driverController.rightTrigger().or(m_driverController.y()).onFalse(updateDriveMode(DriveMode.NORMAL));
        

        addRequirements(drivetrain);
    }


    private double getBumpY() {
        Pose2d robotPose = m_swerveSubsystem.getState().Pose;
        double fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();
        double bumpCenterY = ZoneConstants.BUMP_CENTER_Y.in(Meters);
        if (robotPose.getY() >= fieldWidth / 2.0) {
            return fieldWidth - bumpCenterY;
        }
        return bumpCenterY;
    }

    private Rotation2d getBumpLockAngle() {
        double currentDeg = m_swerveSubsystem.getState().Pose.getRotation().getDegrees();
        for (int i = -135; i < 180; i += 90) {
            if (Math.abs(MathUtil.inputModulus(currentDeg - i, -180, 180)) <= 45) {
                return Rotation2d.fromDegrees(i);
            }
        }
        return Rotation2d.kZero;
    }

    private Rotation2d getBumpLockTightSweepAngle() {
        double currentDeg = m_swerveSubsystem.getState().Pose.getRotation().getDegrees();
        for (int i = -90; i <= 180; i += 90) {
            if (Math.abs(MathUtil.inputModulus(currentDeg - i, -180, 180)) <= 45) {
                return Rotation2d.fromDegrees(i);
            }
        }
        return Rotation2d.kZero;
    }

    private boolean isInNeutralZone() {
        double xPose = m_swerveSubsystem.getState().Pose.getX();
        if (ShiftHelpers.isBlue()) {
            return xPose > PoseConstants.kBlueAllianceZoneLineX;
        } else {
            return xPose < PoseConstants.kRedAllianceZoneLineX;
        }
    }

    private Command updateDriveMode(DriveMode driveMode) {
        return Commands.runOnce(() -> currentDriveMode = driveMode);
    }

    public void setBumpTrenchAssistEnabled(boolean enabled) {
        SmartDashboard.putBoolean("Overrides/Bump Trench Assist Enabled", enabled);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xInput = -m_driverController.getLeftY();
        double yInput = -m_driverController.getLeftX();
        double omegaInput = -m_driverController.getRightX();

        SmartDashboard.putData("Tuning/Bump Y Controller", bumpYController);
        SmartDashboard.putData("Tuning/Rotation Controller", rotationController);

        boolean bumpTrenchEnabled = SmartDashboard.getBoolean("Overrides/Bump Trench Assist Enabled", true);

        // If bump/trench assist is disabled, force trench/bump modes back to normal
        if (!bumpTrenchEnabled && (currentDriveMode == DriveMode.TRENCH_SLOWDOWN || currentDriveMode == DriveMode.BUMP_LOCK)) {
            currentDriveMode = DriveMode.NORMAL;
        }

        switch (currentDriveMode) {
            case NORMAL:
                m_swerveSubsystem.setControl(
                        driveRequest
                                .withVelocityX(xInput * DriveConstants.kMaxSpeed)
                                .withVelocityY(yInput * DriveConstants.kMaxSpeed)
                                .withRotationalRate(omegaInput * DriveConstants.kMaxAngularRate));
                break;

            case TRENCH_SLOWDOWN:
                m_swerveSubsystem.setControl(
                        driveRequest
                                .withVelocityX(xInput * DriveConstants.kMaxSpeed * ZoneConstants.TRENCH_SPEED_FACTOR)
                                .withVelocityY(yInput * DriveConstants.kMaxSpeed * ZoneConstants.TRENCH_SPEED_FACTOR)
                                .withRotationalRate(omegaInput * DriveConstants.kMaxAngularRate * ZoneConstants.TRENCH_SPEED_FACTOR));
                break;

            case BUMP_LOCK:
                // Lock rotation to nearest 90-degree angle
                rotationController.setSetpoint(getBumpLockAngle().getRadians());
                rotationController.setP(ZoneConstants.getRotationkP());
                rotationController.setI(ZoneConstants.getRotationkI());
                rotationController.setD(ZoneConstants.getRotationkD());

                double rotCorrection = rotationController.calculate(m_swerveSubsystem.getState().Pose.getRotation().getRadians());
                if (rotationController.atSetpoint()) {
                    rotCorrection = 0;
                }

                // // Lock Y to bump center
                // bumpYController.setSetpoint(getBumpY()); //in meters
                // bumpYController.setP(ZoneConstants.getBumpYkP());
                // bumpYController.setI(ZoneConstants.getBumpYkI());
                // bumpYController.setD(ZoneConstants.getBumpYkD());

                // double yCorrection = bumpYController.calculate(m_swerveSubsystem.getState().Pose.getY());
                // if (bumpYController.atSetpoint()) {
                //     yCorrection = 0;
                // }

                // // Flip Y correction for red alliance since field-centric Y is inverted
                // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                //     yCorrection = -yCorrection;
                // }

                m_swerveSubsystem.setControl(
                        driveRequest
                                .withVelocityX(xInput * DriveConstants.kMaxSpeed * ZoneConstants.BUMP_SPEED_FACTOR)
                                .withVelocityY(yInput * DriveConstants.kMaxSpeed * ZoneConstants.BUMP_SPEED_FACTOR)
                                .withRotationalRate(rotCorrection));
                break;

            case BUMP_TIGHT_SWEEP:
                ZoneConstants.BUMP_TIGHT_SWEEP_FACTOR = ZoneConstants.kBumpTightSweepSlowdownSpeed.get();
                // Lock rotation to nearest 90-degree angle
                rotationController.setSetpoint(getBumpLockTightSweepAngle().getRadians());
                rotationController.setP(ZoneConstants.getRotationkP());
                rotationController.setI(ZoneConstants.getRotationkI());
                rotationController.setD(ZoneConstants.getRotationkD());

                double rotTightSweepCorrection = rotationController.calculate(m_swerveSubsystem.getState().Pose.getRotation().getRadians());
                if (rotationController.atSetpoint()) {
                    rotTightSweepCorrection = 0;
                }

                m_swerveSubsystem.setControl(
                        driveRequest
                                .withVelocityX(xInput * DriveConstants.kMaxSpeed * ZoneConstants.BUMP_TIGHT_SWEEP_FACTOR)
                                .withVelocityY(yInput * DriveConstants.kMaxSpeed * ZoneConstants.BUMP_TIGHT_SWEEP_FACTOR)
                                .withRotationalRate(rotTightSweepCorrection));
                break;

            case SHOOTING:
                // if (isInNeutralZone()) {
                //     m_swerveSubsystem.setControl(
                //             driveRequest
                //                     .withVelocityX(xInput * DriveConstants.kMaxSpeed)
                //                     .withVelocityY(yInput * DriveConstants.kMaxSpeed)
                //                     .withRotationalRate(omegaInput * DriveConstants.kMaxAngularRate));
                // } else {
                    ZoneConstants.SHOOTING_SPEED_FACTOR = ZoneConstants.kShooterSlowdownTuningSpeed.get();
                    m_swerveSubsystem.setControl(
                            driveRequest
                                    .withVelocityX(xInput * DriveConstants.kMaxSpeed * ZoneConstants.SHOOTING_SPEED_FACTOR)
                                    .withVelocityY(yInput * DriveConstants.kMaxSpeed * ZoneConstants.SHOOTING_SPEED_FACTOR)
                                    .withRotationalRate(omegaInput * DriveConstants.kMaxAngularRate * ZoneConstants.SHOOTING_SPEED_FACTOR));
                // }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    private enum DriveMode {
        NORMAL,
        TRENCH_SLOWDOWN,
        BUMP_LOCK,
        BUMP_TIGHT_SWEEP,
        SHOOTING
    }
}
