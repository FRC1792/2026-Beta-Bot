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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ZoneConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.util.Zones;
import frc.robot.subsystems.Vision.VisionConstants;

/** Default drive command that handles normal driving plus trench/bump auto-alignment */
public class teleopDrive extends Command {
    private final CommandSwerveDrivetrain m_swerveSubsystem;
    private final CommandXboxController m_driverController;
    private int flipFactor = 1; // 1 for blue, -1 for red

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

        inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_SLOWDOWN));
        inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
        m_driverController.rightTrigger().onTrue(updateDriveMode(DriveMode.SHOOTING));
        inTrenchZoneTrigger.or(inBumpZoneTrigger).or(m_driverController.rightTrigger()).onFalse(updateDriveMode(DriveMode.NORMAL));
        

        addRequirements(drivetrain);
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y, double deadband) {
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
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

    private Command updateDriveMode(DriveMode driveMode) {
        return Commands.runOnce(() -> currentDriveMode = driveMode);
    }

    public void setBumpTrenchAssistEnabled(boolean enabled) {
        SmartDashboard.putBoolean("Overrides/Bump Trench Assist Enabled", enabled);
    }

    @Override
    public void initialize() {
        flipFactor = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? -1
                : 1;
    }

    @Override
    public void execute() {
        double xInput = -m_driverController.getLeftY() * flipFactor;
        double yInput = -m_driverController.getLeftX() * flipFactor;
        double omegaInput = -m_driverController.getRightX();

        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                xInput, yInput, DriveConstants.kTranslationDeadband);

        double omega = MathUtil.applyDeadband(omegaInput, DriveConstants.kRotationDeadband);
        omega = Math.copySign(omega * omega, omega);

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
                                .withVelocityX(linearVelocity.getX() * DriveConstants.kMaxSpeed)
                                .withVelocityY(linearVelocity.getY() * DriveConstants.kMaxSpeed)
                                .withRotationalRate(omega * DriveConstants.kMaxAngularRate));
                break;

            case TRENCH_SLOWDOWN:
                m_swerveSubsystem.setControl(
                        driveRequest
                                .withVelocityX(linearVelocity.getX() * DriveConstants.kMaxSpeed * ZoneConstants.TRENCH_SPEED_FACTOR)
                                .withVelocityY(linearVelocity.getY() * DriveConstants.kMaxSpeed * ZoneConstants.TRENCH_SPEED_FACTOR)
                                .withRotationalRate(omega * DriveConstants.kMaxAngularRate * ZoneConstants.TRENCH_SPEED_FACTOR));
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

                // Lock Y to bump center
                bumpYController.setSetpoint(getBumpY()); //in meters
                bumpYController.setP(ZoneConstants.getBumpYkP());
                bumpYController.setI(ZoneConstants.getBumpYkI());
                bumpYController.setD(ZoneConstants.getBumpYkD());

                double yCorrection = -bumpYController.calculate(m_swerveSubsystem.getState().Pose.getY());
                if (bumpYController.atSetpoint()) {
                    yCorrection = 0;
                }

                m_swerveSubsystem.setControl(
                        driveRequest
                                .withVelocityX(linearVelocity.getX() * DriveConstants.kMaxSpeed * ZoneConstants.BUMP_SPEED_FACTOR)
                                .withVelocityY(yCorrection)
                                .withRotationalRate(rotCorrection));
                break;

            case SHOOTING:
                m_swerveSubsystem.setControl(
                        driveRequest
                                .withVelocityX(linearVelocity.getX() * DriveConstants.kMaxSpeed * ZoneConstants.SHOOTING_SPEED_FACTOR)
                                .withVelocityY(linearVelocity.getY() * DriveConstants.kMaxSpeed * ZoneConstants.SHOOTING_SPEED_FACTOR)
                                .withRotationalRate(omega * DriveConstants.kMaxAngularRate * ZoneConstants.SHOOTING_SPEED_FACTOR));
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
        SHOOTING
    }
}
