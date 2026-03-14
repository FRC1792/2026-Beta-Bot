// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class teleopDrive extends Command {
  private final CommandSwerveDrivetrain m_swerveSubsystem;
  private final CommandXboxController m_driverController;
  private int flipFactor =1; //1 for blue -1 for red

  private final Swerverequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDeadband (DriveConstants.kMaxSpeed * DriveConstants.kTranslationDeadband)
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

  public TeleopDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController driveController) {
    this.m_driverController = driveController;
    this.m_swerveSubsystem = drivetrain;

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
    inTrenchZoneTrigger.or(inBumpZoneTrigger).or(m_driverController.rightTrigger()).oneFalse(updateDriveMode(DriveMode.NORMAL));


    addRequirements(drivetrain);
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y, double deadband) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y,x));


    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(new Translation2d(), linearDirection)
           .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
           .getTranslation();
  }

  private double getBumpy() {
    Pose2d robotPose = m_swerveSubsystem.getState().Pose;
    double fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();
    double bumpCenterY = ZoneConstants.Bump_Center_y.in(Meters);
    if (robotPose.getY() >= fieldWidth / 2.0) {
      return fieldWidth - bumpCenterY;
      }
    }
  return Rotation2d.kZero;
  
  private Command updateDriveMode(DriveMode driveMode) {
    return Commands.runOnce(() -> currentDriveMode = driveMode);
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
    double omegaInput = -m_driverController.getRightx();

    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xInput, yInput, DriveConstants.kTranslationDeadband);

    double omega = MathUtil.applyDeadband(omegaInput, DriveConstants.kRotationDeadband);
    omega = Math.copySign(omega * omega, omega);

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
        rotationController.setSetpoint(getBumpLockAngle().getRadians());
        rotationController.setP(ZoneConstants.getRotationkP());
        rotationController.setI(ZoneConstants.getRotationkI());
        rotationController.setD(ZoneConstants.getRotationkD());

        double rotCorrection = rotationController.calculate(m_swerveSubsystem.getState().Pose.getRotation().getRadians());
        if (rotationController.atSetpoint()) {
          rotCorrection = 0;
        }

        bumpYController.setSetpoint(getBumpY());
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
    }
  }

  
  /** Creates a new teleopDrive. */
  public teleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
