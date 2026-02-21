// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Turret extends SubsystemBase {
  private TalonFX turretMotor;
  private TalonFXConfiguration turretConfig;

  private MotionMagicVoltage m_motionRequest;
  private VoltageOut m_voltageRequest;

  private double m_robotRelativeAngle;
  private double m_fieldRelativeAngle;

  private CommandSwerveDrivetrain m_swerveSubsystem;

  private TurretState currentState = TurretState.STOP;

  /** Creates a new Turret. */
  public Turret(CommandSwerveDrivetrain swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;
    turretMotor = new TalonFX(TurretConstants.kMotorId);

    turretConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                          .withNeutralMode(NeutralModeValue.Brake))
                        .withSlot0(new Slot0Configs()
                                    .withKP(TurretConstants.kP)
                                    .withKI(TurretConstants.kI)
                                    .withKD(TurretConstants.kD)
                                    .withKA(TurretConstants.kA)
                                    .withKV(TurretConstants.kV)
                                    .withKS(TurretConstants.kS))
                        .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(TurretConstants.kCruiseVelocity)
                                        .withMotionMagicAcceleration(TurretConstants.kAcceleration))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(TurretConstants.kSupplyCurrentLimit))
                        .withFeedback(new FeedbackConfigs()
                                      .withSensorToMechanismRatio(TurretConstants.kSensorToMechanismRatio));
    
    turretMotor.getConfigurator().apply(turretConfig);

    

    m_voltageRequest = new VoltageOut(0);

    m_motionRequest = new MotionMagicVoltage(0).withSlot(0);

    turretMotor.setPosition(0);
  }

  public void setGoal(TurretState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case BLUE_HUB:
        turretTrackPose(PoseConstants.BLUE_HUB);
        break;
      case BLUE_OUTPOST_SHUTTLING:
        turretTrackPose(PoseConstants.BLUE_OUTPOST_SHUTTLING);
        break;
      case BLUE_DEPOT_SHUTTLING:
        turretTrackPose(PoseConstants.BLUE_DEPOT_SHUTTLING);
        break;
      case RED_HUB:
        turretTrackPose(PoseConstants.RED_HUB);
        break;
      case RED_OUTPOST_SHUTTLING:
        turretTrackPose(PoseConstants.RED_OUTPOST_SHUTTLING);
        break;
      case RED_DEPOT_SHUTTLING:
        turretTrackPose(PoseConstants.RED_DEPOT_SHUTTLING);
        break;
      case STOP:
        turretMotor.stopMotor();
        break;
    }
  }

  public void autoGoal() {
    // Get current robot position
    double xPose = m_swerveSubsystem.getState().Pose.getX();
    double yPose = m_swerveSubsystem.getState().Pose.getY();

    // Get alliance color
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      TurretState targetState;

      if (alliance.get() == Alliance.Blue) {
        if (xPose > PoseConstants.kBlueAllianceZoneLineX) {
          // In shuttling zone - choose depot or outpost based on Y position
          targetState = (yPose > PoseConstants.kFieldMidlineY)
            ? TurretState.BLUE_DEPOT_SHUTTLING
            : TurretState.BLUE_OUTPOST_SHUTTLING;
        } else {
          targetState = TurretState.BLUE_HUB;
        }
      } else {
        if (xPose < PoseConstants.kRedAllianceZoneLineX) {
          // In shuttling zone - choose depot or outpost based on Y position
          targetState = (yPose > PoseConstants.kFieldMidlineY)
            ? TurretState.RED_OUTPOST_SHUTTLING
            : TurretState.RED_DEPOT_SHUTTLING;
        } else {
          targetState = TurretState.RED_HUB;
        }
      }
      setGoal(targetState);
    }
  }

  public void turretTurnLeft() {
    turretMotor.set(TurretConstants.kSpeed);
  }

  public void turretTurnRight() {
    turretMotor.set(-TurretConstants.kSpeed);
  }

  public void turretStop() {
    turretMotor.stopMotor();
  }

  public void setPivotPosition(double position) {
    double moddedPosition = MathUtil.inputModulus(position, TurretConstants.kMinAngle, TurretConstants.kMaxAngle);
    turretMotor.setControl(m_motionRequest.withPosition(moddedPosition/360));
  }

  public void turretTrackPose(Pose2d target) {
    // Get robot pose with turret offset
    Pose2d robotPose = m_swerveSubsystem.getState().Pose;
    // Apply turret offset to robot pose
    Transform2d turretOffset = 
      new Transform2d(
        Units.inchesToMeters(TurretConstants.kTurretTransformInchesX),
        Units.inchesToMeters(TurretConstants.kTurretTransformInchesY),
        new Rotation2d());
    Pose2d turretPose = robotPose.plus(turretOffset);
    
    // Calculate vector to target
    double dY = target.getY() - turretPose.getY();
    double dX = target.getX() - turretPose.getX();
    
    // Calculate field-relative angle to target
    Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(dY, dX));
    
    // Convert to robot-relative angle
    Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

    // Update angle variables
    m_robotRelativeAngle = robotRelativeAngle.getDegrees();
    m_fieldRelativeAngle = fieldRelativeAngle.getDegrees();
    
    // Command turret
    this.setPivotPosition(robotRelativeAngle.getDegrees());
  }

  private final SysIdRoutine m_sysIdRoutine = 
    new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        Seconds.of(10),
        (state) -> SignalLogger.writeString("Turret State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> turretMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
        null,
        this
      )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void zeroTuret() {
    turretMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    autoGoal();
    logMotorData();
  }

  public boolean isAtSetpoint() {
    return Math.abs((turretMotor.getPosition().getValueAsDouble() * 360) - (m_motionRequest.Position*360)) <= TurretConstants.kTolerance;
  }

  public void logMotorData() {

    // SmartDashboard.putData("Tuurret motor", turretMotor.getcontrolle);
    Logger.recordOutput("Subsystems/Turret/TurretState", currentState.name());
    
    Logger.recordOutput("Subsystems/Turret/PivotPosition", turretMotor.getPosition().getValueAsDouble() * 360);
    Logger.recordOutput("Subsystems/Turret/PivotSetpoint", m_motionRequest.Position * 360);
    Logger.recordOutput("Subsystems/Turret/IsAtSetpoint", Math.abs((turretMotor.getPosition().getValueAsDouble() * 360) - (m_motionRequest.Position*360)) <= TurretConstants.kTolerance);

    Logger.recordOutput("Subsystems/Turret/Basic/PivotVelocity", turretMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Turret/Basic/PivotSupplyCurrent", turretMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Turret/Basic/PivotStatorCurrent", turretMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Turret/Basic/PivotVoltage", turretMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Turret/Tracking/RobotRelativeAngle", m_robotRelativeAngle);
    Logger.recordOutput("Subsystems/Turret/Tracking/FieldRelativeAngle", m_fieldRelativeAngle);
    Logger.recordOutput("Subsystems/Turret/Tracking/TurretPose", new Pose2d(m_swerveSubsystem.getState().Pose.getTranslation(), Rotation2d.fromDegrees(m_fieldRelativeAngle)));
  }
}
