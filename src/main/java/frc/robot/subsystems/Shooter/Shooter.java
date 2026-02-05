// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.nio.charset.CharacterCodingException;
import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.Constants.PoseConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor;
  private TalonFXConfiguration shooterConfig;

  private MotionMagicVelocityVoltage m_motionRequest;
  private VoltageOut m_voltageRequest;

  private ShooterState currentState = ShooterState.IDLE;
  private boolean autoGoalEnabled = false;

  private CommandSwerveDrivetrain m_swerveSubsystem;

  private double m_blueHubDistance = 0.0;
  private double m_redHubDistance = 0.0;

  private double m_blueDepotShuttlingDistance = 0.0;
  private double m_blueOutpostShuttlingDistance = 0.0;
  
  private double m_redDepotShuttlingDistance = 0.0;
  private double m_redOutpostShuttlingDistance = 0.0;

  public Shooter(CommandSwerveDrivetrain swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    shooterMotor = new TalonFX(ShooterConstants.kMotorId);

    shooterConfig = new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(new Slot0Configs()
                              .withKP(ShooterConstants.kP)
                              .withKI(ShooterConstants.kI)
                              .withKD(ShooterConstants.kD))
                    .withMotionMagic(new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(ShooterConstants.kCruiseVelocity)
                                    .withMotionMagicAcceleration(ShooterConstants.kAcceleration)
                                    .withMotionMagicJerk(ShooterConstants.kJerk))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit));
    shooterMotor.getConfigurator().apply(shooterConfig);

    m_voltageRequest = new VoltageOut(0);

    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withFeedForward(ShooterConstants.kFeedforward);

    ShooterConstants.setupShooterMap();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();
  }

  public void setGoal(ShooterState desiredState) {
    currentState = desiredState;
    Translation2d currentTranslation2d = m_swerveSubsystem.getState().Pose.getTranslation();
    switch (desiredState) {
      case BLUE_HUB:
        m_blueHubDistance = currentTranslation2d.getDistance(PoseConstants.BLUE_HUB.getTranslation());
        setShooterVelocity(ShooterConstants.kShooterMap.get(m_blueHubDistance));
        break;
      case BLUE_DEPOT_SHUTTLING:
        m_blueDepotShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.BLUE_DEPOT_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.kShooterMap.get(m_blueDepotShuttlingDistance));
        break;
      case BLUE_OUTPOST_SHUTTLING:
        m_blueOutpostShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.BLUE_OUTPOST_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.kShooterMap.get(m_blueOutpostShuttlingDistance));
        break;
      case RED_HUB:
        m_redHubDistance = currentTranslation2d.getDistance(PoseConstants.RED_HUB.getTranslation());
        setShooterVelocity(ShooterConstants.kShooterMap.get(m_redOutpostShuttlingDistance));
        break;
      case RED_DEPOT_SHUTTLING:
        m_redDepotShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.RED_DEPOT_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.kShooterMap.get(m_blueDepotShuttlingDistance));
        break;
      case RED_OUTPOST_SHUTTLING:
        m_redOutpostShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.RED_OUTPOST_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.kShooterMap.get(m_redOutpostShuttlingDistance));
        break;
      case IDLE:
        setShooterVelocity(ShooterConstants.kPrepSpeed);
        break;
      case STOP:
        shooterMotor.stopMotor();
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
      ShooterState targetState;

      if (alliance.get() == Alliance.Blue) {
        if (xPose > PoseConstants.kBlueAllianceZoneLineX) {
          // In shuttling zone - choose depot or outpost based on Y position
          targetState = (yPose > PoseConstants.kFieldMidlineY)
            ? ShooterState.BLUE_OUTPOST_SHUTTLING
            : ShooterState.BLUE_DEPOT_SHUTTLING;
        } else {
          targetState = ShooterState.BLUE_HUB;
        }
      }
      if (alliance.get() == Alliance.Red) {
        if (xPose < PoseConstants.kRedAllianceZoneLineX) {
          // In shuttling zone - choose depot or outpost based on Y position
          targetState = (yPose < PoseConstants.kFieldMidlineY)
            ? ShooterState.RED_OUTPOST_SHUTTLING
            : ShooterState.RED_DEPOT_SHUTTLING;
        } else {
          targetState = ShooterState.RED_HUB;
        }
      }
    }
  }

  public void setShooterVelocity(double velocity) {
    shooterMotor.setControl(m_motionRequest.withVelocity(velocity));
  }

  public void shooterOn() {
    shooterMotor.set(ShooterConstants.kMotorSpeed);
  }

  public void shooterReverse() {
    shooterMotor.set(ShooterConstants.kMotorReverseSpeed);
  }

  public void shooterOff() {
    shooterMotor.stopMotor();
  }

  public boolean isAtSetpoint() {
    return Math.abs(shooterMotor.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance;
  }
  
  private void logMotorData() {
    DogLog.log("Subsystems/Shooter/ShooterState", currentState.name());

    DogLog.log("Subsystems/Shooter/Basic/ShooterMotorVelocity", shooterMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Shooter/Position/ShooterSetpoint", m_motionRequest.Velocity);
    DogLog.log("Subsystems/Shooter/Position/IsAtSetpoint", Math.abs(shooterMotor.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance);

    DogLog.log("Subsystems/Shooter/Basic/ShooterMotorSupplyCurrent", shooterMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Shooter/Basic/ShooterMotorStatorCurrent", shooterMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Shooter/Basic/ShooterMotorVoltage", shooterMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Subsystems/Shooter/Tracking/Blue/HubDistance", m_blueHubDistance);
    DogLog.log("Subsystems/Shooter/Tracking/Blue/DepotShuttlingDistance", m_blueDepotShuttlingDistance);
    DogLog.log("Subsystems/Shooter/Tracking/Blue/OutpostShuttlingDistance", m_blueOutpostShuttlingDistance);

    DogLog.log("Subsystems/Shooter/Tracking/Red/HubDistance", m_redHubDistance);
    DogLog.log("Subsystems/Shooter/Tracking/Red/DepotShuttlingDistance", m_redDepotShuttlingDistance);
    DogLog.log("Subsystems/Shooter/Tracking/Red/OutpostShuttlingDistance", m_redOutpostShuttlingDistance);
  }
}