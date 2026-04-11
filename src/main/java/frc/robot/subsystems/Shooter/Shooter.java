// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.util.ShotCalculator;
import frc.robot.Constants.PoseConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor1Leader;
  private TalonFXConfiguration shooterConfig;

  private TalonFX shooterMotor2Follower;

  private MotionMagicVelocityVoltage m_motionRequest;

  private ShooterState currentState = ShooterState.STOP;
  private boolean autoGoalEnabled = false;

  private CommandSwerveDrivetrain m_swerveSubsystem;

  private double m_goalDistance;

  public Shooter(CommandSwerveDrivetrain swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    shooterMotor1Leader = new TalonFX(ShooterConstants.kMotor1Id);

    shooterConfig = new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                                    .withInverted(InvertedValue.CounterClockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(new Slot0Configs()
                              .withKP(ShooterConstants.kP)
                              .withKI(ShooterConstants.kI)
                              .withKD(ShooterConstants.kD)
                              .withKS(ShooterConstants.kS)
                              .withKA(ShooterConstants.kA)
                              .withKV(ShooterConstants.kV))
                    .withMotionMagic(new MotionMagicConfigs()
                                    .withMotionMagicAcceleration(ShooterConstants.kAcceleration)
                                    .withMotionMagicJerk(ShooterConstants.kJerk))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit));

    shooterMotor1Leader.getConfigurator().apply(shooterConfig);

    shooterMotor2Follower = new TalonFX(ShooterConstants.kMotor2Id);

    shooterMotor2Follower.getConfigurator().apply(shooterConfig);

    shooterMotor2Follower.setControl(new Follower(shooterMotor1Leader.getDeviceID(), MotorAlignmentValue.Opposed));

    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);

    // Initialize SmartDashboard toggle for shooter enable/disable (always start enabled)
    SmartDashboard.putBoolean("Overrides/Shooter Enabled", true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean dashboardEnabled = SmartDashboard.getBoolean("Overrides/Shooter Enabled", false);

    if (dashboardEnabled) {
      if (autoGoalEnabled) {
        // Dashboard enabled + trigger held - run autoGoal
        autoGoal();
      } else {
        // Dashboard enabled + trigger released - idle
        setGoal(ShooterState.IDLE);
      }
    } else {
      if (autoGoalEnabled) {
        // Dashboard disabled + trigger held - manual tower shot
        setGoal(ShooterState.MANUAL_TOWER);
      } else {
        // Dashboard disabled + trigger released - motors off
        setGoal(ShooterState.STOP);
      }
    }

    logMotorData();
  }

  public void setGoal(ShooterState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case BLUE_HUB:
        m_goalDistance = ShotCalculator.getInstance().getCompensatedDistance(PoseConstants.BLUE_HUB);
        setShooterVelocity(ShooterConstants.getShooterHubVelocity(m_goalDistance));
        break;
      case BLUE_DEPOT_SHUTTLING:
        m_goalDistance = ShotCalculator.getInstance().getCompensatedDistance(PoseConstants.BLUE_DEPOT_SHUTTLING);
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_goalDistance));
        break;
      case BLUE_OUTPOST_SHUTTLING:
        m_goalDistance = ShotCalculator.getInstance().getCompensatedDistance(PoseConstants.BLUE_OUTPOST_SHUTTLING);
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_goalDistance));
        break;
      case RED_HUB:
        m_goalDistance = ShotCalculator.getInstance().getCompensatedDistance(PoseConstants.RED_HUB);
        setShooterVelocity(ShooterConstants.getShooterHubVelocity(m_goalDistance));
        break;
      case RED_DEPOT_SHUTTLING:
        m_goalDistance = ShotCalculator.getInstance().getCompensatedDistance(PoseConstants.RED_DEPOT_SHUTTLING);
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_goalDistance));
        break;
      case RED_OUTPOST_SHUTTLING:
        m_goalDistance = ShotCalculator.getInstance().getCompensatedDistance(PoseConstants.RED_OUTPOST_SHUTTLING);
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_goalDistance));
        break;
      case IDLE:
        setShooterVelocity(ShooterConstants.kIdleSpeed);
        break;
      case MANUAL_TOWER:
        setShooterVelocity(ShooterConstants.kManualTowerSpeed);
        break;
      case STOP:
        shooterMotor1Leader.stopMotor();
        shooterMotor2Follower.stopMotor();
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
            ? ShooterState.BLUE_DEPOT_SHUTTLING
            : ShooterState.BLUE_OUTPOST_SHUTTLING;
        } else {
          targetState = ShooterState.BLUE_HUB;
        }
      } else {
        if (xPose < PoseConstants.kRedAllianceZoneLineX) {
          // In shuttling zone - choose depot or outpost based on Y position
          targetState = (yPose > PoseConstants.kFieldMidlineY)
            ? ShooterState.RED_OUTPOST_SHUTTLING
            : ShooterState.RED_DEPOT_SHUTTLING;
        } else {
          targetState = ShooterState.RED_HUB;
        }
      }
      setGoal(targetState);
    }
  }

  public void setShooterVelocity(double velocity) {
    shooterMotor1Leader.setControl(m_motionRequest.withVelocity(velocity));
  }

  public void setAutoGoalEnabled(boolean enabled) {
    autoGoalEnabled = enabled;
  }

  public boolean getAutoGoalEnabled() {
    return autoGoalEnabled;
  }

  public boolean isAtSetpoint() {
    return Math.abs(shooterMotor1Leader.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance;
  }
  
  private void logMotorData() {
    Logger.recordOutput("Subsystems/Shooter/ShooterState", currentState.name());

    Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterMotorVelocity", shooterMotor1Leader.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterSetpoint", m_motionRequest.Velocity);
    Logger.recordOutput("Subsystems/Shooter/Velocity/IsAtSetpoint", Math.abs(shooterMotor1Leader.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance);

    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorSupplyCurrent", shooterMotor1Leader.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorStatorCurrent", shooterMotor1Leader.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorVoltage", shooterMotor1Leader.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Shooter/Tracking/GoalDistance", m_goalDistance);
  }
}