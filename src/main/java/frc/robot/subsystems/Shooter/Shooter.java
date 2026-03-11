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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.Constants.PoseConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotorLeader;
  private TalonFXConfiguration shooterConfig;

  private TalonFX shooterMotorFollower;
  private TalonFXConfiguration shooter2Config;

  private Follower shooterFollower;
  

  private MotionMagicVelocityVoltage m_motionRequest;
  private MotionMagicVelocityVoltage m_motionRequest2;

  private ShooterState currentState = ShooterState.STOP;
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
    shooterMotorLeader = new TalonFX(ShooterConstants.kMotorId);

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
    shooterMotorLeader.getConfigurator().apply(shooterConfig);
    shooterMotorFollower = new TalonFX(ShooterConstants.kMotor2Id);
    shooterMotorFollower.getConfigurator().apply(shooterConfig);
    shooterMotorFollower.setControl(new
    Follower(shooterMotorLeader.getDeviceID(),MotorAlignmentValue.Opposed));

    // shooter2Config = new TalonFXConfiguration()
    //                  .withMotorOutput(new MotorOutputConfigs()
    //                                  .withInverted(InvertedValue.Clockwise_Positive)
    //                                  .withNeutralMode(NeutralModeValue.Coast))
    //                  .withSlot0(new Slot0Configs()
    //                            .withKP(ShooterConstants.kP2)
    //                            .withKI(ShooterConstants.kI2)
    //                            .withKD(ShooterConstants.kD2)
    //                            .withKS(ShooterConstants.kS2)
    //                            .withKA(ShooterConstants.kA2)
    //                            .withKV(ShooterConstants.kV2))
    //                  .withMotionMagic(new MotionMagicConfigs()
    //                                  .withMotionMagicAcceleration(ShooterConstants.kAcceleration2)
    //                                  .withMotionMagicJerk(ShooterConstants.kJerk2))
    //                  .withCurrentLimits(new CurrentLimitsConfigs()
    //                                  .withStatorCurrentLimit(ShooterConstants.kSupplyCurrentLimit)); 
    // shooterMotorFollower.getConfigurator().apply(shooter2Config);   
    


    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);
    m_motionRequest2 = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(false);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (autoGoalEnabled) {
    autoGoal();
    }

    logMotorData();
  }

  public void setGoal(ShooterState desiredState) {
    currentState = desiredState;
    Translation2d currentTranslation2d = m_swerveSubsystem.getState().Pose.getTranslation();
    switch (desiredState) {
      case BLUE_HUB:
        m_blueHubDistance = currentTranslation2d.getDistance(PoseConstants.BLUE_HUB.getTranslation());
        setShooterVelocity(ShooterConstants.getShooterHubVelocity(m_blueHubDistance));
        break;
      case BLUE_DEPOT_SHUTTLING:
        m_blueDepotShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.BLUE_DEPOT_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_blueDepotShuttlingDistance));
        break;
      case BLUE_OUTPOST_SHUTTLING:
        m_blueOutpostShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.BLUE_OUTPOST_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_blueOutpostShuttlingDistance));
        break;
      case RED_HUB:
        m_redHubDistance = currentTranslation2d.getDistance(PoseConstants.RED_HUB.getTranslation());
        setShooterVelocity(ShooterConstants.getShooterHubVelocity(m_redHubDistance));
        break;
      case RED_DEPOT_SHUTTLING:
        m_redDepotShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.RED_DEPOT_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_redDepotShuttlingDistance));
        break;
      case RED_OUTPOST_SHUTTLING:
        m_redOutpostShuttlingDistance = currentTranslation2d.getDistance(PoseConstants.RED_OUTPOST_SHUTTLING.getTranslation());
        setShooterVelocity(ShooterConstants.getShooterNeutralVelocity(m_redOutpostShuttlingDistance));
        break;
      case IDLE:
        setShooterVelocity(ShooterConstants.kPrepSpeed);
        break;
      case STOP:
        shooterMotorLeader.stopMotor();
        break;  
    }
  }

  //Useless code adventure, 3/10/26
  //override
  // public void idk(){
  //    RobotContainer.m_driverController.b()
  //    .onTrue(
  //       Commands.runOnce(() -> {
  //               setShooterVelocity(ShooterConstants.kOverrideSpeed);
  //       }))
        
  //     .onFalse(
  //       Commands.runOnce(() -> {
  //               setAutoGoalEnabled(true);
  //       }));
  // }

  //override
  public void toggleAutoGoal() {
    autoGoalEnabled = !autoGoalEnabled;
    shooterMotorLeader.set(ShooterConstants.kOverrideSpeed);
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
    shooterMotorLeader.setControl(m_motionRequest.withVelocity(velocity));
    shooterMotorFollower.setControl(m_motionRequest2.withVelocity(velocity));
  }

  public void setAutoGoalEnabled(boolean enabled) {
    autoGoalEnabled = enabled;
  }

  public boolean isAtSetpoint() {
    return Math.abs(shooterMotorLeader.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance;
  }
  
  private void logMotorData() {
    Logger.recordOutput("Subsystems/Shooter/ShooterState", currentState.name());

    Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterMotorVelocity", shooterMotorLeader.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Velocity/ShooterSetpoint", m_motionRequest.Velocity);
    Logger.recordOutput("Subsystems/Shooter/Velocity/IsAtSetpoint", Math.abs(shooterMotorLeader.getVelocity().getValueAsDouble() - m_motionRequest.Velocity) <= ShooterConstants.kVelocityTolerance);

    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorSupplyCurrent", shooterMotorLeader.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorStatorCurrent", shooterMotorFollower.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Shooter/Basic/ShooterMotorVoltage", shooterMotorLeader.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Shooter/Tracking/Blue/HubDistance", m_blueHubDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/Blue/DepotShuttlingDistance", m_blueDepotShuttlingDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/Blue/OutpostShuttlingDistance", m_blueOutpostShuttlingDistance);

    Logger.recordOutput("Subsystems/Shooter/Tracking/Red/HubDistance", m_redHubDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/Red/DepotShuttlingDistance", m_redDepotShuttlingDistance);
    Logger.recordOutput("Subsystems/Shooter/Tracking/Red/OutpostShuttlingDistance", m_redOutpostShuttlingDistance);
  }
}