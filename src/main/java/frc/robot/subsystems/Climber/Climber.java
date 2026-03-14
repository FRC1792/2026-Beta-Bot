// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  private TalonFX climberMotor;
  private TalonFXConfiguration climberConfig;
  private ClimberState currentState = ClimberState.RETRACT;
  
  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(ClimberConstants.kMotorId);
    climberConfig = new TalonFXConfiguration()
                          .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                          .withCurrentLimits(new CurrentLimitsConfigs()
                                                  .withSupplyCurrentLimit(ClimberConstants.kSupplyCurrentLimit));
    climberMotor.getConfigurator().apply(climberConfig)
                          .withSoftwareLimitSwitchs(new SoftwareLimitSwitchConfigs()
                          .withForwardSoftLimitEnable(true)
                          .withForwardSoftLimitThreshold(ClimberConstants.kMaxExtension)
                          .withReverseSoftLimitEnable(true)
                          .withReverseSoftLimitThreshold(ClimberConstants.kMinExtension));
  }

  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch(desiredState){
      case EXTEND:
        climberExtend();
        break;
      case RETRACT:
        climberRetract();
        break;
      case OFF:
        climberStop();
        break;
    }
  }

  public void climberExtend() {
    if (climberMotor.getPosition().getValueAsDouble() < ClimberConstants.kMaxExtension) {
      climberMotor.set(ClimberConstants.kSpeed);
    }
  }

  public void climberRetract() {
    if (climberMotor.getPosition().getValueAsDouble() > ClimberConstants.kMinExtension) {
      climberMotor.set(-ClimberConstants.kSpeed);
    }
  }

  public void climberStop() {
    climberMotor.stopMotor();
  }

  public void logMotorData() {
    Logger.recordOutput("Subsystems/Climber/ClimberState", currentState.name());

    Logger.recordOutput("Subsystems/Climber/Basic/MotorSpeed", climberMotor.get());
    Logger.recordOutput("Subsystems/Climber/Basic/MotorSupplyCurrent", climberMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Climber/Basic/MotorStatorCurrent", climberMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Climber/Basic/MotorVoltage", climberMotor.getMotorVoltage().getValueAsDouble());

  @Override
  public void periodic() {
    logMotorData();
  }
}
