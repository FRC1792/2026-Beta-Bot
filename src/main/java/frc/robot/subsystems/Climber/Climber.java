// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticHub;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private ClimberState currentState = ClimberState.STOP;
  
  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new TalonFX(ClimberConstants.kMotorId);
    climberConfig = new TalonFXConfiguration()
                          .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                          .withCurrentLimits(new CurrentLimitsConfigs()
                                                  .withSupplyCurrentLimit(ClimberConstants.kSupplyCurrentLimit)
                                                  .withStatorCurrentLimit(ClimberConstants.kStatorCurrentLimit))
                                                  
                          .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                                                  .withForwardSoftLimitEnable(true)
                                                  .withForwardSoftLimitThreshold(ClimberConstants.kMaxExtension)
                                                  .withReverseSoftLimitEnable(true)
                                                  .withReverseSoftLimitThreshold(ClimberConstants.kMinExtension));
    climberMotor.getConfigurator().apply(climberConfig);
    
    climberMotor.setPosition(0);
  }

  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch(desiredState){
      case EXTEND:
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        climberMotor.getConfigurator().apply(climberConfig);
        climberMotor.set(ClimberConstants.kSpeed);
        break;
      case RETRACT:
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        climberMotor.getConfigurator().apply(climberConfig);
        climberMotor.set(-ClimberConstants.kSpeed);
        break;
      case STOP:
        climberMotor.stopMotor();
        break;
      case MANUAL_UP:
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        climberMotor.getConfigurator().apply(climberConfig);
        climberMotor.set(ClimberConstants.kSpeed);
        break;
      case MANUAL_DOWN:
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        climberMotor.getConfigurator().apply(climberConfig);
        climberMotor.set(-ClimberConstants.kSpeed);
        break;
    }
  }

  public void logMotorData() {
    Logger.recordOutput("Subsystems/Climber/ClimberState", currentState.name());

    Logger.recordOutput("Subsystems/Climber/Basic/MotorSpeed", climberMotor.get());
    Logger.recordOutput("Subsystems/Climber/Basic/MotorSupplyCurrent", climberMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Climber/Basic/MotorStatorCurrent", climberMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Climber/Basic/MotorVoltage", climberMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Climber/Position/MotorPosition", climberMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    logMotorData();
  }
}
