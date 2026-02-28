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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  // private DoubleSolenoid solenoid;
  // private Compressor compressor;
  //private PneumaticHub pneumaticHub;
  private TalonFX climberMotor;
  private TalonFXConfiguration climberConfig;
  private ClimberState currentState = ClimberState.OFF;
  
  /** Creates a new Climber. */
  public Climber() {
    // pneumaticHub = new PneumaticHub(ClimberConstants.kPHcanId);
    // solenoid = pneumaticHub.makeDoubleSolenoid(ClimberConstants.kForwardChannel, ClimberConstants.kReverseChannel);
    // compressor = pneumaticHub.makeCompressor();
    climberMotor = new TalonFX(ClimberConstants.kMotorId);
    climberConfig = new TalonFXConfiguration()
                          .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                          .withCurrentLimits(new CurrentLimitsConfigs()
                                                  .withSupplyCurrentLimit(ClimberConstants.kSupplyCurrentLimit));
    climberMotor.getConfigurator().apply(climberConfig);       
  }

  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch(desiredState){
      case EXTEND:
        // solenoid.set(DoubleSolenoid.Value.kForward);
        climberExtend();
        break;
      case RETRACT:
        // solenoid.set(DoubleSolenoid.Value.kReverse);
        climberRetract();
        break;
      case OFF:
        // solenoid.set(DoubleSolenoid.Value.kOff);
        climberStop();
        break;
    }
  }

  public void climberExtend() {
    climberMotor.set(ClimberConstants.kSpeed);
  }

  public void climberRetract() {
    climberMotor.set(-ClimberConstants.kSpeed);
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
    // SmartDashboard.putData("Subsystems/Climber/Compressor", compressor);
    // SmartDashboard.putData("Subsystems/Climber/Solenoid", solenoid);
  }

  @Override
  public void periodic() {
    logMotorData();
    //compressor.disable();
  }
}
