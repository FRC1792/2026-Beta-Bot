// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.TurretConstants;

public class Intake extends SubsystemBase {

  private TalonFX rollerMotor;
  private TalonFXConfiguration rollerConfig;

  private TalonFX pivotMotor;
  private TalonFXConfiguration pivotConfig;

  private MotionMagicVoltage m_motionRequest;

  private IntakeState currentState = IntakeState.STOP;

  /** Creates a new Intake. */
  public Intake() {

  rollerMotor = new TalonFX(IntakeConstants.kIntakeMotorId);

  pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);

  rollerConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive) //Set motor inversion based on mechanism
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(IntakeConstants.kIntakeSupplyCurrentLimit));

  pivotConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(IntakeConstants.kPivotSupplyCurrentLimit));
  
  rollerMotor.getConfigurator().apply(rollerConfig);

  pivotMotor.getConfigurator().apply(pivotConfig);

  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();
    if (currentState == IntakeState.INTAKE || currentState == IntakeState.DOWN || currentState == IntakeState.OUTTAKE) {
      if (isAtIntakeSetpoint()) {
        pivotMotor.stopMotor();
      } else {
        pivotMotor.setControl(m_motionRequest);
      }
    }
  }

  public void setGoal(IntakeState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case INTAKE:
        rollerMotor.set(IntakeConstants.kIntakeInSpeed);
        break;
      case OUTTAKE:
        rollerMotor.set(IntakeConstants.kIntakeOutSpeed);
        break;
      case STOP:
        rollerMotor.stopMotor(); //stop the rollers
        break;
      case DOWN:
        break;
      case STOW:
        rollerMotor.stopMotor();
        break;
    }
  }

  private boolean isAtIntakeSetpoint() {
    return Math.abs((pivotMotor.getPosition().getValueAsDouble()) - (m_motionRequest.Position)) <= IntakeConstants.kTolerance;
  }

  private void logMotorData(){
    Logger.recordOutput("Subsystems/Intake/IntakeState", currentState.name());
    Logger.recordOutput("Subsystems/Intake/Basic/RollerMotorSpeed", rollerMotor.get());
    Logger.recordOutput("Subsystems/Intake/Basic/RollerMotorSupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/RollerMotorStatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/RollerMotorVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());
    
    Logger.recordOutput("Subsystems/Intake/Basic/PivotMotorSpeed", pivotMotor.get());
    Logger.recordOutput("Subsystems/Intake/Basic/PivotMotorSupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/PivotMotorStatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/PivotMotorVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
