// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  rollerConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(IntakeConstants.kIntakeSupplyCurrentLimit));

  rollerMotor.getConfigurator().apply(rollerConfig);


  pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);

  pivotConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                          .withInverted(InvertedValue.CounterClockwise_Positive)
                                          .withNeutralMode(NeutralModeValue.Coast))
                        .withSlot0(new Slot0Configs()
                                    .withKP(IntakeConstants.kP)
                                    .withKI(IntakeConstants.kI)
                                    .withKD(IntakeConstants.kD))
                        .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(IntakeConstants.kCruiseVelocity)
                                        .withMotionMagicAcceleration(IntakeConstants.kAcceleration))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(IntakeConstants.kPivotSupplyCurrentLimit));
  
  pivotMotor.getConfigurator().apply(pivotConfig);

  m_motionRequest = new MotionMagicVoltage(0).withSlot(0);

  pivotMotor.setPosition(0);

  SmartDashboard.putData("Overrides/Zero Intake Pivot", runOnce(this::zeroIntakePivot).ignoringDisable(true).withName("Zero Intake Pivot"));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();

    if (currentState == IntakeState.INTAKE || currentState == IntakeState.OUTTAKE || currentState == IntakeState.DOWN) {
      if (isAtIntakeSetpoint()) {
        pivotMotor.stopMotor();
      }else{
        pivotMotor.setControl(m_motionRequest.withPosition(IntakeConstants.kIntakePivotIntakePosition));
      }
    }

  }

  public void setGoal(IntakeState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case INTAKE:
        //Pivot handled in periodic to allow for stopping at setpoint
        rollerMotor.set(IntakeConstants.kIntakeInSpeed);
        break;
      case OUTTAKE:
        //Pivot handled in periodic to allow for stopping at setpoint
        rollerMotor.set(IntakeConstants.kIntakeOutSpeed);
        break;
      case DOWN:
        //Pivot handled in periodic to allow for stopping at setpoint
        rollerMotor.stopMotor();
        break;
      case AGITATE:
        pivotMotor.setControl(m_motionRequest.withPosition(IntakeConstants.kIntakePivotAgitatePosition));
        // rollerMotor.set(IntakeConstants.kIntakeInSpeed);
        break;
      case STOW:
        pivotMotor.setControl(m_motionRequest.withPosition(IntakeConstants.kIntakePivotStowPosition));
        rollerMotor.stopMotor();
        break;
      case STOP:
        rollerMotor.stopMotor();
        break;
    }
  }

  public boolean isAtIntakeSetpoint() {
    return Math.abs(pivotMotor.getPosition().getValueAsDouble() - IntakeConstants.kIntakePivotIntakePosition) < IntakeConstants.kPivotTolerance;
  }

  public boolean isIntaking() {
    return currentState == IntakeState.INTAKE;
  }

  public void setPivotBrakeMode(boolean brake) {
    pivotMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public void zeroIntakePivot() {
    pivotMotor.setPosition(0);
  }

  private void logMotorData(){
    Logger.recordOutput("Subsystems/Intake/IntakeState", currentState.name());

    Logger.recordOutput("Subsystems/Intake/Basic/Roller/MotorSpeed", rollerMotor.get());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/MotorSupplyCurrent", rollerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/MotorStatorCurrent", rollerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/MotorVoltage", rollerMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorVelocity", pivotMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorSupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorStatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Intake/Position/Pivot/MotorPosition", pivotMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Position/Pivot/MotorSetpoint", IntakeConstants.kIntakePivotIntakePosition);
    Logger.recordOutput("Subsystems/Intake/Position/Pivot/IsAtSetpoint", Math.abs(pivotMotor.getPosition().getValueAsDouble() - m_motionRequest.Position) < IntakeConstants.kPivotTolerance);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
