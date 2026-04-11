// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private TalonFX rollerMotor;
  private TalonFXConfiguration rollerConfig;

  private TalonFX rollerMotor2;

  private TalonFX pivotMotor;
  private TalonFXConfiguration pivotConfig;

  private DutyCycleEncoder throughBorePivot;

  private MotionMagicVoltage m_motionRequest;

  private IntakeState currentState = IntakeState.STOP;

  private double crescendoAmplitude = 0;
  private double crescendoTargetPosition = 0;
  private boolean crescendoGoingTowardStow = true;

  /** Creates a new Intake. */
  public Intake() {

  rollerMotor = new TalonFX(IntakeConstants.kIntakeMotorId);

  rollerConfig = new TalonFXConfiguration()
                      .withMotorOutput(new MotorOutputConfigs()
                                            .withInverted(InvertedValue.Clockwise_Positive)
                                            .withNeutralMode(NeutralModeValue.Brake))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimit(IntakeConstants.kIntakeSupplyCurrentLimit)
                                            .withStatorCurrentLimit(IntakeConstants.kIntakeStatorCurrentLimit));

  rollerMotor.getConfigurator().apply(rollerConfig);

  rollerMotor2 = new TalonFX(IntakeConstants.kIntakeMotor2Id);
  rollerMotor2.getConfigurator().apply(rollerConfig);
  rollerMotor2.setControl(new Follower(rollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));

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
  
  throughBorePivot = new DutyCycleEncoder(IntakeConstants.kEncoderChannel);

  m_motionRequest = new MotionMagicVoltage(0).withSlot(0);

  pivotMotor.setPosition(0);

  SmartDashboard.putData("Overrides/Zero Intake Pivot", runOnce(this::zeroIntakePivot).ignoringDisable(true).withName("Zero Intake Pivot"));

  SmartDashboard.putBoolean("Overrides/Crescendo Enabled", true);

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

    if(SmartDashboard.getBoolean("Overrides/Crescendo Enabled", true)){
      if (currentState == IntakeState.CRESCENDO) {
        boolean atTarget = Math.abs(throughBorePivot.get() - crescendoTargetPosition) < IntakeConstants.kPivotTolerance;

        if (atTarget) {
          if (crescendoGoingTowardStow) {
            if (throughBorePivot.get() <= -13) {
              rollerMotor.set(IntakeConstants.kIntakeInSpeed);
            }else{
              rollerMotor.stopMotor();
            }
            // Reached the stow position, now go back to start
            crescendoGoingTowardStow = false;
            crescendoTargetPosition = IntakeConstants.kCrescendoStartPosition;
          } else {
            
            if (throughBorePivot.get() <= -13) {
              rollerMotor.set(IntakeConstants.kIntakeInSpeed);
            }else{
              rollerMotor.stopMotor();
            }
            // Reached start position, increase amplitude and go toward stow again
            crescendoGoingTowardStow = true;
            crescendoAmplitude = Math.min(
                crescendoAmplitude + IntakeConstants.kCrescendoAmplitudeStep,
                IntakeConstants.kCrescendoMaxAmplitude);
            crescendoTargetPosition = IntakeConstants.kCrescendoStartPosition - crescendoAmplitude;
          }
        }

        pivotMotor.setControl(m_motionRequest.withPosition(crescendoTargetPosition));

        Logger.recordOutput("Subsystems/Intake/Crescendo/Amplitude", crescendoAmplitude);
        Logger.recordOutput("Subsystems/Intake/Crescendo/TargetPosition", crescendoTargetPosition);
        Logger.recordOutput("Subsystems/Intake/Crescendo/GoingTowardStow", crescendoGoingTowardStow);
      }
    }

  }

  public void setGoal(IntakeState desiredState) {
    boolean enteringCrescendo = desiredState == IntakeState.CRESCENDO && currentState != IntakeState.CRESCENDO;

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
      case CRESCENDO:
        if (enteringCrescendo) {
          // Reset crescendo state when entering
          crescendoAmplitude = IntakeConstants.kCrescendoMinAmplitude;
          crescendoTargetPosition = IntakeConstants.kCrescendoStartPosition - crescendoAmplitude;
          crescendoGoingTowardStow = true;
         }
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
    return Math.abs(throughBorePivot.get() - IntakeConstants.kIntakePivotIntakePosition) < IntakeConstants.kPivotTolerance;
  }

  public boolean isIntaking() {
    return currentState == IntakeState.INTAKE;
  }

  public IntakeState getCurrentState() {
    return currentState;
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

    Logger.recordOutput("Subsystems/Intake/Basic/Roller/Motor2Speed", rollerMotor2.get());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/Motor2SupplyCurrent", rollerMotor2.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/Motor2StatorCurrent", rollerMotor2.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Roller/Motor2Voltage", rollerMotor2.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorVelocity", pivotMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorSupplyCurrent", pivotMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorStatorCurrent", pivotMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Intake/Basic/Pivot/MotorVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Intake/Position/Pivot/MotorPosition", throughBorePivot.get());
    Logger.recordOutput("Subsystems/Intake/Position/Pivot/MotorSetpoint", IntakeConstants.kIntakePivotIntakePosition);
    Logger.recordOutput("Subsystems/Intake/Position/Pivot/IsAtSetpoint", Math.abs(throughBorePivot.get() - m_motionRequest.Position) < IntakeConstants.kPivotTolerance);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
