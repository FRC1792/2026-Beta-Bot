// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class Indexer extends SubsystemBase {

  private TalonFX spindexerMotor;
  private TalonFXConfiguration spindexerConfig;

  private TalonFX indexerMotor;
  private TalonFXConfiguration indexerConfig;

  private CANrange indexerSensor;
  private CANrangeConfiguration indexerSensorConfig;

  private IndexerState currentState = IndexerState.STOP;

  private MotionMagicVelocityVoltage m_motionRequest;
  private VoltageOut m_voltageRequest;


  /** Creates a new Indexer. */
  public Indexer() {
    spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorId);

    spindexerConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withNeutralMode(NeutralModeValue.Brake)
                                              .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withSlot0(new Slot0Configs()
                              .withKP(0.11353)
                              .withKI(0)
                              .withKD(0)
                              .withKA(0.016981)
                              .withKV(0.14126)
                              .withKS(-0.075292))
                      .withMotionMagic(new MotionMagicConfigs()
                                      .withMotionMagicCruiseVelocity(1000)
                                      .withMotionMagicAcceleration(1000))
                      .withCurrentLimits(new CurrentLimitsConfigs()
                                      .withSupplyCurrentLimit(IndexerConstants.kSpindexerSupplyCurrentLimit));
    spindexerMotor.getConfigurator().apply(spindexerConfig);

    indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorId);

    indexerConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withNeutralMode(NeutralModeValue.Brake)
                                              .withInverted(InvertedValue.CounterClockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(IndexerConstants.kIndexerSupplyCurrentLimit));

    indexerSensor = new CANrange(IndexerConstants.kIndexerSensorId);
    
    indexerSensorConfig = new CANrangeConfiguration()
                        .withProximityParams(new ProximityParamsConfigs()
                                              .withMinSignalStrengthForValidMeasurement(IndexerConstants.kIndexerSensorMinSignalStrength)
                                              .withProximityThreshold(IndexerConstants.kIndexerSensorProximityThreshold))
                        .withToFParams(new ToFParamsConfigs()
                                              .withUpdateMode(UpdateModeValue.ShortRange100Hz));
    indexerSensor.getConfigurator().apply(indexerSensorConfig);

    m_voltageRequest = new VoltageOut(0);

    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withEnableFOC(true);
  }

  private final SysIdRoutine m_sysIdRoutine = 
    new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        Seconds.of(10),
        (state) -> SignalLogger.writeString("Spindexer State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> spindexerMotor.setControl(m_voltageRequest.withOutput(volts.in(Volts))),
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();
  }

  public void setGoal(IndexerState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case SPINDEX:
        spindexerMotor.setControl(m_motionRequest.withVelocity(IndexerConstants.kSpindexerInSpeed));
        indexerMotor.set(IndexerConstants.kIndexerInSpeed);
        break;
      case OUTTAKE:
        spindexerMotor.set(IndexerConstants.kSpindexerOutSpeed);
        indexerMotor.set(IndexerConstants.kIndexerOutSpeed);
        break;
      case IDLE:
        if(indexerSensor.getIsDetected().getValue()){
          spindexerMotor.stopMotor();
          indexerMotor.stopMotor();
        } else {
          spindexerMotor.set(IndexerConstants.kSpindexerInSpeed);
          indexerMotor.set(IndexerConstants.kIndexerInSpeed);
        }
        break;
      case STOP:
        spindexerMotor.stopMotor();
        indexerMotor.stopMotor();
        break;
    }
  }

  private void logMotorData(){
    DogLog.log("Subsystems/Indexer/IndexerState", currentState.name());
    
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorVelocity", spindexerMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorSupplyCurrent", spindexerMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorStatorCurrent", spindexerMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorVoltage", spindexerMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Subsystems/Indexer/Basic/Indexer/IndexerMotorVelocity", indexerMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Indexer/IndexerMotorSupplyCurrent", indexerMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Indexer/IndexerMotorStatorCurrent", indexerMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Indexer/IndexerMotorVoltage", indexerMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Subsystems/Indexer/Basic/Indexer/IndexerSensor", indexerSensor.getIsDetected().getValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
