// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private TalonFX spindexerMotor;
  private TalonFXConfiguration spindexerConfig;

  private TalonFX indexerMotor;
  private TalonFXConfiguration indexerConfig;

  //private CANrange indexerSensor;
  //private CANrangeConfiguration indexerSensorConfig;

  private IndexerState currentState = IndexerState.STOP;

  /** Creates a new Indexer. */
  public Indexer() {
    spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorId);

    spindexerConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withNeutralMode(NeutralModeValue.Brake)
                                              .withInverted(InvertedValue.Clockwise_Positive))
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
    indexerMotor.getConfigurator().apply(indexerConfig);


    //indexerSensor = new CANrange(IndexerConstants.kIndexerSensorId);
    
    // indexerSensorConfig = new CANrangeConfiguration()
    //                     .withProximityParams(new ProximityParamsConfigs()
    //                                           .withMinSignalStrengthForValidMeasurement(IndexerConstants.kIndexerSensorMinSignalStrength)
    //                                           .withProximityThreshold(IndexerConstants.kIndexerSensorProximityThreshold))
    //                     .withToFParams(new ToFParamsConfigs()
    //                                           .withUpdateMode(UpdateModeValue.ShortRange100Hz));
    //indexerSensor.getConfigurator().apply(indexerSensorConfig);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // logMotorData();
  }

  public void setGoal(IndexerState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case SPINDEX:
        spindexerMotor.set(IndexerConstants.kSpindexerInSpeed);
        indexerMotor.set(IndexerConstants.kIndexerInSpeed);
        break;
      case OUTTAKE:
        spindexerMotor.set(IndexerConstants.kSpindexerOutSpeed);
        indexerMotor.set(IndexerConstants.kIndexerOutSpeed);
        break;
      // case IDLE:
      //   if(indexerSensor.getIsDetected().getValue()){
      //     spindexerMotor.stopMotor();
      //     indexerMotor.stopMotor();
      //   } else {
      //     spindexerMotor.set(IndexerConstants.kSpindexerInSpeed);
      //     indexerMotor.set(IndexerConstants.kIndexerInSpeed);
      //   }
      //   break;
      case STOP:
        spindexerMotor.stopMotor();
        indexerMotor.stopMotor();
        break;
    }
  }

  private void logMotorData(){
    Logger.recordOutput("Subsystems/Indexer/IndexerState", currentState.name());
    
    Logger.recordOutput("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorVelocity", spindexerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorSupplyCurrent", spindexerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorStatorCurrent", spindexerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorVoltage", spindexerMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Subsystems/Indexer/Basic/Indexer/IndexerMotorVelocity", indexerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/Indexer/IndexerMotorSupplyCurrent", indexerMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/Indexer/IndexerMotorStatorCurrent", indexerMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Indexer/Basic/Indexer/IndexerMotorVoltage", indexerMotor.getMotorVoltage().getValueAsDouble());

    //Logger.recordOutput("Subsystems/Indexer/Basic/Indexer/IndexerSensor", indexerSensor.getIsDetected().getValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
