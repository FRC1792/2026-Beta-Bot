// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  private DoubleSolenoid solenoid;
  private Compressor compressor;
  private PneumaticHub pneumaticHub;
  private ClimberState currentState = ClimberState.RETRACT;
  
  /** Creates a new Climber. */
  public Climber() {
    pneumaticHub = new PneumaticHub(ClimberConstants.kPHcanId);
    solenoid = pneumaticHub.makeDoubleSolenoid(ClimberConstants.kForwardChannel, ClimberConstants.kReverseChannel);
    compressor = pneumaticHub.makeCompressor();
  }

  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch(desiredState){
      case EXTEND:
        solenoid.set(DoubleSolenoid.Value.kForward);
        break;
      case RETRACT:
        solenoid.set(DoubleSolenoid.Value.kReverse);
        break;
      case OFF:
        solenoid.set(DoubleSolenoid.Value.kOff);
        break;
    }
  }

  public void logPneumaticsData() {
    Logger.recordOutput("Subsystems/Climber/ClimberState", currentState.name());
    SmartDashboard.putData("Subsystems/Climber/Compressor", compressor);
    SmartDashboard.putData("Subsystems/Climber/Solenoid", solenoid);
  }

  @Override
  public void periodic() {
    // logPneumaticsData();
    compressor.enableDigital();
  }
}
