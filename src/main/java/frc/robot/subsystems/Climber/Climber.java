// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import dev.doglog.DogLog;

public class Climber extends SubsystemBase {
  private Solenoid solenoid;
  //private Compressor compressor;
  private PneumaticHub pneumaticHub;
  private ClimberState currentState = ClimberState.RETRACT;
  
  /** Creates a new Climber. */
  public Climber() {
    pneumaticHub = new PneumaticHub(ClimberConstants.kPHcanId);
    solenoid = pneumaticHub.makeSolenoid(ClimberConstants.kSolenoidChannel);
    //compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  }

  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch(desiredState){
      case EXTEND:
        solenoid.set(true);
        break;
      case RETRACT:
        solenoid.set(false);
        break;
    }
  }

  public void logPneumaticsData() {
    DogLog.log("Subsystems/Climber/ClimberState", currentState.name());
    DogLog.log("Subsystems/Climber/SolenoidOpen?", solenoid.get());
    /* Potentially more log stuff */
  }

  @Override
  public void periodic() {
    logPneumaticsData();
    //compressor.enableDigital();
  }
}
