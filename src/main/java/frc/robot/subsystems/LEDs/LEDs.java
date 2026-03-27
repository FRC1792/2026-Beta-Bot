// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private CANdle m_CANdle;
  private CANdleConfiguration m_CANdleConfig;

  private LEDStates currentState = LEDStates.OFF;
  
  /** Creates a new LEDsubsystem. */
  public LEDs() {
    m_CANdle = new CANdle(LEDConstants.kCANdleId);
    m_CANdleConfig = new CANdleConfiguration();

    m_CANdle.getConfigurator().apply(m_CANdleConfig);
  }

  public void setPattern(LEDStates desiredPattern) {
    currentState = desiredPattern;
    switch (desiredPattern) {
      case CAN_SHOOT:
        m_CANdle.setControl(new SingleFadeAnimation(LEDConstants.kStartIndex, LEDConstants.kEndIndex).withColor(LEDConstants.kGreen));
        break;
      case TEN_SECONDS:
        m_CANdle.setControl(new StrobeAnimation(LEDConstants.kStartIndex, LEDConstants.kEndIndex).withColor(LEDConstants.kRed));
        break;
      case DEFAULT:
        m_CANdle.setControl(new SingleFadeAnimation(LEDConstants.kStartIndex, LEDConstants.kEndIndex).withColor(LEDConstants.kBlue));
        break;
      case OFF:
      m_CANdle.clearAllAnimations();
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
