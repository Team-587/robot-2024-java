// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightsSubsystem extends SubsystemBase {
//Define the variables and arrays used to controll the light sequences

  boolean blueACtive;
  boolean readyToRumble = false;
  int ledLoopCount;
  int disableLoopCount;
  boolean haveNoteLight;
  AddressableLED m_led = new AddressableLED(Constants.OperatorConstants.kLEDPort);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);


//Set lights to base color

  /** Creates a new LightsSubsystem. */
  public LightsSubsystem() {
    m_led.SetLength(m_ledBuffer.getLength());


  }

  public void baseLoop() {
    for(int i = 0; i < m_ledBuffer.getLength(); i = i + 3) {
      m_ledBuffer.setHSV(i, 238, 98, 38);
      m_ledBuffer.setHSV(i + 1, 200, 100, 79);
      m_ledBuffer.setHSV(i + 2, 44, 99, 99);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    

  }
}
