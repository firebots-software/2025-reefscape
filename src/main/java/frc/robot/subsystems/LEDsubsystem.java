// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANdle candle;
  private CANdleConfiguration config;

  private int r, g, b;

  public LEDsubsystem() {
    candle = new CANdle(5);
    config = new CANdleConfiguration();

    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }
  
  public CANdle getCandle() {
    return candle;
  }

  public void setLEDcontrol(int red, int green, int blue) {
    r = red;
    g = green;
    b = blue;
  }

  public int rChannel() {
    return r;
  }
  public int gChannel() {
    return g;
  }
  public int bChannel() {
    return b;
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
