// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private AddressableLED o_led;
  private AddressableLEDBuffer o_ledBuffer;
  private int v_rainbowFirstPixelHue;
  private int length = 40; // each strand is 20
  private int counter = 0;
  private int chaseCounter = 0;
  // private int[] lowerBounds = {0, 8, 16, 24, 32}; //Each chase strip is 5 long
  // private int[] upperBounds = {4, 12, 20, 28, 36};

  private int[] lowerBounds = { 0, 10, 20, 30 }; // Each chase strip is 5 long
  private int[] upperBounds = { 1, 11, 21, 31 };

  public LEDSubsystem() {

    o_led = new AddressableLED(LEDConstants.kLEDPort);
    o_ledBuffer = new AddressableLEDBuffer(length);
    o_led.setLength(o_ledBuffer.getLength());
    o_led.setData(o_ledBuffer);
    o_led.start();
    setOff();

  }

  public void setOff() {
    for (var i = 0; i < o_ledBuffer.getLength(); i++) {
      o_ledBuffer.setRGB(i, 0, 0, 0);
    }
    o_led.setData(o_ledBuffer);
  }

  public void setSolidColor(int r, int g, int b) {
    for (var i = 0; i < o_ledBuffer.getLength(); i++) {
      o_ledBuffer.setRGB(i, g, r, b);
    }
    o_led.setData(o_ledBuffer);
  }

  public void blinkSolidColor(int r, int g, int b) {
    counter++; // This probably should be done with a timer, but I hate timers!
    if (counter < 10) {
      setSolidColor(r, g, b);
    } else {
      setOff();
    }
    if (counter > 20) {
      counter = 0;
    }
  }

  public void setColorChase(int mainHue, int chaseHue, double pulseSpeed) {
    int hue = mainHue;

    for (var i = 0; i < o_ledBuffer.getLength(); i++) {
      boolean foundWithinBounds = false;
      // Check if i is within the chase bound
      for (int j = 0; j < lowerBounds.length; j++) {
        if (lowerBounds[j] <= i && i <= upperBounds[j]) {
          hue = chaseHue;
          foundWithinBounds = true;
        } else {
          if (!foundWithinBounds) {
            hue = mainHue;
          }
        }
      }
      o_ledBuffer.setHSV(i, hue, 255, 128);
    }
    o_led.setData(o_ledBuffer);
    if (chaseCounter > 10 - pulseSpeed) {
      for (int i = 0; i < lowerBounds.length; i++) {
        lowerBounds[i] = lowerBounds[i] + 1;
        upperBounds[i] = upperBounds[i] + 1;
        // The stand has reached the end of the line, reset

        if (upperBounds[i] > 40) {
          lowerBounds[i] = 0;
          upperBounds[i] = 1;
        }
      }
      chaseCounter = 0;
    }

    chaseCounter++;
  }

  public void setRainbow(int pulseSpeed) {
    for (var i = 0; i < o_ledBuffer.getLength(); i++) {
      final var hue = (v_rainbowFirstPixelHue + (i * 180 / o_ledBuffer.getLength())) % 180;
      o_ledBuffer.setHSV(i, hue, 255, 128);
    }

    v_rainbowFirstPixelHue += pulseSpeed; // Increase by to make the rainbow "move"

    v_rainbowFirstPixelHue %= 180; // Check bounds
    o_led.setData(o_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // setRainbow(1);
  }
}
