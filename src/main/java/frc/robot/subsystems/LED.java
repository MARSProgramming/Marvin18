package frc.robot.subsystems;

//import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 Colors:
Flashing yellow
Flashing white
Flashing green
Red
Blue
Green
Rainbow 

4 distinct LED strips
One on the intake
Two for left/right climbers
One for top of climber

Global LED behaviors
Intaking -flashing yellow
Scoring coral - flashing white
Scoring algae - flashing green
Current command to score indicator
LEDs on climber that indicate what level the robot will automatically score at (for driver confirmation)
LEDs that indicate when the robot is at the correct pose for automatic scoring (if scoring is not integrated into alignment)

Aesthetic LED actions
When climbing, climb leds “continue up” 
When fully climbed, rainbow leds
When robot disabled, everything red (unless we want to do auto alignment in disabled using the camera)

 */

enum LEDState {
FLASHING_YELLOW,
FLASHING_WHITE,
FLASHING_GREEN, 
RED, 
BLUE,
GREEN,
WHITE,
YELLOW,
RAINBOW,
OFF
  }  

public class LED extends SubsystemBase {
  LEDState state = LEDState.RAINBOW;
  AddressableLED m_led = new AddressableLED(0);
  AddressableLEDBuffer m_ledBuffer;

 // private static final Random RANDOM = new Random();

  public LED() {
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(14);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    //leds.start();

  }

  public Command setRainbow(){
    return run(
      () -> {
        state = LEDState.RAINBOW;
      }
    );
  }

  public Command setWhite(){
    return run(
      () -> {
        state = LEDState.WHITE;
      }
    );
  }

  public Command setOff(){
    return run(
      () -> {
        state = LEDState.OFF;
      }
    );
  }

  public Command setYellow(){
    return run(
      () -> {
        state = LEDState.YELLOW;
      }
    );
  }

  public Command setRed(){
    return run(
      () -> {
        state = LEDState.RED;
      }
    );
  }
  public Command setblue(){
    return run(
      () -> {
        state = LEDState.BLUE;
      }
    );
  }
  public Command setGreen(){
    return run(
      () -> {
        state = LEDState.GREEN;
      }
    );
  }
  public Command setYellowFlashing(){
    return run(
      () -> {
        state = LEDState.FLASHING_YELLOW;
      }
    );
  }
  public Command setWhiteFlashing(){
    return run(
      () -> {
        state = LEDState.FLASHING_WHITE;
      }
    );
  }
  public Command setGreenFlashing(){
    return run(
      () -> {
        state = LEDState.FLASHING_GREEN;
      }
    );
  }
  public void setFlashing(boolean flash){
    if(flash){
        if(state == LEDState.GREEN){
            state = LEDState.FLASHING_GREEN;
        }
        if(state == LEDState.YELLOW){
            state = LEDState.FLASHING_YELLOW;
        }
        if(state == LEDState.WHITE) {
          state = LEDState.FLASHING_WHITE;
        }
    }
  }

  public Command setFlashingCommand(boolean flash){
    return run(
        () -> {
            setFlashing(flash);
        }
    );
  }

  @Override
  public void periodic() {
        /*for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
   
   m_led.setData(m_ledBuffer);*/
   if (state == LEDState.RAINBOW) {
    rainbow();
   } 
   else if (state == LEDState.WHITE) {
    white();
   }
   else if (state == LEDState.OFF) {
    off();
   }
   else if (state == LEDState.FLASHING_GREEN) {
    green_flash();
   }
   else if (state == LEDState.GREEN) {
    green();
   }
   else if (state == LEDState.RED) {
    red();
   }
   else if (state == LEDState.WHITE) {
    white();
   }
   else if (state == LEDState.FLASHING_WHITE) {
    white_flash();
   }
   else if (state == LEDState.YELLOW) {
    yellow();
   }
   else if (state == LEDState.FLASHING_YELLOW){
    yellow_flash();
   }
   else if (state == LEDState.BLUE) {
    blue();
   }
   m_led.setData(m_ledBuffer);
  }

  double m_rainbowFirstPixelHue = 0;
  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, (int) hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void red() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);
    }

    private void blue() {
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
      m_led.setData(m_ledBuffer);
    }
    private void green() {
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 0, 255, 0);
      }
      m_led.setData(m_ledBuffer);
    }
    private int iterations = 0;

    private void green_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 5) % 2 == 0) {
          m_ledBuffer.setRGB(i, 0, 255, 0);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }

     }
     iterations++;
    }

    private void yellow_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 5) % 2 == 0) {
          m_ledBuffer.setRGB(i, 255, 255, 0);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }

     }
     iterations++;
    }
      private void yellow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          m_ledBuffer.setRGB(i, 255, 255, 0);
       }
       
       m_led.setData(m_ledBuffer);
        }

      private void white() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for red
          m_ledBuffer.setRGB(i, 255, 255, 255);
       }
       
       m_led.setData(m_ledBuffer);
        }
        private void off() {
          // For every pixel
          for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 0, 0);
         }
         
         m_led.setData(m_ledBuffer);
          }
    private void white_flash() {

      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        if ((iterations / 5) % 2 == 0) {
          m_ledBuffer.setRGB(i, 255, 255, 255);
        } else {
          m_ledBuffer.setRGB(i, 0,0,0);
        }
     }
     iterations++;
    }
  }

