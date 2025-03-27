/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2025 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.bling;

import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.LED_OFFSET;
import static frc.alotobots.library.subsystems.bling.constants.BlingConstants.NUM_LEDS;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.alotobots.library.subsystems.bling.io.BlingIO;
import frc.alotobots.library.subsystems.bling.io.BlingIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling robot LED lighting effects. Manages LED animations and solid colors for
 * visual feedback.
 */
public class BlingSubsystem extends SubsystemBase {
  /** The IO interface for LED control */
  private final BlingIO io;

  /** Logged inputs for monitoring LED state */
  private final BlingIOInputsAutoLogged inputs = new BlingIOInputsAutoLogged();

  /**
   * Creates a new BlingSubsystem.
   *
   * @param io The BlingIO implementation to use for LED control
   */
  public BlingSubsystem(BlingIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update inputs
    io.updateInputs(inputs);
    Logger.processInputs("Bling", inputs);
  }

  /**
   * Sets a range of LEDs to display a solid color.
   *
   * @param color The color to display
   * @param from The start index
   * @param to The end index
   */
  public void setSolidColor(BlingIO.LoggedColor color, int from, int to) {
    io.setSolidColor(color, from, to);
  }

  /**
   * Sets the display LEDs to display a solid color.
   *
   * @param color The color to display
   */
  public void setSolidColor(BlingIO.LoggedColor color) {
    io.setSolidColor(color, LED_OFFSET, NUM_LEDS);
  }

  /**
   * Sets the LEDs to display an animation pattern.
   *
   * @param animation The animation pattern to display
   */
  public void setAnimation(Animation animation) {
    io.setAnimation(animation);
  }

  /** Clears all LED effects, turning off animations and solid colors. */
  public void clear() {
    io.clearAnimation();
    io.clearSolidColor(LED_OFFSET, NUM_LEDS);
  }
}
