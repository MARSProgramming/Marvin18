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
package frc.alotobots.library.subsystems.vision.photonvision.objectdetection.constants;

import edu.wpi.first.math.geometry.Transform3d;
import frc.alotobots.library.subsystems.vision.photonvision.objectdetection.util.GameElement;
import lombok.experimental.UtilityClass;

@SuppressWarnings("resource")
@UtilityClass
public class ObjectDetectionConstants {
  // Camera mounting offsets (FORWARD: +, LEFT: +, UP: +)
  private static final Transform3d[] CAMERA_OFFSETS = new Transform3d[] {};

  // Camera configurations
  public static final CameraConfig[] CAMERA_CONFIGS = {};

  // Game elements
  // Game elements array indexed by class ID
  // Index 0 = Note (class ID 0)
  public static final GameElement[] GAME_ELEMENTS = new GameElement[] {};

  // TUNE ON COMP. DAY!!
  /**
   * The number of detections to save in history, the smaller the better, but still needs to be
   * enough to ensure we have a good detection. Remember, the loop time is about 20ms 1s = 50 frames
   */
  public static final int HISTORY_LENGTH = 50;

  /**
   * Number of detections in history that need to be detected with that object for it to be
   * considered stable
   */
  public static final int REQUIRED_DETECTIONS = 20;

  /**
   * Number of missed detections IN A ROW of a stable object for it to be removed from the stable
   * list
   */
  public static final int MISSING_FRAMES_THRESHOLD = 15;

  /*** Scale Factor. Tune this if the ObjectDetection is consistently under/over reporting */
  public static final double SCALE_FACTOR = 1.15;
}
