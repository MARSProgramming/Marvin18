package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final CANdle candle;  // CANdle object to control the LED strip
    private static final int LED_COUNT = 90;  // Total number of LEDs in the strip
    private final int[] ledBuffer;  // Buffer to store LED color data (RGBA format)

    // Enum to define different sections of the LED strip
    public enum LEDSection {
        ALL(0, 90),
        CANdle(0, 8),
        R45(8, 7),
        L45(15, 6),
        LEFTVERT(21, 9),
        CROSSBAR(30, 14),
        FRONTCHUTE(44, 13),
        BACKCHUTE(57, 13);
        

        private final int start;  // Start index of the section
        private final int length; // Length of the section

        // Constructor for the enum to initialize start and length
        LEDSection(int start, int length) {
            this.start = start;
            this.length = length;
        }

        public int getStart() {
            return start;  // Getter for start index
        }

        public int getLength() {
            return length;  // Getter for length
        }
    }

    // Constructor to initialize the CANdle LED controller and configure it
    public LED(int candleID) {
        candle = new CANdle(candleID);  // Create CANdle object with the provided ID
        CANdleConfiguration config = new CANdleConfiguration();  // Configuration object for CANdle
        candle.configAllSettings(config);  // Apply configuration to CANdle
        ledBuffer = new int[LED_COUNT * 4];  // Initialize buffer to hold RGBA values for each LED
    }

    // Command to set the LED color (RGB) for the entire strip
    public Command setLEDColorCommand(int r, int g, int b) {
        return run(() -> setLEDColor(r, g, b));  // Runs the setLEDColor method
    }


    // Command to turn off all LEDs (set them to black)
    public Command turnOffLEDsCommand() {
        return run(this::turnOffLEDs);  // Runs the turnOffLEDs method
    }

    // Command to set the color of a specific section of the LED strip
    public Command setLEDColorSectionCommand(LEDSection section, int r, int g, int b) {
        return run(() -> setLEDColorSection(section, r, g, b));  // Runs the setLEDColorSection method
    }

    // Command to set a rainbow animation across the LED strip
    public Command setRainbowAnimationCommand() {
        return run(this::setRainbowAnimation);  // Runs the setRainbowAnimation method
    }

    // Command to set a strobe animation with the given color and speed
    public Command setStrobeAnimationCommand(int r, int g, int b, double speed) {
        return run(() -> setStrobeAnimation(r, g, b, speed));  // Runs the setStrobeAnimation method
    }

    // Command to set a color flow animation with the given color and direction
    public Command setColorFlowAnimationCommand(int r, int g, int b, boolean reversed) {
        return run(() -> setColorFlowAnimation(r, g, b, reversed));  // Runs the setColorFlowAnimation method
    }

    // Method to set the color of the entire LED strip (RGB)
    private void setLEDColor(int r, int g, int b) {
        setLEDColorSection(0, LED_COUNT, r, g, b, 0);  // Calls setLEDColorSection to apply color to the entire strip
    }

    // Method to turn off all LEDs (set them to black)
    private void turnOffLEDs() {
        for (int i = 0; i < LED_COUNT * 4; i++) {
            ledBuffer[i] = 0;  // Set all buffer values to 0 (off)
        }
        updateLEDs();  // Update the LEDs with the new buffer (all off)
    }

    // Method to set the color of a specific section of the LED strip (RGB)
    private void setLEDColorSection(LEDSection section, int r, int g, int b) {
        setLEDColorSection(section, r, g, b, 0);  // Calls the method with white channel set to 0 (no white)
    }

    // Method to set the color of a specific section of the LED strip (RGB + White)
    private void setLEDColorSection(LEDSection section, int r, int g, int b, int w) {
        setLEDColorSection(section.getStart(), section.getLength(), r, g, b, w);  // Calls the generic method with the section's start and length
    }

    // Method to set the color of a specific section of the LED strip (start index, length, RGB + White)
    private void setLEDColorSection(int start, int length, int r, int g, int b, int w) {
        // Ensure that the length does not exceed the total LED count
        if (start + length > LED_COUNT) {
            length = LED_COUNT - start;  // Adjust length if it goes out of bounds
        }
        // Loop through the specified section and set the color for each LED
        for (int i = 0; i < length; i++) {
            int index = (start + i) * 4;  // Calculate the index for the LED in the buffer
            ledBuffer[index] = r;  // Set red value
            ledBuffer[index + 1] = g;  // Set green value
            ledBuffer[index + 2] = b;  // Set blue value
            ledBuffer[index + 3] = w;  // Set white value
        }
        updateLEDs();  // Update the LEDs with the new buffer
    }

    // Method to update the LEDs based on the current buffer
    private void updateLEDs() {
        // Loop through each LED in the strip
        for (int i = 0; i < LED_COUNT; i++) {
            int index = i * 4;  // Calculate the index for the LED in the buffer
            // Set the LED color based on the buffer values (RGBA)
            candle.setLEDs(ledBuffer[index], ledBuffer[index + 1], ledBuffer[index + 2], ledBuffer[index + 3], i, 1);
        }
    }

    // Method to set a rainbow animation across the LED strip
    private void setRainbowAnimation() {
        Animation rainbow = new RainbowAnimation(1.0, 0.5, LED_COUNT);  // Create rainbow animation
        candle.animate(rainbow);  // Apply the rainbow animation to the LED strip
    }

    // Method to set a strobe animation with the given color and speed
    private void setStrobeAnimation(int r, int g, int b, double speed) {
        Animation strobe = new StrobeAnimation(r, g, b, 0, speed, LED_COUNT);  // Create strobe animation
        candle.animate(strobe);  // Apply the strobe animation to the LED strip
    }

    // Method to set a color flow animation with the given color and direction
    private void setColorFlowAnimation(int r, int g, int b, boolean reversed) {
        Animation flow = new ColorFlowAnimation(r, g, b, 0, 0.7, LED_COUNT, ColorFlowAnimation.Direction.Forward);  // Create color flow animation
        candle.animate(flow);  // Apply the color flow animation to the LED strip
    }

}