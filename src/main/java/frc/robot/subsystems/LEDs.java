package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    Spark blinkin = new Spark(0);
    private double colour = Constants.colourConstants.PINK;
    private double[] colour_list = {Constants.colourConstants.PINK, Constants.colourConstants.VIOLET, Constants.colourConstants.FLASHBANG, Constants.colourConstants.RAINBOW, Constants.colourConstants.BLUEGREEN, Constants.colourConstants.DARKBLUE};
    private double[] strobe_list = {Constants.colourConstants.STROBEBLUE, Constants.colourConstants.STROBEGOLD, Constants.colourConstants.STROBERED, Constants.colourConstants.STROBEWHITE};
    private int current_colour = 0;

    private static LEDs LEDS;
    
    public LEDs() {

    }

    @Override
    public void periodic() {
        blinkin.set(colour);
    }

    public static LEDs getInstance() {
        if(LEDS == null) {
            LEDS = new LEDs();
        }
        return LEDS;
    }

    public Command setColour(double constant) {
        return Commands.runOnce(
            () -> {
                colour = constant;
            }
        );
    }
    public Command changeColour() {
        return Commands.runOnce(
            () -> {
                current_colour += 1;
                if (current_colour == colour_list.length) {
                    current_colour = 0;
                }
                System.out.println(current_colour);
                colour = colour_list[current_colour];
            }
        );
    }
    public Command indicator(int index) {
        return Commands.sequence(
        Commands.runOnce(() -> colour = strobe_list[index]),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> colour = 0.99)
    );
    }
};