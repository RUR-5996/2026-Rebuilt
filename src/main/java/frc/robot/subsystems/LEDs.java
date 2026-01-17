package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
    Spark blinkin = new Spark(0);
    private double color = Constants.ColorConstants.PINK;
    private double[] color_list = {Constants.ColorConstants.PINK, Constants.ColorConstants.VIOLET, Constants.ColorConstants.FLASHBANG, Constants.ColorConstants.RAINBOW, Constants.ColorConstants.BLUEGREEN, Constants.ColorConstants.DARKBLUE};
    private double[] strobe_list = {Constants.ColorConstants.STROBEBLUE, Constants.ColorConstants.STROBEGOLD, Constants.ColorConstants.STROBERED, Constants.ColorConstants.STROBEWHITE};
    private int current_color = 0;

    private static LEDs LEDS;
    
    public LEDs() {

    }

    @Override
    public void periodic() {
        blinkin.set(color);
    }

    public static LEDs getInstance() {
        if(LEDS == null) {
            LEDS = new LEDs();
        }
        return LEDS;
    }

    public Command setColor(double constant) {
        return Commands.runOnce(
            () -> {
                color = constant;
            }
        );
    }
    public Command changeColor() {
        return Commands.runOnce(
            () -> {
                current_color += 1;
                if (current_color == color_list.length) {
                    current_color = 0;
                }
                System.out.println(current_color);
                color = color_list[current_color];
            }
        );
    }
    public Command indicator(int index) {
        return Commands.sequence(
        Commands.runOnce(() -> color = strobe_list[index]),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> color = 0.99)
    );
    }
};