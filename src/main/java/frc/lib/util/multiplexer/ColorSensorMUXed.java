package frc.lib.util.multiplexer;

import static frc.robot.Constants.MultiplexerConstants.*;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Wrapper class for ColorSensorV3 to use {@link Multiplexer}
 */
public class ColorSensorMUXed {
    private final int port;
    private ColorSensorV3 sensor;
    private boolean enabled;

    public ColorSensorMUXed(int port) {

        this.port = port;
        if (Multiplexer.getInstance().setDevice(port)) {
            sensor = new ColorSensorV3(DEFAULT_PORT);
            if (sensor.isConnected()) {
                enabled = true;
            }
        }

    }

    public ColorSensorV3 get() {
        if (Multiplexer.getInstance().setDevice(port) && enabled)
            return sensor;
        return null;
    }

    public boolean getEnabled() {
        return enabled;
    }

    public void reset() {
        enabled = false;
        if (Multiplexer.getInstance().setDevice(port)) {
            sensor = new ColorSensorV3(DEFAULT_PORT);
            if (sensor.isConnected()) {
                enabled = true;
            }
        }
    }

    public static double xyzColorDifference(Color color1, Color color2){
        return Math.sqrt(Math.pow(color1.red - color2.red, 2) + Math.pow(color1.green - color2.green, 2) + Math.pow(color1.blue - color2.blue, 20));     
    }
}
