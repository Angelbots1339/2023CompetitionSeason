package frc.lib.util.multiplexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import static frc.robot.Constants.MultiplexerConstants.*;

/**
 * The TCA9548A multiplexer is interesting in that it has an I2C address (0x70
 * by default) - and you basically send it a command to tell it which I2C
 * multiplexed output you want to talk to, then you can address the board you
 * want to address.
 * 
 * https://www.ti.com/lit/ds/symlink/tca9548a.pdf?ts=1643751985463&ref_url=https%253A%252F%252Fwww.google.com%252F
 */
public class Multiplexer extends I2C {
    private static Multiplexer MULTIPLEXER;
    private static int currentDevice = 0;

    private Multiplexer() {
        super(DEFAULT_PORT, DEFAULT_ADDRESS);
    }

    /**
     * Configures the multiplexer to pass through the selected device, if it is not
     * already selected
     * 
     * @param port Port on multiplexer that the 12c bus is switched to
     * @return Transfer success.
     * @throws MultiplexerSwitchException
     */
    public boolean setDevice(int port) {
        assert port <= 7 && port >= 0;
        boolean success = !getInstance().write(0x00, 1 << port);
        if (!success) {
            DriverStation.reportError(String.format("Error switching to channel " + port +
                    " on Multiplexer. Please check your connections",
                    DEFAULT_PORT == Port.kMXP ? "MXP" : "Onboard"), false);
        }
        currentDevice = port;
        return success;
    }

    /**
     * @return Current device address, or 0 if none are selected
     */
    public static int getCurrentDevice() {
        return currentDevice;
    }

    public static Multiplexer getInstance() {
        if (MULTIPLEXER == null) {
            MULTIPLEXER = new Multiplexer();
        }
        return MULTIPLEXER;
    }
}
