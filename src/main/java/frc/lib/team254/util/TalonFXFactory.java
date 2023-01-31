package frc.lib.team254.util;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    private final static int kTimeoutMs = 100;

    public static class Configuration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        // factory default
        public double NEUTRAL_DEADBAND = 0.04;

        public SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY = SensorInitializationStrategy.BootToZero;
        public double SENSOR_OFFSET_DEGREES = 0;

        public boolean ENABLE_SUPPLY_CURRENT_LIMIT = false;
        public boolean ENABLE_STATOR_CURRENT_LIMIT = false;

        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;


       

        /*
         * Status 1 (Default Period 10ms):
         - Applied Motor Output
         - Fault Information
         - Limit Switch Information
         */
        public int GENERAL_STATUS_FRAME_RATE_MS = 10;

        /*
         * Status 2 (Default Period 20ms):
         - Selected Sensor Position (PID 0)
         - Selected Sensor Velocity (PID 0)
         - Brushed Supply Current Measurement
         - Sticky Fault Information
         */
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        
        /*
         * Status 3 (Default Period >100ms):
         - Quadrature Information
         */
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        
        /*
         * Status 4 (Default Period >100ms):
         - Analog Input
         - Supply Battery Voltage
         - Controller Temperature
         */
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;

        /*Status 8 (Default Period >100ms):
        - Pulse Width Information 
        */
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        public int CONTROL_FRAME_PERIOD_MS = 10;

        public int MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        
        

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kFollowerConfiguration = new Configuration();

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically. Potentially try to increase as much as possible.
        kFollowerConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
        kFollowerConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kFollowerConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kFollowerConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kFollowerConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kFollowerConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kFollowerConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        kFollowerConfiguration.ENABLE_SOFT_LIMIT = false;
    }

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(int id, String canBus) {
        return createTalon(id, canBus, kDefaultConfiguration);
    }

    public static TalonFX createPermanentFollowerTalon(int follower_id, int leader_id, String canBus) {
        final TalonFX talon = createTalon(follower_id, canBus, kFollowerConfiguration);
        talon.set(ControlMode.Follower, leader_id);
        return talon;
    }

    public static TalonFX createTalon(int id, String canBus, Configuration config) {
        TalonFX talon = new LazyTalonFX(id, canBus);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kTimeoutMs);
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kTimeoutMs);


        talon.configNominalOutputForward(0, kTimeoutMs);
        talon.configNominalOutputReverse(0, kTimeoutMs);
        talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);

        talon.configMotorCommutation(MotorCommutation.Trapezoidal, kTimeoutMs);


        talon.configPeakOutputForward(1.0, kTimeoutMs);
        talon.configPeakOutputReverse(-1.0, kTimeoutMs);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, kTimeoutMs);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);

        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, kTimeoutMs);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
                kTimeoutMs);

        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs);

        talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        talon.configVoltageMeasurementFilter(32, kTimeoutMs);
        talon.enableVoltageCompensation(false);

        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(config.ENABLE_SUPPLY_CURRENT_LIMIT, 20, 60, .2), kTimeoutMs);
        talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(config.ENABLE_STATOR_CURRENT_LIMIT, 20, 60, .2), kTimeoutMs);

        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);
        talon.configIntegratedSensorInitializationStrategy(config.SENSOR_INITIALIZATION_STRATEGY, kTimeoutMs);
        talon.configIntegratedSensorOffset(config.SENSOR_OFFSET_DEGREES, kTimeoutMs);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return talon;
    }
}