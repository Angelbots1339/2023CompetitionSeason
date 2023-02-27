package frc.robot;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;
import frc.lib.util.ElevatorWristState;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double angularStickDeadband = 0.1;

    public static final String CANIVORE = "canivore";
    public static final int MAX_VOLTAGE = 12;
    public static final int CAN_TIMEOUT = 100; // ms

    public static final class SwerveConstants {

        public static final double LOOPER_DT = 0.02; // used for 254's solution to swerve skew it is loop time in sec
        public static final double FUDGE_FACTOR_KP = 0.1; // used for the CD fudge factor solution to swerve skew
        public static final double FUDGE_FACTOR_SIMPLE_KP = 0.1; // used for the CD fudge factor solution to swerve skew

        public static final int PIGEON_ID = 21;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-
        public static final int YAW_BUFFER_PERIOD = 5; // 5ms
        public static final int YAW_BUFFER_SIZE = 20; // amount of values to store in buffer

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(19.75);
        public static final double WHEEL_CIRCUMFERENCE = 0.0980808154 * Math.PI;
        public static final double ALIGN_OFFSET= Units.inchesToMeters(15.75); //distance from center of robot to edge of bumper track width + bumper width



        /* Module Gear Ratios */
        /** SDS MK4i l2 - 6.75 : 1 */
        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);
        /** SDS MK4i l1 - (150 / 7) : 1 */
        public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         * max speed:
         * 6380/8.14 * 0.1 * pi / 60 = 4.103882295
         * 12 / 4.103882295 = 2.920
         */
        ////FIXL2
        public static final double DRIVE_KS = (0.23153 / 12);
        public static final double DRIVE_KV = (2.3061 / 12);
        public static final double DRIVE_KA = (0.27485 / 12);



        /* Heading deadBand */
        public static final double HEADING_DEADBAND = 0.3;

        /* Swerve Profiling Values */
        /** Meters per Second */
        private static final double TRUE_MAX_SPEED = 6380 / 60 / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE; // 1.5;
        private static final double KV_MAXSPEED = 1 / DRIVE_KV;
        public static final double MAX_SPEED = KV_MAXSPEED;// 4;
        /** Radians per Second */
        private static final double TRUE_MAX_ANGULAR = MAX_SPEED / 0.7094402336;
        public static final double MAX_ANGULAR_VELOCITY = TRUE_MAX_ANGULAR; // 5.0;
        /** Meters per Second */
        public static final double MIN_CLOSE_LOOP_SPEED = 0.2;

        public static final class FalconConfigConstants {

            /* Neutral Modes */
            public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Brake;
            public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

            /* Sensor Initialization strategy */
            public static final SensorInitializationStrategy ANGLE_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToZero;
            public static final SensorInitializationStrategy DRIVE_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToZero;

            /* drive motor velocity sensor period*/
            public static final SensorVelocityMeasPeriod DRIVE_SENSOR_VELOCITY_MEAS_PERIOD = SensorVelocityMeasPeriod.Period_5Ms;
            public static final int DRIVE_SENSOR_VELOCITY_MEAS_WINDOW = 32;

            /* Angle Motor PID Values */
            public static final double ANGLE_KP = 0.3;
            public static final double ANGLE_KI = 0.0;
            public static final double ANGLE_KD = 0.0;
            public static final double ANGLE_KF = 0.0;

            /* Drive Motor PID Values */
            public static final double DRIVE_KP = 0.1; 
            public static final double DRIVE_KD = 0.0;
            public static final double DRIVE_KF = 0.0;
            /*
             * These values are used by the drive falcon to ramp in open loop and closed
             * loop driving.
             * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
             */
            public static final double OPEN_LOOP_RAMP = 0.25;
            public static final double CLOSED_LOOP_RAMP = 0.0;

            /* Motor Inverts */
            public static final TalonFXInvertType ANGLE_MOTOR_INVERT = TalonFXInvertType.Clockwise;
            public static final TalonFXInvertType DRIVE_MOTOR_INVERT = TalonFXInvertType.Clockwise;

            /* Swerve Current Limiting */
            private static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
            private static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
            private static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
            private static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
            public static final SupplyCurrentLimitConfiguration ANGLE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                    ANGLE_ENABLE_CURRENT_LIMIT, ANGLE_CONTINUOUS_CURRENT_LIMIT, ANGLE_PEAK_CURRENT_LIMIT,
                    ANGLE_PEAK_CURRENT_DURATION);

            private static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
            private static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
            private static final double DRIVE_PEAL_CURRENT_DURATION = 0.1;
            private static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
            public static final SupplyCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                    DRIVE_ENABLE_CURRENT_LIMIT, DRIVE_CONTINUOUS_CURRENT_LIMIT, DRIVE_PEAK_CURRENT_LIMIT,
                    DRIVE_PEAL_CURRENT_DURATION);

            /*------- CANcoder Config ------- */
            /* Angle Encoder Invert */
            public static final boolean CANCODER_INVERT = false;

            public static final AbsoluteSensorRange CANCODER_ABSOLUTE_SENSOR_RANGE = AbsoluteSensorRange.Unsigned_0_to_360;
            public static final SensorInitializationStrategy CANCODER_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToAbsolutePosition;
            public static final SensorTimeBase CANCODER_SENSOR_TIME_BASE = SensorTimeBase.PerSecond;
        }

        /* Module Specific Constants */
        /** Back Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(280.37109375);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Front Left Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int CANCODER_ID = 2;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(223.330078125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        
        /** Back Right Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int CANCODER_ID = 10;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(227.900390625);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
            ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /** front Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 13;
            public static final int ANGLE_MOTOR_ID = 14;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(197.138671875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        public static final class DrivePidConstants {

            public static final double ANGLE_KP = 0.025;  //0.05// radians per sec per degrees
            public static final double ANGLE_KD = 0.0;
            public static final double ANGLE_KV = 0.0;

            public static final double TRANSLATION_KP = 0.5; // meters per sec per meter
            public static final double TRANSLATION_KD = 0;
            public static final double TRANSLATION_KV = 0.0;

            // Feedfowards
            public static final double ANGLE_KS = 0.7; // radians per sec
            public static final double ANGLE_TOLERANCE = 10; // Degrees
        
            public static final double TRANSLATION_PID_TOLERANCE = 0;

        }
    }

    public static final class AutoConstants {
        public static final double X_KP = 1;
        public static final double Y_KP = 1;
        public static final double THETA_KP = 3.2;
    }

    public static final class ElevatorConstants {
        // Calculated maxs
        private static final double MAX_ELEVATOR_VELOCITY = 14.905; //Clicks per sec 

        public static final int FOLLOWER_MOTOR_ID = 7;// TODO right motor
        public static final int LEADER_MOTOR_ID = 8;// TODO

        public static double SPOOL_DIAMETER = Units.inchesToMeters(1.7); // Meters

        public static final double GEAR_RATIO = (5 / 1.0);

        public static final double ANGLE = 4096;


        public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 35, 60, 0.1);

         public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                    false, 1000, 1000, 1);

        public static final TalonFXInvertType MOTOR_INVERTED = TalonFXInvertType.CounterClockwise;

        public static final int SENSOR_VELOCITY_MEAS_WINDOW = 8;

        public static final int FORWARD_SOFT_LIMIT = 50276; // TODO
        public static final int REVERSE_SOFT_LIMIT = 0; // TODO

        public static final double KP = 0.2; // 0.0731;
        public static final double KD = 0;

        // Resonable value 0.1079
        //  Feed-forward gain so that 20% motor output is calculated when the requested speed is 2981 native units per 100ms.
        // F-gain = (0.2 X 1023) / 2981 F-gain = 0.06863
        public static final double KF =0.06863; //cP100msToMPS(4.16 * 1023 / 12);

        public static final double KS = 0.08; // test this is calculated from recalc

        public static final double MAX_VELOCITY = 3100 * 6; // TODO
        public static final double MAX_ACCELERATION = 3100 *4; // TODO

        public static final int S_CURVE_STRENGTH = 4; 

        

        public static final double MOTION_MAGIC_ERROR_THRESHOLD = 0.02; // Meters
        public static final double TIME_TO_SETTLE = 0.1;//seconds 

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;


        /**
         * Converts the extension of the elevator relative to the home position in
         * meters
         * 
         * @param meters
         * @return
         */
        public static int metersToClicks(double meters) {
            /*
             *  1m | 1Rm| 1 Rot                       | 5 FRo | 2048 Clicks |
             * ----|----|-----------------------------|-------|-------------|
             *    | 2m | Math.PI * SPOOL_DIAMETER Rm | 1 Rot | 1 FRot |
             * 
             */
            double rotations = meters / 2 / (Math.PI * SPOOL_DIAMETER);
            return Conversions.RotationsToFalcons(rotations, GEAR_RATIO);
        }

        public static double clicksToMeters(double ticks) {
            double rotations = Conversions.falconToRotations(ticks, GEAR_RATIO);
            return rotations * 2 * Math.PI * SPOOL_DIAMETER;
        }

        /**
         * Converts the extension of the elevator relative to the home position in
         * meters
         * 
         * @param meters
         * @return
         */
        public static int mPSToCP100ms(double MetersPerSec) {
            return metersToClicks(MetersPerSec) / 10;// to convert to 1s loops
        }

        public static double cP100msToMPS(double CountsPerSec) {
            return clicksToMeters(CountsPerSec * 10);// to convert to 100ms loops
        }
    }


    public static final class WristConstants{
        public static final int MOTOR_ID = 16;

        public static final double WRIST_HOME_ANGLE = 10;//TODO

        public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 35, 60, 0.1);

        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(
                false, 70, 90, 0.1);

        public static final TalonFXInvertType MOTOR_INVERTED = TalonFXInvertType.CounterClockwise;
    
        public static final int SENSOR_VELOCITY_MEAS_WINDOW = 8;

        public static final int CLICKS_AT_HORIZONTAL = 18159; 
    
        public static final int FORWARD_SOFT_LIMIT = radiansToClicks(Math.toRadians(204.462891)); 
        public static final int REVERSE_SOFT_LIMIT = radiansToClicks(Math.toRadians(13));

        public static final double MAX_PLANETARY = (5 / 1.0);
        public static final double GEARS = (60 / 30);
        public static final double CHAIN_SPROCKET = (48 / 16);

        //angle when intake is against wrist 
        public static final double angleOffset = 100;
        public static final double TimeBeforeReset = 1.5;//sec b/c through bore initializes with 0;

       // public static final double KP = 0.01; // 0.0731;
        public static final double KP = 0.2; // 0.0731;
        public static final double KD = 0;
        // Resonable value 0.1079
        //  Feed-forward gain so that 75% motor output is calculated when the requested speed is 7112 native units per 100ms.
        // F-gain = (0.1 X 1023) / 1718 F-gain = 0.1079
        public static final double KF = 0.1131;
        //at horizontal
        public static final double KS =  -0.1; //TODO
        
        public static final double MAX_VELOCITY =  27488.0; 
        public static final double MAX_ACCELERATION = 13744.0; 

        public static final int S_CURVE_STRENGTH = 4; // TODO
        public static final double MOTION_MAGIC_ERROR_THRESHOLD = 3; // Degrees
        public static final double TIME_TO_SETTLE = 0.1;//seconds 

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;

        public static int radiansToClicks(double radians) {
            double rotations = radians / (2 * Math.PI);
            return Conversions.RotationsToFalcons(rotations, MAX_PLANETARY * CHAIN_SPROCKET * GEARS);
        }
        public static double clicksToRadians(double clicks) {
            double rotations = Conversions.falconToRotations(clicks, MAX_PLANETARY * CHAIN_SPROCKET * GEARS);
            return rotations * 2 * Math.PI;
        }
        public static int radiansPerSecToClicksPer100ms(double radiansPerSec) {
            return radiansToClicks(radiansPerSec) / 10;// to convert from 1s loops
        }
        public static double clicksPer100msToRadiansPerSec(double clicksPer100ms) {
            return clicksToRadians(clicksPer100ms * 10);// to convert from 100ms loops
        }
        public static Rotation2d throughBoreToAngle(double rots) {
            double endRots = rots / CHAIN_SPROCKET;
            return Rotation2d.fromRotations(endRots);
        }

    }
    
    public static final class ElevatorWristStateConstants{
        public static final double IGNORE_CRUSH_ANGLE_HEIGHT = 0.4; //Height at witch crush angle protection is ignored
        public static final Rotation2d CRUSH_ANGLE = Rotation2d.fromDegrees(130); // angle at which the intake would be crushed if the elevator was lowered
        public static final Rotation2d SAFE_LIMELIGHT_ANGLE = Rotation2d.fromDegrees(16);//angle at which the lime light can not be hit
        public static final double LIMELIGHT_CRUSH_HEIGHT_LOW = 0.35;//height at witch the limelight could be hit that assure the intake is at safe angle before passing
        public static final double LIMELIGHT_CRUSH_HEIGHT_HIGH = 0.5;//height at witch the limelight could be hit that assure the intake is at safe angle before passing

        public static final double LimeLightMidPosition = 0;
        public static final Rotation2d LimeLightMidAngle = Rotation2d.fromDegrees(0);

        public static final ElevatorWristState HOME = new ElevatorWristState(13, 0);
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 17;// TODO
        public static final int SHOOT_MOTOR_ID = 15;// TODO
        public static final int COLOR_SENSOR_PORT = 2;
        public static final int RIGHT_DIST_SENSOR_PORT = 2;
        public static final int LEFT_DIST_SENSOR_PORT = 2;

        public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                true, 35, 60, 0.1);

        public static final TalonFXInvertType INTAKE_MOTOR_INVERTED = TalonFXInvertType.CounterClockwise;
        public static final TalonFXInvertType SHOOT_MOTOR_INVERTED = TalonFXInvertType.CounterClockwise;


        public static final RangeProfile DISTANCE_SENSOR_PROFILE = RangeProfile.kHighAccuracy;

        public static final int DISTANCE_SENSOR_EMPTY_THRESHOLD = 0; //TODO cm

        public static final Color CUBE_COLOR = new Color(60, 104, 89);
        public static final Color CONE_COLOR = new Color(88, 126, 39); 

    }




    public static final class VisionConstants {
        public static final Transform3d APRILTAG_CAM_POS = new Transform3d(new Translation3d(-0.24345, 0.25397, 0.56859),
                new Rotation3d(0, -0, 0)); // TODO OPI pos
        public static final PhotonCamera APRILTAG_CAM = new PhotonCamera("Cam1");



        public static final PoseStrategy APRILTAG_POSE_STRATEGY = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
        /*
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ, with
         * units in
         * meters and radians.
         */
        public static final Matrix<N3, N1> STATE_STD_DEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                0.10,
                0.10,
                0.10);


        public static final double MID_LIMELIGHT_OFFSET = Units.inchesToMeters(36.0);//used to calculate cone offset 
        public static final double HIGH_LIMELIGHT_OFFSET = 1.36633;//used to calculate cone offset

        public static final int FAVOR_HIGH_HIGH_PIPELINE = 0;
        public static final int FAVOR_HIGH_MID_PIPELINE = 1;

        public static final int FAVOR_MID_HIGH_PIPELINE = 2;
        public static final int FAVOR_MID_MID_PIPELINE = 3;
        


        public static final Transform3d LIMELIGHT_CAM_POS = new Transform3d(new Translation3d(-0.04, -0.25, 0.85),
                new Rotation3d(0, Math.toRadians(0), 0));

        public static final double LIMELIGHT_ALIGN_MAX_SPEED = 2; //m/s

    }

    public final static class MultiplexerConstants {
        public static final byte DEFAULT_ADDRESS = 0x70;
        public static final Port DEFAULT_PORT = Port.kMXP;
    }

    public final static class FieldConstants {
        public static final Translation2d RED_ORIGIN = new Translation2d(16.540988, 1.071626);
    }
    

    public final static class CandleConstants {
        public static final int CANDLE_ID = 22;
        public static final int TOTAL_STRIP_LENGTH = 247; 
        public static final int OFFSET_LENGTH = 8; // LEDs to skip at the front of the strip, 8 means no lights on the CANDLE are on.

        public static final double HUMAN_PLAYER_COM_BLINK_RATE = 0.5;
    }

}
