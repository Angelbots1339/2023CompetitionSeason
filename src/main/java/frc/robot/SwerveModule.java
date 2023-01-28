package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import static frc.robot.Constants.SwerveConstants.FalconConfigConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int moduleNumber;
    public Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.DRIVE_KS,
            Constants.SwerveConstants.DRIVE_KV, Constants.SwerveConstants.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, Constants.CANIVORE);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = TalonFXFactory.createDefaultTalon(moduleConstants.angleMotorID, Constants.CANIVORE);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = TalonFXFactory.createDefaultTalon(moduleConstants.driveMotorID, Constants.CANIVORE);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.MAX_SPEED;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.WHEEL_CIRCUMFERENCE, Constants.SwerveConstants.DRIVE_GEAR_RATIO);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.MAX_SPEED * 0.01))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        angleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), Constants.SwerveConstants.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.ANGLE_GEAR_RATIO));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public TalonFX getAngleMotor() {
        return angleMotor;
    }

    public TalonFX getDriveMotor() {
        return angleMotor;
    }

    public double getEncoderInMeters() {
        return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.WHEEL_CIRCUMFERENCE,
                Constants.SwerveConstants.DRIVE_GEAR_RATIO);
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
                angleEncoder.getAbsolutePosition() - angleOffset.getDegrees(),
                Constants.SwerveConstants.ANGLE_GEAR_RATIO);
        angleMotor.setSelectedSensorPosition(absolutePosition);

    }

    private void configAngleEncoder() {
        TalonUtil.checkError(angleEncoder.configFactoryDefault(Constants.CAN_TIMEOUT),
                "failed to config factory default CANCoder mod: " + moduleNumber);
        TalonUtil.checkError(
                angleEncoder.configSensorInitializationStrategy(ANGLE_SENSOR_INIT_STRATEGY, Constants.CAN_TIMEOUT),
                "failed to config sensor init strategy CANCoder mod: " + moduleNumber);
        TalonUtil.checkError(
                angleEncoder.configAbsoluteSensorRange(CANCODER_ABSOLUTE_SENSOR_RANGE, Constants.CAN_TIMEOUT),
                "failed to config absolute sensor range CANCoder mod: " + moduleNumber);
        TalonUtil.checkError(angleEncoder.configFeedbackCoefficient(0.087890625, "deg", CANCODER_SENSOR_TIME_BASE,
                Constants.CAN_TIMEOUT), "failed to config feedback coefficient CANCoder mod: " + moduleNumber);

    }

    private void configAngleMotor() {

        TalonUtil.checkError(angleMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
                "failed to config voltage comp angle motor mod: " + moduleNumber);
        TalonUtil.checkError(angleMotor.configSupplyCurrentLimit(ANGLE_CURRENT_LIMIT, Constants.CAN_TIMEOUT),
                "failed to config current limit angle motor mod: " + moduleNumber);
        TalonUtil.checkError(angleMotor.config_kP(0, ANGLE_KP, Constants.CAN_TIMEOUT),
                "failed to config kP angle motor mod: "
                        + moduleNumber);
        TalonUtil.checkError(angleMotor.config_kI(0, ANGLE_KI, Constants.CAN_TIMEOUT),
                "failed to config kI angle motor mod: "
                        + moduleNumber);
        TalonUtil.checkError(angleMotor.config_kD(0, ANGLE_KD, Constants.CAN_TIMEOUT),
                "failed to config kD angle motor mod: "
                        + moduleNumber);
        TalonUtil.checkError(angleMotor.config_kF(0, ANGLE_KF, Constants.CAN_TIMEOUT),
                "failed to config kF angle motor mod: " + moduleNumber);
        TalonUtil.checkError(
                angleMotor.configIntegratedSensorInitializationStrategy(ANGLE_SENSOR_INIT_STRATEGY,
                        Constants.CAN_TIMEOUT),
                "failed to config sensor init strategy angle motor mod: " + moduleNumber);

        angleMotor.selectProfileSlot(0, 0);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.setInverted(ANGLE_MOTOR_INVERT);
        angleMotor.setNeutralMode(ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        TalonUtil.checkError(driveMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
                "failed to config voltage comp drive motor mod: " + moduleNumber);
        TalonUtil.checkError(driveMotor.configSupplyCurrentLimit(DRIVE_CURRENT_LIMIT, Constants.CAN_TIMEOUT),
                "failed to config current limit drive motor mod: " + moduleNumber);
        TalonUtil.checkError(driveMotor.config_kP(0, DRIVE_KP, Constants.CAN_TIMEOUT),
                "failed to config kP drive motor mod: " + moduleNumber);
        TalonUtil.checkError(driveMotor.config_kI(0, DRIVE_KI, Constants.CAN_TIMEOUT),
                "failed to config kI drive motor mod: " + moduleNumber);
        TalonUtil.checkError(driveMotor.config_kD(0, DRIVE_KD, Constants.CAN_TIMEOUT),
                "failed to config kD drive motor mod: " + moduleNumber);
        TalonUtil.checkError(driveMotor.config_kF(0, DRIVE_KF, Constants.CAN_TIMEOUT),
                "failed to config kF drive motor mod: " + moduleNumber);
        TalonUtil.checkError(
                driveMotor.configIntegratedSensorInitializationStrategy(DRIVE_SENSOR_INIT_STRATEGY,
                        Constants.CAN_TIMEOUT),
                "failed to config sensor init strategy drive motor mod: " + moduleNumber);
        TalonUtil.checkError(driveMotor.configOpenloopRamp(OPEN_LOOP_RAMP),
                "failed to config open loop ramp drive motor mod: " + moduleNumber);
        TalonUtil.checkError(driveMotor.configClosedloopRamp(CLOSED_LOOP_RAMP),
                "failed to config closed loop ramp drive motor mod: " + moduleNumber);

        driveMotor.selectProfileSlot(0, 0);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setInverted(DRIVE_MOTOR_INVERT);
        driveMotor.setNeutralMode(DRIVE_NEUTRAL_MODE);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                // FIXME check conversion
                Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                        Constants.SwerveConstants.WHEEL_CIRCUMFERENCE,
                        Constants.SwerveConstants.DRIVE_GEAR_RATIO),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.WHEEL_CIRCUMFERENCE, Constants.SwerveConstants.DRIVE_GEAR_RATIO), getAngle());
    }

    public double getRotations() {
        return Conversions.falconToRotaiton(driveMotor.getSelectedSensorPosition(),
                Constants.SwerveConstants.DRIVE_GEAR_RATIO);
    }
}