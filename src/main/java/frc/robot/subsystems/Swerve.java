package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import frc.robot.Constants.ElevatorWristStateConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.auto.SwerveFollowTrajectory;
import frc.robot.vision.PoseEstimator;
import frc.robot.vision.RetroReflectiveTargeter;
import frc.lib.math.ClosedLoopUtil;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.Candle;
import frc.lib.util.FieldUtil;
import frc.lib.util.GyroLatencyBuffer;
import frc.lib.util.LatencyDoubleBuffer;
import frc.lib.util.LimeLight;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.logging.loggedObjects.LoggedPigeon2;
import frc.lib.util.logging.loggedObjects.LoggedSwerveModule;
import frc.robot.FieldDependentConstants;
import frc.robot.LoggingConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.SwerveConstants.DrivePidConstants.*;

import java.util.Optional;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] swerveMods;
    private final Pigeon2 gyro;
    private final LoggedSubsystem logger;

    private final PoseEstimator poseEstimation;
    private final LoggedField field;
    private final LoggedField autoField;
    private Translation2d autoErrorTranslation = new Translation2d();
    private Rotation2d autoErrorRotation = new Rotation2d();
    public GyroLatencyBuffer gyroBuffer;

    public Swerve() {
        configPIDtoPoseControllers();
        configPidAngularDrive();

        swerveMods = new SwerveModule[] {
                new SwerveModule(0, Mod0.constants),
                new SwerveModule(1, Mod1.constants),
                new SwerveModule(2, Mod2.constants),
                new SwerveModule(3, Mod3.constants)
        };

        gyro = new Pigeon2(PIGEON_ID);
        TalonUtil.checkError(gyro.configFactoryDefault(), "Failed to config factory default on pigeon");
        zeroGyro();

        gyroBuffer = new GyroLatencyBuffer(gyro, GYRO_BUFFER_SIZE, GYRO_BUFFER_PERIOD);
        gyroBuffer.start();

        // logging
        logger = new LoggedSubsystem("Swerve", LoggingConstants.SWERVE);

        field = new LoggedField("PoseEstimator", logger, "PoseEstimator", true);
        autoField = new LoggedField("AutoField", logger, "Auto", true);
        poseEstimation = new PoseEstimator(field, getYaw(), getPositions());

        SwerveFollowTrajectory.setLoggingCallbacks((PathPlannerTrajectory traj) -> {
            autoField.setTrajectory("AutoPath", traj, true);
        }, (Pose2d pose2d) -> {
            autoField.addPose2d("DesiredPose", () -> pose2d, true);
        }, null, (Translation2d translation2d, Rotation2d rotation2d) -> {
            autoErrorTranslation = translation2d;
            autoErrorRotation = rotation2d;
        });

        initializeLog();
    }

    /**
     * 
     * @param translation   Translation2d holding the desired velocities on each
     *                      axis
     * @param rotation      Desired rotational velocity
     * @param fieldRelative If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        Rotation2d adjustedYaw = DriverStation.getAlliance() == Alliance.Blue ? getYaw()
                : getYaw().plus(new Rotation2d(Math.PI));
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    adjustedYaw);
        } else {
            chassisSpeeds = new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation);
        }
        chassisSpeeds = reduceSkewFromLogTwist2d(chassisSpeeds);
        logger.updateDouble("x", chassisSpeeds.vxMetersPerSecond, "Drive");
        logger.updateDouble("y", chassisSpeeds.vyMetersPerSecond, "Drive");
        logger.updateDouble("rot", chassisSpeeds.omegaRadiansPerSecond, "Drive");

        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public void disable() {
        setModuleStates(KINEMATICS.toSwerveModuleStates(
                new ChassisSpeeds(0, 0, 0)),
                true);

    }

    public void xPos() {
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredStateStrict(states[mod.moduleNumber], false);
        }

    }

    private PIDController angularDrivePID = new PIDController(ANGLE_KP,
            0, ANGLE_KD);

    private void configPidAngularDrive() {
        angularDrivePID.enableContinuousInput(0, 360);
        angularDrivePID.setTolerance(ANGLE_TOLERANCE);
    }

    /**
     * @param translation    Translation2d holding the desired velocities on each
     *                       axis
     * @param desiredDegrees The desired orientation of the robot, whith 0 being
     *                       straight forward
     * @param fieldRelative  If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void angularDrive(Translation2d translation, Rotation2d desiredDegrees, boolean fieldRelative,
            boolean isOpenLoop) {

        Rotation2d desiredAngularVelocity = Rotation2d.fromDegrees(
                (lastDesiredDegrees - desiredDegrees.getDegrees()) / (Timer.getFPGATimestamp() - lastTimeDrive));

        lastDesiredDegrees = desiredDegrees.getDegrees();
        lastTimeDrive = Timer.getFPGATimestamp();

        double rotation = 0;
        // convert yaw into -180 -> 180 and absolute
        double yaw = (0 > getYaw().getDegrees() ? getYaw().getDegrees() % 360 + 360 : getYaw().getDegrees() % 360);

        logger.updateDouble("desiredHeading", desiredDegrees.getDegrees(),
                "AngularDrive");
        logger.updateDouble("currentHeading", yaw, "AngularDrive");
        double pid = angularDrivePID.calculate(yaw, desiredDegrees.getDegrees());

        if (Math.abs(angularDrivePID.getPositionError()) >= ANGLE_TOLERANCE) {
            rotation = MathUtil.clamp(pid + (-desiredAngularVelocity.getRadians() * ANGLE_KV) + // Kv
                                                                                                // //
                                                                                                // Velocity
            // Feedfoward
                    ((Math.signum(angularDrivePID.getPositionError())
                            * ANGLE_KS)) // Ks Static Friction Feedforward
                    , -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        }

        logger.updateDouble("Output rotation", rotation, "AngularDrive");
        logger.updateBoolean("At setpoint", angularDrivePID.atSetpoint(),
                "AngularDrive");

        drive(translation, rotation, fieldRelative, isOpenLoop);
    }
    /*-----Possible Solutions for Swerve Drive Skew------ */

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5?u=ethan1}
     */
    public ChassisSpeeds reduceSkewFromChassisSpeeds254(ChassisSpeeds chassisSpeeds) {
        frc.lib.team254.geometry.Pose2d robot_pose_vel = new frc.lib.team254.geometry.Pose2d(
                chassisSpeeds.vxMetersPerSecond * LOOPER_DT,
                chassisSpeeds.vyMetersPerSecond * LOOPER_DT,
                frc.lib.team254.geometry.Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * LOOPER_DT));
        frc.lib.team254.geometry.Twist2d twist_vel = frc.lib.team254.geometry.Pose2d.log(robot_pose_vel);
        return new ChassisSpeeds(
                twist_vel.dx / LOOPER_DT, twist_vel.dy / LOOPER_DT, twist_vel.dtheta / LOOPER_DT);
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
     */

    private double previousDriveTime;

    public ChassisSpeeds reduceSkewFromLogTwist2d(ChassisSpeeds chassisSpeeds) {
        double timerDt = (Timer.getFPGATimestamp() - previousDriveTime);
        Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * timerDt,
                chassisSpeeds.vyMetersPerSecond * timerDt,
                Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * timerDt));
        Twist2d twist_vel = new Pose2d(0, 0, new Rotation2d(0)).log(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / timerDt, twist_vel.dy / timerDt, twist_vel.dtheta / timerDt);
        previousDriveTime = Timer.getFPGATimestamp();
        return updated_chassis_speeds;
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
     */
    public ChassisSpeeds reduceSkewFromChassisSpeedsFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);
        double fudgeFactor = chassisSpeeds.omegaRadiansPerSecond / linearVelocity * FUDGE_FACTOR_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthY = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthY;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         Takes the unit orthogonal vector and scales it by a constant times
     *         the angular velocity
     */
    public ChassisSpeeds reduceSkewFromChassisSpeedsSimpleFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);

        double fudgeFactor = Math.signum(chassisSpeeds.omegaRadiansPerSecond) * FUDGE_FACTOR_SIMPLE_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthY = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthY;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    /*-----Setters----- */

    /* Used by SwerveControllerCommand in Auto */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        // ChassisSpeeds adjustedChassisSpeeds =
        // reduceSkewFromLogTwist2d(chassisSpeeds);
        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, false);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, Boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void resetToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public AprilTagFieldLayout getField() {
        return poseEstimation.getField();
    }

    public Pose2d getClosestCubeNode() {
        return poseEstimation.getClosestCubeNode();
    }

    public void resetGyroTowardsDriverStation() {
        gyro.setYaw(FieldUtil.getTowardsDriverStation().getDegrees());
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimation.resetPose(pose.getRotation(), getPositions(), pose);
    }

    public void alignPoseNonVisionEstimator() {
        poseEstimation.alignPoseNonVisionEstimator(getPositions());
    }

    public void resetEncoders() {
        for (SwerveModule mod : swerveMods) {
            mod.resetDriveEncoder();
        }

    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void zeroGyro(double deg) {
        gyro.setYaw(deg);
    }

    /*-----Getters----- */

    public Pose2d getPoseOdometry() {
        return poseEstimation.getPoseNonVision();
    }

    public Pose2d getPose() {
        return poseEstimation.getPose();
    }

    public Pose2d getPoseTranslatedForAuto() {
        if (FieldUtil.isBlueAlliance()) {
            return getPose();
        } else {
            Pose2d pose = getPose();
            return new Pose2d(new Translation2d(FieldConstants.RED_ORIGIN.getX() - pose.getX(),
                    FieldConstants.RED_ORIGIN.getY() - pose.getY()), getAdjustedYaw());
        }
    }

    public Pose2d getPoseTranslatedForAutoOdometry() {
        if (FieldUtil.isBlueAlliance()) {
            return getPoseOdometry();
        } else {
            Pose2d pose = getPoseOdometry();
            return new Pose2d(new Translation2d(FieldConstants.RED_ORIGIN.getX() - pose.getX(),
                    FieldConstants.RED_ORIGIN.getY() - pose.getY()), getAdjustedYaw());
        }
    }
    

    


    private double lastTimePeriodic;
    private double lastTimeDrive;
    private double lastDesiredDegrees;
    private Translation2d currentVel = new Translation2d();
    private Pose2d lastPose = new Pose2d();

    /**
     * Get the current velocity of the robot
     * 
     * @return A Translation2d containing the current X and Y velocity
     */
    public Translation2d getCurrentVelocity() {
        return currentVel;
    }

    public void calculateVelocity() {
        double deltaTime = Timer.getFPGATimestamp() - lastTimePeriodic;
        currentVel = getPose().minus(lastPose).getTranslation().div(deltaTime);
        lastPose = getPose();
        lastTimePeriodic = Timer.getFPGATimestamp();
    }

    public double getCurrentTotalVelocity() {
        return Math.sqrt(Math.pow(currentVel.getX(), 2) + Math.pow(currentVel.getY(), 2));
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromRadians(Math.atan2(MathUtil.applyDeadband(currentVel.getY(), HEADING_DEADBAND),
                MathUtil.applyDeadband(currentVel.getX(), HEADING_DEADBAND)));
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getPosition();
        }
        return states;
    }

    public Rotation2d getYaw() {
        return (INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getYawAtTime(double time) {
        return (INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyroBuffer.getYawAtSeconds(time))
                : Rotation2d.fromDegrees(gyroBuffer.getYawAtSeconds(time));
    }

    public Rotation2d getAdjustedYaw() {
        return DriverStation.getAlliance() == Alliance.Blue ? getYaw()
                : getYaw().plus(new Rotation2d(Math.PI));
    }

    public Rotation3d getGyro() {
        return new Rotation3d(Math.toRadians(gyro.getRoll()), Math.toRadians(gyro.getPitch()),
                Math.toRadians(gyro.getYaw()));
    }

    public Rotation3d getGyroAtTime(double timeInSec) {
        return new Rotation3d(Math.toRadians(gyroBuffer.getYawAtSeconds(timeInSec)), 
                Math.toRadians(gyroBuffer.getPitchAtSeconds(timeInSec)), 
                Math.toRadians(gyroBuffer.getRollAtSeconds(timeInSec)));
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    // DRIVE PID
    private PIDController pidToPoseXController = new PIDController(TRANSLATION_KP, 0, 0);
    private PIDController pidToPoseYController = new PIDController(TRANSLATION_KP, 0, 0);

    private void configPIDtoPoseControllers() {
        pidToPoseXController.setTolerance(TRANSLATION_PID_TOLERANCE);
        pidToPoseYController.setTolerance(TRANSLATION_PID_TOLERANCE);
    }

    /**
     * Use this to have the robot move to a position. Ideally use when the robot
     * isn't very far away.
     * 
     * @param target The Pose for the robot to go to
     * @return Whether or not the robot has gotten to within tolerance on both
     *         translation and rotation
     */
    public boolean pidToPose(Pose2d target) {

        double X = pidToPoseXController.calculate(getPose().getTranslation().getX(), target.getX())
                + ClosedLoopUtil.positionFeedForward(pidToPoseXController.getPositionError(), TRANSLATION_KS);
        double Y = pidToPoseYController.calculate(getPose().getTranslation().getY(), target.getY())
                + ClosedLoopUtil.positionFeedForward(pidToPoseYController.getPositionError(), TRANSLATION_KS);

        Translation2d calculatedValues = new Translation2d(
                X,
                Y);

        angularDrive(calculatedValues, target.getRotation(), true, false);

        logger.updateDouble("out x", calculatedValues.getX(), "PidPose");

        return pidToPoseXController.atSetpoint() && pidToPoseYController.atSetpoint() && angularDrivePID.atSetpoint();
    }

    public double pidToX(double target) {
        double output = pidToPoseXController.calculate(getPose().getTranslation().getX(), target)
                + ClosedLoopUtil.positionFeedForward(pidToPoseXController.getPositionError(), SwerveConstants.DRIVE_KS);
        output = ClosedLoopUtil.stopAtSetPoint(output, pidToPoseXController.getPositionError(),
                TRANSLATION_PID_TOLERANCE);
        return ClosedLoopUtil.clampMaxEffort(output, MAX_SPEED);

    }

    public boolean pidToXAtSetPoint(double target) {
        return Math.abs(pidToPoseXController.getPositionError()) <= TRANSLATION_PID_TOLERANCE;
    }

    public double pidToY(double target) {
        double output = pidToPoseYController.calculate(getPose().getTranslation().getY(), target) + ClosedLoopUtil
                .positionFeedForward(pidToPoseYController.getPositionError(), SwerveConstants.DRIVE_KS);
        output = ClosedLoopUtil.stopAtSetPoint(output, pidToPoseYController.getPositionError(),
                TRANSLATION_PID_TOLERANCE);
        return ClosedLoopUtil.clampMaxEffort(output, MAX_SPEED);
    }

    public boolean pidToYAtSetPoint(double target) {
        return Math.abs(pidToPoseYController.getPositionError()) <= TRANSLATION_PID_TOLERANCE;
    }

    public void TestTrajectoryGeneration(PathPoint endpoint) {
        field.setTrajectory("AlignTraj", SwerveFollowTrajectory.SwerveGenerateTrajectoryToPoint(endpoint, this), true);
    }

    private String command = "None";

    


    public void initializeLog() {
        logger.add(field);
        logger.add(autoField);

        logger.add(new LoggedPigeon2("Gyro", logger, gyro, "Gyro"));

        logger.addDouble("Heading", () -> getHeading().getDegrees(), "Pose");
        logger.addDouble("x velocity", () -> getCurrentVelocity().getX(), "Pose");
        logger.addDouble("y velocity", () -> getCurrentVelocity().getY(), "Pose");

        logger.addString("Command", () -> {
            Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {command = c.getName();});
            return command;
          }, "Main");

        logger.addDouble("xAutoError", () -> autoErrorTranslation.getX(), "Auto");
        logger.addDouble("yAutoError", () -> autoErrorTranslation.getY(), "Auto");
        logger.addDouble("thetaAutoError", () -> autoErrorRotation.getDegrees(), "Auto");

        logger.add(new LoggedSwerveModule("Modules", logger, swerveMods, "Module", true));

        for (int i = 0; i < swerveMods.length; i++) {
            logger.add(new LoggedFalcon("Angle Motor: " + i, logger, swerveMods[i].getAngleMotor(), "Motor", true));
            logger.add(new LoggedFalcon("Drive Motor: " + i, logger, swerveMods[i].getDriveMotor(), "Motor", true));
        }
    }

    

   

    @Override
    public void periodic() {
        poseEstimation.updateOdometry(this, getPose());
        poseEstimation.justUpdate(this);
        RetroReflectiveTargeter.update(getPose(), true);


        SmartDashboard.putNumber("delayed yaw", gyroBuffer.getYaw(30 * 20));
        SmartDashboard.putNumber("yaw", getYaw().getDegrees());

        // SmartDashboard.putNumber("hor off", LimeLight.getHorizontalOffset());
        SmartDashboard.putNumber("XOffset", RetroReflectiveTargeter.getXOffset());
        SmartDashboard.putNumber("YOffset", RetroReflectiveTargeter.getYOffset());


        // SmartDashboard.putNumber("h offset", getPose().getX() -
        // getField().getTagPose(7).get().toPose2d().getX());
        calculateVelocity();
    }
}