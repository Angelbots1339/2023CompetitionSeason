package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPoint;

import frc.robot.SwerveModule;
import frc.robot.commands.Auto.SwerveFollowTrajectory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.logging.loggedObjects.LoggedPigeon2;
import frc.lib.util.logging.loggedObjects.LoggedSwerveModule;
import frc.robot.LoggingConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.PIDToPoseConstants.*;

public class Swerve extends SubsystemBase {
    public SwerveModule[] swerveMods;
    public Pigeon2 gyro;
    public LoggedSubsystem logger;
    
    private Translation2d currentVel = new Translation2d();
    private Pose2d lastPose = new Pose2d();

    private PoseEstimation poseEstimation;
    private LoggedField field;

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
        
        //logging
        logger = new LoggedSubsystem("Swerve", LoggingConstants.SWERVE);

        field = new LoggedField("PoseEstimator", logger, "PoseEstimator", true);
        poseEstimation = new PoseEstimation(field, getYaw(), getPositions());

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

        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw());
        } else {
            chassisSpeeds = new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation);
        }
        chassisSpeeds = reduceSkewFromChassisSpeeds254(chassisSpeeds);
        logger.updateDouble("x", chassisSpeeds.vxMetersPerSecond, "Drive");
        logger.updateDouble("y", chassisSpeeds.vyMetersPerSecond, "Drive");
        logger.updateDouble("rot", chassisSpeeds.omegaRadiansPerSecond, "Drive");


        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(swerveModuleStates, isOpenLoop);
    }



    private PIDController angularDrivePID = new PIDController(AngularDriveConstants.ANGLE_KP,
    AngularDriveConstants.ANGLE_KI, AngularDriveConstants.ANGLE_KD);
    private void configPidAngularDrive() {
        angularDrivePID.enableContinuousInput(0, 360);
        angularDrivePID.setTolerance(AngularDriveConstants.TURN_TO_ANGLE_TOLERANCE);
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
        if (Math.abs(angularDrivePID.getPositionError()) >= 2) {
            rotation = MathUtil.clamp(pid + (-desiredAngularVelocity.getRadians() * AngularDriveConstants.ANGLE_KV) + // Kv
                                                                                                                      // //
                                                                                                                      // Velocity
            // Feedfoward
                    ((Math.signum(angularDrivePID.getPositionError())
                            * AngularDriveConstants.ANGLE_KS)) // Ks Static Friction Feedforward
                    , -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        }

        logger.updateDouble("Output rotation", rotation, "AngularDrive");
        logger.updateBoolean("At setpoint", angularDrivePID.atSetpoint(),
                "AngularDrive");

        drive(translation, rotation, fieldRelative, isOpenLoop);
    }
    /*-----Possible Solutions for Swerve Drive Skew------ */

    

    
    private double previousDriveTime;

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     * See {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5?u=ethan1}
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
     * See {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
     */
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
     * See {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
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
     * Takes the unit orthogonal vector and scales it by a constant times the angular velocity
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
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    }

     /* Used by SwerveControllerCommand in Auto */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(swerveModuleStates, true);
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

    public void resetOdometry(Pose2d pose) {
        poseEstimation.resetPose(getYaw(), getPositions(), pose);
    }
    public void resetToVision() {
        poseEstimation.resetToVisionPose(getPositions());
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /*-----Getters----- */

    public Pose2d getPoseOdometry() {
        return poseEstimation.getPoseNonVision();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimation.getPose();
    }



    private double lastTimePeriodic;
    private double lastTimeDrive;
    private double lastDesiredDegrees;

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
        currentVel = getEstimatedPose().minus(lastPose).getTranslation().div(deltaTime);
        lastPose = getEstimatedPose();
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


    private PIDController pidToPoseXController = new PIDController(PID_TO_POSE_X_P, PID_TO_POSE_X_I, PID_TO_POSE_X_D);;
    private PIDController pidToPoseYController = new PIDController(PID_TO_POSE_Y_P, PID_TO_POSE_Y_I, PID_TO_POSE_Y_D);;

    private void configPIDtoPoseControllers(){
        pidToPoseXController.setTolerance(PID_TO_POSE_TOLERANCE);
        pidToPoseYController.setTolerance(PID_TO_POSE_TOLERANCE);
    }
    /**
     * Use this to have the robot move to a position. Ideally use when the robot
     * isn't very far away.
     * 
     * @param target The Pose for the robot to go to
     * @return Whether or not the robot has gotten to within tolerance on both
     *         translation and rotation
     */
    public boolean PIDToPose(Pose2d target) {

        logger.updateDouble("set x", target.getX(), "PidPose");
        logger.updateDouble("set y", target.getY(), "PidPose");

        Translation2d calculatedValues = new Translation2d(
                pidToPoseXController.calculate(getEstimatedPose().getTranslation().getX(), target.getX()),
                pidToPoseYController.calculate(getEstimatedPose().getTranslation().getY(), target.getY()));
        
        angularDrive(calculatedValues, target.getRotation(), true, false);
        
        logger.updateDouble("out x", calculatedValues.getX(), "PidPose");
        logger.updateDouble("out y", calculatedValues.getY(), "PidPose");

        return pidToPoseXController.atSetpoint() && pidToPoseYController.atSetpoint() && angularDrivePID.atSetpoint();
    }

    public void TestTrajectoryGeneration(PathPoint endpoint) {
        field.setTrajectory("AlignTraj", SwerveFollowTrajectory.SwerveGenerateTrajectoryToPoint(endpoint, this), true);
    }

    public void initializeLog() {
        logger.add(field);
        logger.add(new LoggedPigeon2("Gyro", logger, gyro, "Gyro"));


        logger.addDouble("Heading", () -> getHeading().getDegrees(), "Pose");
        logger.addDouble("x velocity", () -> getCurrentVelocity().getX(), "Pose");
        logger.addDouble("y velocity", () -> getCurrentVelocity().getY(), "Pose");

        logger.add(new LoggedSwerveModule("Modules", logger, swerveMods, "Module", true));

        for (int i = 0; i < swerveMods.length; i++) {
            logger.add(new LoggedFalcon("Angle Motor: " + i, logger, swerveMods[i].getAngleMotor(), "Motor", true));
            logger.add(new LoggedFalcon("Drive Motor: " + i, logger, swerveMods[i].getDriveMotor(), "Motor", true));
        }
    }

    @Override
    public void periodic() {
        poseEstimation.updateOdometry(getHeading(), getPositions());
        calculateVelocity();
    }
}