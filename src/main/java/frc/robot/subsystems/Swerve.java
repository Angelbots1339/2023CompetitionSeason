package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


import frc.robot.SwerveModule;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.robot.LoggingConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.Swerve.PoseEstimatorConstants.*;
import static frc.robot.Constants.Swerve.*;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public LoggedSubsystem logger;
    private PIDController angularDrivePID;
    private SwerveDrivePoseEstimator poseEstimator;

    public Swerve() {

        logger = new LoggedSubsystem("Swerve", LoggingConstants.SWERVE);

        gyro = new Pigeon2(PIGEON_ID);
        gyro.configFactoryDefault();
        zeroGyro();


        swerveOdometry = new SwerveDriveOdometry(KINEMATICS, getYaw(), new SwerveModulePosition[4]);
        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getYaw(), getPositions(), getEstimatedPose(), STATE_STD_DEVS, VISION_MEASUREMENT_STD_DEVS);
    

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Mod0.constants),
                new SwerveModule(1, Mod1.constants),
                new SwerveModule(2, Mod2.constants),
                new SwerveModule(3, Mod3.constants)
        };

        initializeLog();


        angularDrivePID = new PIDController(AngularDriveConstants.ANGLE_KP,
                AngularDriveConstants.ANGLE_KI, AngularDriveConstants.ANGLE_KD);

        angularDrivePID.enableContinuousInput(0, 360);
        angularDrivePID.setTolerance(AngularDriveConstants.TURN_TO_ANGLE_TOLERANCE);
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
        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    private double lastDesiredDegrees;
    private double lastTime;

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
                (lastDesiredDegrees - desiredDegrees.getDegrees()) / (Timer.getFPGATimestamp() - lastTime));
        lastDesiredDegrees = desiredDegrees.getDegrees();
        lastTime = Timer.getFPGATimestamp();

        double rotation = 0;
        // convert yaw into -180 -> 180 and absolute
        double yaw = (0 > getYaw().getDegrees() ? getYaw().getDegrees() % 360 + 360 : getYaw().getDegrees() % 360);
        rotation = MathUtil.clamp(
                angularDrivePID.calculate(yaw, desiredDegrees.getDegrees() + // PID
                        (!angularDrivePID.atSetpoint() ? // feed foward
                                (-desiredAngularVelocity.getRadians() * AngularDriveConstants.ANGLE_KV) + // Kv Velocity
                                                                                                          // Feedfoward
                                        ((Math.signum(angularDrivePID.getPositionError())
                                                * AngularDriveConstants.ANGLE_KS)) // Ks Static Friction Feedforward
                                : 0)),
                -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, Boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);

        }
    }
    /*-----Possible Slutions for Swerve Drive Skew------ */

    // See
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5?u=ethan1
    public ChassisSpeeds reduceSkewFromChassisSpeeds254(ChassisSpeeds chassisSpeeds) {
        frc.lib.team254.geometry.Pose2d robot_pose_vel = new frc.lib.team254.geometry.Pose2d(
                chassisSpeeds.vxMetersPerSecond * LOOPER_DT,
                chassisSpeeds.vyMetersPerSecond * LOOPER_DT,
                frc.lib.team254.geometry.Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * LOOPER_DT));
        frc.lib.team254.geometry.Twist2d twist_vel = frc.lib.team254.geometry.Pose2d.log(robot_pose_vel);
        return new ChassisSpeeds(
                twist_vel.dx / LOOPER_DT, twist_vel.dy / LOOPER_DT, twist_vel.dtheta / LOOPER_DT);

    }

    // See
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1
    public ChassisSpeeds reduceSkewFromChassisSpeedsFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);
        double fudgeFactor = chassisSpeeds.omegaRadiansPerSecond / linearVelocity * FUDGE_FACTOR_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                              // sign of movment
        double unitOrthy = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthy;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    public ChassisSpeeds reduceSkewFromChassisSpeedsSimpleFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);

                
        double fudgeFactor = Math.signum(chassisSpeeds.omegaRadiansPerSecond) * FUDGE_FACTOR_SIMPLE_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                              // sign of movment
        double unitOrthy = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthy;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[4], pose);
        poseEstimator.resetPosition(getYaw(), new SwerveModulePosition[4], pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getPosition();
        }
        return states;
    }

    public void resetToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }


    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
        poseEstimator.update(getYaw(),  getPositions());

        ChassisSpeeds speed = new ChassisSpeeds(0, 2, 10);

        SmartDashboard.putString("t", speed.toString());
        SmartDashboard.putString("254", reduceSkewFromChassisSpeeds254(speed).toString());
        SmartDashboard.putString("ff1", reduceSkewFromChassisSpeedsFudgeFactor(speed).toString());
        SmartDashboard.putString("ff2", reduceSkewFromChassisSpeedsSimpleFudgeFactor(speed).toString());
    }

    public void initializeLog() {
        logger.addDouble("Yaw", () -> gyro.getYaw(), "Gyro");
        logger.addDouble("Roll", () -> gyro.getRoll(), "Gyro");
        logger.addDouble("Pitch", () -> gyro.getPitch(), "Gyro");

        for (int i = 0; i < mSwerveMods.length; i++) {
            logger.add(new LoggedFalcon("Angle Motor: " + i, logger, mSwerveMods[i].getAngleMotor(), "Motor", true));
            logger.add(new LoggedFalcon("Drive Motor: " + i, logger, mSwerveMods[i].getDriveMotor(), "Motor", true));
        }
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        KINEMATICS, // SwerveDriveKinematics
                        new PIDController(3.2023, 0, 0), // X controller. Tune these values for your robot. Leaving them
                                                         // 0 will only use feedforwards.
                        new PIDController(3.2023, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(THETA_KP, 0, 0), // Rotation controller. Tune these values for your robot.
                                                           // Leaving them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer // This argument is optional if you don't use
                                               // event markers
                        this // Requires this drive subsystem
                ));
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        return followTrajectoryCommand(traj, false);
    }

}