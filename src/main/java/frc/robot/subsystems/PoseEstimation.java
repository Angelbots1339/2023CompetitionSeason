// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedField;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Constants.SwerveConstants;;

/** Add your docs here. */
public class PoseEstimation {

    private SwerveDrivePoseEstimator poseEstimator;
    public SwerveDrivePoseEstimator poseEstimatorNonVision;
    private RobotPoseEstimator apriltagPoseEstimator;
    private AprilTagFieldLayout layout;

    public PoseEstimation(LoggedField logger, Rotation2d gyroAngle, SwerveModulePosition[] positions) {

        // Pose Estimator
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        poseEstimatorNonVision = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, gyroAngle, positions,
                new Pose2d(), STATE_STD_DEVS, VISION_MEASUREMENT_STD_DEVS);
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, gyroAngle, positions, new Pose2d(),
                STATE_STD_DEVS,
                VISION_MEASUREMENT_STD_DEVS);

        List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(APRILTAG_CAM, APRILTAG_CAM_POS));
        apriltagPoseEstimator = new RobotPoseEstimator(layout, APRILTAG_POSE_STRATEGY, camList);

        logger.addPose2d("PoseEstimation", () -> poseEstimator.getEstimatedPosition(), true);
        logger.addPose2d("NonVisionPoseEstimation", () -> poseEstimatorNonVision.getEstimatedPosition(), false);
    }

    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        poseEstimatorNonVision.update(gyroAngle, positions);
        poseEstimator.update(gyroAngle, positions);

        // FIXME remove tesing code
        Pose2d desired = layout.getTagPose(6).get().toPose2d()
                .transformBy(new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180)));
        apriltagPoseEstimator.setReferencePose(desired);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = apriltagPoseEstimator.update();
        if (result.isPresent()) {
            if (result.get().getFirst() != null) {

                poseEstimator.addVisionMeasurement(result.get().getFirst().toPose2d(),
                        currentTime - (result.get().getSecond() / 1000));

                layout.getTagPose(6);
                Pose2d error = result.get().getFirst().toPose2d().relativeTo(desired);
                Pose2d relTO = result.get().getFirst().toPose2d().relativeTo(layout.getTagPose(6).get().toPose2d());

                // Transform2d error = desired.relativeTo(result.get().getFirst().toPose2d());

                // SmartDashboard.putNumber("xE", error.getX());
                // SmartDashboard.putNumber("yE", error.getY());
                // SmartDashboard.putNumber("rotE", error.getRotation().getRadians());

                // logger.updateDouble("vision x", result.get().getFirst().toPose2d().getX(),
                // "PoseEstimator");
                // logger.updateDouble("vision y", result.get().getFirst().toPose2d().getY(),
                // "PoseEstimator");
            }
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPoseNonVision() {
        return poseEstimatorNonVision.getEstimatedPosition();
    }

    public Optional<Pose2d> getPoseOnlyVision() {
        Optional<Pair<Pose3d, Double>> result = apriltagPoseEstimator.update();
        if (result.isPresent()) {
            return Optional.of(apriltagPoseEstimator.update().get().getFirst().toPose2d());
        }
        return Optional.empty();
    }

    public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
        poseEstimator.resetPosition(gyroAngle, swerveModulePositions, pose);
        poseEstimatorNonVision.resetPosition(gyroAngle, swerveModulePositions, pose);
    }

    public void resetToVisionPose(SwerveModulePosition[] swerveModulePositions){
        Optional<Pair<Pose3d, Double>> aprilTagEstimation = apriltagPoseEstimator.update();
        if (aprilTagEstimation.isPresent()) {
            poseEstimatorNonVision.resetPosition(aprilTagEstimation.get().getFirst().getRotation().toRotation2d(), swerveModulePositions, aprilTagEstimation.get().getFirst().toPose2d());
            poseEstimator.resetPosition(aprilTagEstimation.get().getFirst().getRotation().toRotation2d(), swerveModulePositions, aprilTagEstimation.get().getFirst().toPose2d());
        }
    }
}
