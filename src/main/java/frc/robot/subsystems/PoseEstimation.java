// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.util.logging.loggedObjects.LoggedField;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Constants.SwerveConstants;
import frc.robot.regressions.KalmanVisionRegression;;

/** Add your docs here. */
public class PoseEstimation {

    private SwerveDrivePoseEstimator poseEstimator;
    public SwerveDrivePoseEstimator poseEstimatorNonVision;
    public SwerveDrivePoseEstimator poseEstimatorNonGyroAngle;
    private AprilTagFieldLayout layout;

    public PoseEstimation(LoggedField logger, Rotation2d gyroAngle, SwerveModulePosition[] positions) {

        // Pose Estimator
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        poseEstimatorNonVision = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, gyroAngle, positions,
                new Pose2d(), STATE_STD_DEVS, VecBuilder.fill(0, 0, 0));

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, gyroAngle, positions, new Pose2d(),
                STATE_STD_DEVS, VecBuilder.fill(0, 0, 0));

        poseEstimatorNonGyroAngle = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, gyroAngle, positions,
                new Pose2d(),
                STATE_STD_DEVS, VecBuilder.fill(0, 0, 0));

        List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(APRILTAG_CAM, APRILTAG_CAM_POS));

        logger.addPose2d("Robot", () -> poseEstimator.getEstimatedPosition(), true);
        logger.addPose2d("NonVisionPoseEstimation", () -> poseEstimatorNonVision.getEstimatedPosition(), false);
        logger.addPose2d("NonGyroAnglePoseEstimation", () -> poseEstimatorNonGyroAngle.getEstimatedPosition(), false);
    }

    public void updateOdometry(Rotation2d gyroAngle, SwerveModulePosition[] positions, Pose2d referencePose) {
        poseEstimatorNonVision.update(gyroAngle, positions);
        poseEstimator.update(gyroAngle, positions);

        PhotonPipelineResult result = APRILTAG_CAM.getLatestResult();
        if (result.hasTargets()) {

            double smallestPoseDelta = 10e9;
            EstimatedRobotPose lowestDeltaPose = null;
            Translation2d robotToTarget = null;

            for (PhotonTrackedTarget target : result.getTargets()) {
                int id = target.getFiducialId();
                if (id > 8 || id < 1)
                    continue;
                Pose3d targetPostiton = layout.getTagPose(target.getFiducialId()).get();

                // TODO check sign of pitch and maybe add pitch from gyro
                Rotation3d gyroCalculatedAngle;
                if (id > 4)
                    gyroCalculatedAngle = new Rotation3d(0, -APRILTAG_CAM_POS.getRotation().getY(),
                            -gyroAngle.getRadians());
                else
                    gyroCalculatedAngle = new Rotation3d(0, -APRILTAG_CAM_POS.getRotation().getY(),
                            (gyroAngle.getDegrees() > 0 ? 1 : -1) * 180 - gyroAngle.getDegrees());

                Translation3d transformToTarget = target.getBestCameraToTarget().getTranslation();

                Pose3d estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        new Transform3d(transformToTarget, gyroCalculatedAngle), targetPostiton, APRILTAG_CAM_POS);

                double poseDelta = referencePose.getTranslation()
                        .getDistance(estimatedPose.getTranslation().toTranslation2d());
                if (poseDelta < smallestPoseDelta) {
                    smallestPoseDelta = poseDelta;
                    lowestDeltaPose = new EstimatedRobotPose(estimatedPose, result.getTimestampSeconds());
                    robotToTarget = transformToTarget.toTranslation2d();
                }

            }

            double tagDistance = robotToTarget.getNorm();
            double xyStdDev = KalmanVisionRegression.xyStdDevReg.predict(tagDistance);

            poseEstimator.addVisionMeasurement(lowestDeltaPose.estimatedPose.toPose2d(),
                    lowestDeltaPose.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 0));
            Optional<Pair<EstimatedRobotPose, Translation2d>> poseOnlyVision = getPoseOnlyVision(referencePose);
            if (poseOnlyVision.isPresent()) {
                tagDistance = poseOnlyVision.get().getSecond().getNorm();
                xyStdDev = KalmanVisionRegression.xyStdDevReg.predict(tagDistance);
                poseEstimatorNonGyroAngle.addVisionMeasurement(poseOnlyVision.get().getFirst().estimatedPose.toPose2d(),
                        poseOnlyVision.get().getFirst().timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 0));
            }
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPoseNonVision() {
        return poseEstimatorNonVision.getEstimatedPosition();
    }

    public Optional<Pair<EstimatedRobotPose, Translation2d>> getPoseOnlyVision(Pose2d referencePose) {
        PhotonPipelineResult result = APRILTAG_CAM.getLatestResult();
        if (result.hasTargets()) {
            EstimatedRobotPose lowestDeltaPose = null;
            double smallestPoseDelta = 10e9;
            Translation2d robotToTarget = null;

            for (PhotonTrackedTarget target : result.getTargets()) {
                int id = target.getFiducialId();
                if (id > 8 || id < 1)
                    continue;
                Pose3d targetPostiton = layout.getTagPose(target.getFiducialId()).get();

                Transform3d transformToTarget = target.getBestCameraToTarget();

                Pose3d estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(transformToTarget, targetPostiton,
                        APRILTAG_CAM_POS);

                double poseDelta = referencePose.getTranslation()
                        .getDistance(estimatedPose.getTranslation().toTranslation2d());
                if (poseDelta < smallestPoseDelta) {
                    smallestPoseDelta = poseDelta;
                    lowestDeltaPose = new EstimatedRobotPose(estimatedPose, result.getTimestampSeconds());
                    robotToTarget = transformToTarget.getTranslation().toTranslation2d();
                }
            }
            return Optional.of(Pair.of(lowestDeltaPose, robotToTarget));
        }
        return Optional.empty();
    }

    public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
        poseEstimator.resetPosition(gyroAngle, swerveModulePositions, pose);
        poseEstimatorNonVision.resetPosition(gyroAngle, swerveModulePositions, pose);
    }

    public void alignPoseNonVisionEstimator(SwerveModulePosition[] swerveModulePositions) {
        poseEstimatorNonVision.resetPosition(getPose().getRotation(), swerveModulePositions, getPose());
    }

}
