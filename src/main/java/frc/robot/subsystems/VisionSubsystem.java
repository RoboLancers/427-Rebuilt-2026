// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
  
public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
                 
                 PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
                 PhotonCamera camera = new PhotonCamera("photonvision");
                  //ent();
                                     
                  
            @SuppressWarnings("unchecked")
            @Override
  public void periodic() {
      // This method will be called once per scheduler run
  
      Object camera;
          for (var change : ((PhotonCamera) camera).getAllUnreadResults()) {
              Object photonEstimator;
                          Object visionEst = ((PhotonPoseEstimator) photonEstimator).update(change);
              updateEstimationStdDevs(visionEst, change.getTargets());
  
              if (Robot.isSimulation()) {
                  ((Optional<EstimatedRobotPose>) visionEst).ifPresentOrElse(
                          est ->
                                     getSimDebugField().getObject("VisionEstimation")
                                          .setPose(est.estimatedPose.toPose2d()),
                          () -> {
                              getSimDebugField().getObject("VisionEstimation").setPoses();
                          });
              }
  
              ((Optional<EstimatedRobotPose>) visionEst).ifPresent(
                      est -> {
                          // Change our trust in the measurement based on the tags we can see
                          var estStdDevs = getEstimationStdDevs();
                          
                                                                      Vision Vision = new Vision(drivetrain::addVisionMeasurement);     estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                                              });
                                }
                          }
                        
                          private Object getEstimationStdDevs() {
                            // TODO Auto-generated method stub
                            throw new UnsupportedOperationException("Unimplemented method 'getEstimationStdDevs'");
                        }
                                
                                private Object getSimDebugField() {
            // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getSimDebugField'");
}

}