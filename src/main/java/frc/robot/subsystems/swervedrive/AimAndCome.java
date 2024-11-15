// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AimAndCome extends SubsystemBase {
  /** Creates a new AimAndAlign_SS. */

  boolean targetVisible = false; 
  double targetYaw = 0.0;
  PhotonCamera camera = new PhotonCamera("photo2");
  double iD;
  double targetRange = 0.0;
  double maxforwardspeed;
  double forward;
  double x;


  public AimAndCome() {

  
  }
  
  public double metersfromapriltagx(){
    var results = camera.getLatestResult();

    if (results.hasTargets()){
      var target = results.getBestTarget();
      Transform3d bestcameratotarget = target.getBestCameraToTarget();
      x = bestcameratotarget.getX();
      targetVisible = true;
      iD = target.getFiducialId();
      }  
      return x;
    
    }
  public double ForwardAim(double turnKP, double maxforwardspeed){
     var results = camera.getLatestResult();


    
    if (results.hasTargets()){
      var target = results.getBestTarget();
      targetYaw = target.getYaw();
      targetVisible = true;
      iD = target.getFiducialId();
      Transform3d bestcameratotarget = target.getBestCameraToTarget();
      x = bestcameratotarget.getX();

      /*targetRange =
          PhotonUtils.calculateDistanceToTargetMeters(
                                        0, // Measured with a tape measure, or in CAD.
                                        0, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(0.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));
*/
      }  
    forward = (0.5 - x) * turnKP * -maxforwardspeed;
    return forward;
  }

  public double WhichPosition(){
 var results = camera.getLatestResult();

    return iD;
    }
      

  @Override 
  public void periodic() {
    SmartDashboard.putNumber("Position", x);
    // This method will be called once per scheduler run
  }
}
