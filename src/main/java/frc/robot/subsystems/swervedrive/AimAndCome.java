// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

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

  double kP = 0.9;
  double kI = 0.01;
  double kD = 0.0;

  double kP2 = 2.4;
  double kI2 = 0.05;
  double kD2 = 0.0;

  PIDController mPidController1 = new PIDController(kP, kI, kD);
    PIDController mPidController2 = new PIDController(kP2, kI2, kD2);


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
  public double ForwardAim(){
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
    

    if(x > 0.62){
        forward = mPidController1.calculate(x, 0.5);
    } else if(x < 0.62){
        forward = mPidController2.calculate(x, 0.5);
             System.out.println("pid2" + forward);


    }
     System.out.println("forward:" + -forward);
     System.out.println("x" + x);
    
  
     
 
    return -forward;
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
