// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.DistanceFromAprilTag;

public class Shooter_SS extends SubsystemBase {

  private final CANSparkMax mLeftMotorMaster;
  private final CANSparkMax mLeftMotorSlave;
  private final CANSparkMax mRightMotorMaster;
  private final CANSparkMax mRightMotorSlave;

  private final RelativeEncoder mRelativeEncoderLeft;
  private final RelativeEncoder mRelativeEncoderRight;
  private final SparkPIDController mPidcontrollerleft; 
    private final SparkPIDController mPidcontrollerright; 

    private final DistanceFromAprilTag mDistanceFromAprilTag = new DistanceFromAprilTag();

  /** Creates a new Shooter_SS. */
  public Shooter_SS() {
  
     
    mLeftMotorMaster = new CANSparkMax(19, MotorType.kBrushless);
    mLeftMotorMaster.setIdleMode(IdleMode.kCoast);
    mLeftMotorSlave = new CANSparkMax(20, MotorType.kBrushless);
    mLeftMotorSlave.setIdleMode(IdleMode.kCoast);
    mRightMotorMaster = new CANSparkMax(21, MotorType.kBrushless);
    mRightMotorMaster.setIdleMode(IdleMode.kCoast);
    mRightMotorSlave = new CANSparkMax(22, MotorType.kBrushless);
    mRightMotorSlave.setIdleMode(IdleMode.kCoast);

    mLeftMotorSlave.follow(mLeftMotorMaster);
    mRightMotorSlave.follow(mRightMotorMaster);

    mLeftMotorMaster.setInverted(true);
    mRightMotorMaster.setInverted(true);

    
    mRelativeEncoderLeft = mLeftMotorMaster.getEncoder();
mRelativeEncoderRight = mRightMotorMaster.getEncoder();

    mPidcontrollerleft = mLeftMotorMaster.getPIDController();
    mPidcontrollerleft.setFeedbackDevice(mRelativeEncoderLeft);
    mPidcontrollerleft.setP(0.0001);
    mPidcontrollerleft.setI(0.000001);
    mPidcontrollerleft.setD(0);

     mPidcontrollerright = mRightMotorMaster.getPIDController();
    mPidcontrollerright.setFeedbackDevice(mRelativeEncoderRight);
    mPidcontrollerright.setP(0.0001);
    mPidcontrollerright.setI(0.000001);
    mPidcontrollerright.setD(0);
  
    SmartDashboard.putNumber("ShootLeft P", mPidcontrollerleft.getP());
    SmartDashboard.putNumber("ShootLeft I", mPidcontrollerleft.getI());
    SmartDashboard.putNumber("ShootLeft D", mPidcontrollerleft.getD());
    SmartDashboard.putNumber("velocity left",0);
      
    SmartDashboard.putNumber("ShootRight P", mPidcontrollerright.getP());
    SmartDashboard.putNumber("ShootRight I", mPidcontrollerright.getI());
    SmartDashboard.putNumber("ShootRight D", mPidcontrollerright.getD());
    SmartDashboard.putNumber("velocity right",0);
  }

  public void shoot() {
    mRightMotorMaster.set(0.9);
  }
  
  public void shootauto(double speed) {
    
    }


  public double rightvelocity(){
    switch (mDistanceFromAprilTag.iD()) {
        case 1:
            return Constants.PositionVelocities.RightVelocity1;
        case 2:
            return Constants.PositionVelocities.RightVelocity2;
        case 3:
            return Constants.PositionVelocities.RightVelocity3;
        case 4:
            return Constants.PositionVelocities.RightVelocity4;
        case 5:
            return Constants.PositionVelocities.RightVelocity5;
        default:
            return 0.0;
    }
  }

  public double leftvelocity(){
    switch (mDistanceFromAprilTag.iD()) {
        case 1:
            return Constants.PositionVelocities.LeftVelocity1;
        case 2:
            return Constants.PositionVelocities.LeftVelocity2;
        case 3:
            return Constants.PositionVelocities.LeftVelocity3;
        case 4:
            return Constants.PositionVelocities.LeftVelocity4;
        case 5:
            return Constants.PositionVelocities.LeftVelocity5;
        default:
            return 0.0;
    }
  }

  public double getVelocityLeft(){
    return mRelativeEncoderLeft.getVelocity();
  }

  public void PidSpeed(double velocityleft, double velocityright){ 
    mPidcontrollerleft.setReference(SmartDashboard.getNumber("velocity left", 0), ControlType.kVelocity);
    mPidcontrollerright.setReference(SmartDashboard.getNumber("velocity right", 0), ControlType.kVelocity);


  }

    public void PidSpeedset(double velocityleft, double velocityright){ 
    mPidcontrollerleft.setReference(velocityleft, ControlType.kVelocity);
    mPidcontrollerright.setReference(velocityright, ControlType.kVelocity);


  }
   public double getVelocityRight(){
    return mRelativeEncoderRight.getVelocity();
  }

  

  public void suck() {
    mLeftMotorMaster.set(-0.05);
    mRightMotorMaster.set(-0.05);
  }

  public void stop() {
    mLeftMotorMaster.stopMotor();
    mRightMotorMaster.stopMotor();
  }

  @Override
  public void periodic() {
    /* */
    mPidcontrollerright.setP(SmartDashboard.getNumber("ShootRight P", 0));
    mPidcontrollerright.setI(SmartDashboard.getNumber("ShootRight I", 0));
    mPidcontrollerright.setD(SmartDashboard.getNumber("ShootRight D", 0));

    mPidcontrollerleft.setP(SmartDashboard.getNumber("ShootLeft P", 0));
    mPidcontrollerleft.setI(SmartDashboard.getNumber("ShootLeft I", 0));
    mPidcontrollerleft.setD(SmartDashboard.getNumber("ShootLeft D", 0));

    SmartDashboard.putNumber("current velocity right",getVelocityRight());
    SmartDashboard.putNumber("current velocity left",getVelocityLeft());
  

    // This method will be called once per scheduler run
  }
  }

