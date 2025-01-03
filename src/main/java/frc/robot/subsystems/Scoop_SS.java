// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.concurrent.TimeUnit;

import javax.swing.text.Position;

import frc.robot.subsystems.Actuator_SS;

public class Scoop_SS extends SubsystemBase {

  private final TalonSRX mScoopMotor;
  public final DigitalInput mLimitSwitch;
  public boolean ScoopCalibrationDone = false;
  //Need to check, either 2048 or 8192
  final double gearRatio = 10;
  final double tickPerDegree = 40960 / 360;
  public boolean mDONTSTOPINSTANTLY;
  public boolean mSTOP = false;

  double kMeasuredPosHorizontal = 30 * tickPerDegree ; //Position measured when arm is horizontal
  double currentPos ;
  double degrees;
  double radians ;
  double cosineScalar ;
  Double kP;
  Double kI;
  Double kD;
  public Scoop_SS() {
    mLimitSwitch = new DigitalInput(0);
    
    mScoopMotor = new TalonSRX(24);
    //set factory default and set basic parameter (not nescessary)
    mScoopMotor.configFactoryDefault();
    mScoopMotor.setNeutralMode(NeutralMode.Coast);
    //set what kind of feedback device we use
    mScoopMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    //to use if forward on the motor is not equal to forward on sensor
    mScoopMotor.setSensorPhase(true);
    mScoopMotor.configAllowableClosedloopError(0,0 );
    
		mScoopMotor.selectProfileSlot(0, 0);
		mScoopMotor.config_kP(0, 0.6, 30);
    SmartDashboard.putNumber("kP",0.6);
		mScoopMotor.config_kI(0, 0.0001, 30);
    SmartDashboard.putNumber("kI",0.0001);
		mScoopMotor.config_kD(0, 0, 30);
    SmartDashboard.putNumber("kD",0);
    mScoopMotor.config_kF(0, 0, 30);
    mScoopMotor.configClosedLoopPeakOutput(0,1);
  }

  public void ScoopCalibration (){
    if ( mLimitSwitch.get() == false) {
    mScoopMotor.set(ControlMode.PercentOutput, 0);}
    else if (mLimitSwitch.get() == true) {
      mScoopMotor.set(ControlMode.PercentOutput,0);
      mScoopMotor.setSelectedSensorPosition(0);}
      ScoopCalibrationDone = true;
  }

  public void stop() {
    mScoopMotor.set(ControlMode.PercentOutput, 0);
  }
  
  public void gotoPosition(Double pPosition){
    currentPos = mScoopMotor.getSelectedSensorPosition();
    degrees = (currentPos - kMeasuredPosHorizontal) / tickPerDegree;
    radians = java.lang.Math.toRadians(degrees);
    cosineScalar = java.lang.Math.cos(radians);
    SmartDashboard.putNumber("cosine",cosineScalar);
    SmartDashboard.putNumber("wanted Scoop position",pPosition);
    mScoopMotor.set(ControlMode.Position, pPosition * tickPerDegree);
    //mScoopMotor.set(ControlMode.Position, pPosition * tickPerDegree,DemandType.ArbitraryFeedForward,0.25*cosineScalar);
    //mScoopMotor.set(ControlMode.Position, 30 * tickPerDegree);

  }


  public boolean speed_0(){
   return mScoopMotor.getSelectedSensorVelocity()== 0;
  }

  public void scooptoArm(double armPosition) {
    mScoopMotor.set(ControlMode.PercentOutput, 0.35);
    if (getEncoderInDegrees() > armPosition-29) {
      mScoopMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void scoopUP(){
    mScoopMotor.set(ControlMode.PercentOutput,0.7);
  }
  public void scoopMedium(){

    mScoopMotor.set(ControlMode.PercentOutput,0.25);
  }

  public void scoopDOWN(){
    mScoopMotor.set(ControlMode.PercentOutput, -0.27);
  }
  
  public double getEncoderInDegrees() {
    return mScoopMotor.getSelectedSensorPosition()/ tickPerDegree;
  }

  
  public void setBrakeMode() {
    mScoopMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  
  public double getEncodervalue(){
    return mScoopMotor.getSelectedSensorPosition();
  }
  @Override
  public void periodic() {

    mScoopMotor.config_kP(0, SmartDashboard.getNumber("kP",0.6), 30);
		mScoopMotor.config_kI(0, SmartDashboard.getNumber("kI",0.0001), 30);
		mScoopMotor.config_kD(0,SmartDashboard.getNumber("kD",0), 30);


    SmartDashboard.putBoolean("LimiySwitch", mLimitSwitch.get());
    SmartDashboard.putBoolean("LimiySwitch2", !mLimitSwitch.get());
    SmartDashboard.putNumber("scoop encoder Value", getEncodervalue()); 
    SmartDashboard.putNumber("Actual Angle", getEncoderInDegrees());  
    SmartDashboard.putNumber("Scoop Speed", mScoopMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Degrees per tick", tickPerDegree);
    SmartDashboard.putNumber("Scoop output",mScoopMotor.getMotorOutputPercent());
  }
}
