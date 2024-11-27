// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoop_SS extends SubsystemBase {

  private final TalonSRX mScoopMotor;
  private final DigitalInput mLimitSwitch;
  public boolean ScoopCalibrationDone = false;
  //Need to check, either 2048 or 8192
  final double ticksPerRev = 4096;
  final double gearRatio = 10;
  final double degreesPerTick = 360 / (ticksPerRev * gearRatio);

  final double kP = 0;
  final double kI = 0;
  final double kD = 0;

  public Scoop_SS() {
    mLimitSwitch = new DigitalInput(9);

    mScoopMotor = new TalonSRX(24);
    //set factory default and set basic parameter (not nescessary)
    mScoopMotor.configFactoryDefault();
    mScoopMotor.setNeutralMode(NeutralMode.Brake);
    //set what kind of feedback device we use
    mScoopMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    //to use if forward on the motor is not equal to forward on sensor
    mScoopMotor.setSensorPhase(false);
    //mScoopMotor.configNeutralDeadband(0.04);
    mScoopMotor.configAllowableClosedloopError(0,10 );
    //mScoopMotor.getSensorCollection();
		mScoopMotor.selectProfileSlot(0, 0);
		mScoopMotor.config_kP(0, 0.1, 30);
		mScoopMotor.config_kI(0, 0, 30);
		mScoopMotor.config_kD(0, 0, 30);
    mScoopMotor.config_kF(0, 0, 30);
  }

  public void ScoopCalibration (){
    if ( mLimitSwitch.get() == false) {
    mScoopMotor.set(ControlMode.PercentOutput, -0.15);}
    else if (mLimitSwitch.get() == true) {
      mScoopMotor.set(ControlMode.PercentOutput,0);
      mScoopMotor.setSelectedSensorPosition(0);}
      ScoopCalibrationDone = true;
  }

  public void stop() {
    mScoopMotor.set(ControlMode.PercentOutput, 0);
  }
  
  public void gotoPosition(int pPosition){
      SmartDashboard.putNumber("wanted Scoop position", pPosition);
    mScoopMotor.set(ControlMode.MotionMagic, pPosition/degreesPerTick);
  }

  public void limitSwitch() {
    if (mLimitSwitch.get()){
      mScoopMotor.set(ControlMode.PercentOutput, 0); 
    } 
  }

  public void scoopUP(){
    mScoopMotor.set(ControlMode.PercentOutput,0.1);
  }

  public void scoopDOWN(){
    mScoopMotor.set(ControlMode.PercentOutput,-0.1);
  }
  
  public double GetencoderInDegrees() {
    return mScoopMotor.getSelectedSensorPosition() * degreesPerTick;
  }
  

  /* 
  //im going to change this bs
  public double getOutputForTelemetry() {
    double actualAngle = getScoopAngle();
    double targetAngle = 45;
    //double outputforTelemetry = mPidController.calculate(actualAngle, targetAngle);
    //return outputforTelemetry;

    //TO CHANGE LATER
    return 0;
  }
  */
  
  public double getEncodervalue(){
    return mScoopMotor.getSelectedSensorPosition();
  }
  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Scoop Limit switch", mLimitSwitch.get());
    SmartDashboard.putNumber("encoder Value", getEncodervalue()); 
    SmartDashboard.putNumber("Actual Angle", GetencoderInDegrees());  

  }
}
