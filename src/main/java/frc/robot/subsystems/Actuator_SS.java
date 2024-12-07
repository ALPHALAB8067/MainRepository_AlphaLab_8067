// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator_SS extends SubsystemBase {

  private final CANSparkMax mActuator;
  private final RelativeEncoder mEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private final SparkPIDController mPIDcontroller;
  public boolean ActuatorCalibrationDone = false;

  /** Creates a new Actuator_SS. */
  public Actuator_SS() {
    kP = 0.75; 
    kI = 0.00005;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
  //motor setup
  mActuator = new CANSparkMax(23,MotorType.kBrushed); 
  
  
  //encoder setup
  mEncoder = mActuator.getEncoder(Type.kQuadrature,8192);
  mEncoder.setInverted(true);
  mEncoder.setPositionConversionFactor(240);

  //pid setup
  mPIDcontroller = mActuator.getPIDController();
  mPIDcontroller.setFeedbackDevice(mEncoder);
  mPIDcontroller.setP(kP);
  mPIDcontroller.setI(kI);
  mPIDcontroller.setD(kD);
  mPIDcontroller.setFF(kFF);
  mPIDcontroller.setIZone(kIz);
  mPIDcontroller.setOutputRange(kMinOutput,kMaxOutput);
  
  SmartDashboard.putNumber("actuator setpoint",25);
  SmartDashboard.putNumber("Actuator P",1);
  SmartDashboard.putNumber("Actuator I",0.000001 );
  SmartDashboard.putNumber("Actuator D", 0);

  }

  public void encoderReset(){
    mEncoder.setPosition(0);
  }

  public void Actuator_up (){
    mActuator.set(1);
  }
  
  public void Actuator_down (){
    mActuator.set( -1);
  }
  public void stop(){
    mActuator.set(0);
  }

  public void gotoPosition(double pPosition){
    SmartDashboard.putNumber("wanted Actuator position", pPosition);
    mPIDcontroller.setReference(pPosition, ControlType.kPosition);
  }

  public double getPosition() {
    return mEncoder.getPosition();
  }

  public void gotopositionfromdashboard(){
    mPIDcontroller.setReference(SmartDashboard.getNumber("actuator setpoint", 25),ControlType.kPosition);
  }

  @Override
  public void periodic() {
    mPIDcontroller.setP(SmartDashboard.getNumber("Actuator P",1));
    mPIDcontroller.setI(SmartDashboard.getNumber("Actuator I",0.000001 ));
    mPIDcontroller.setD(SmartDashboard.getNumber("Actuator D", 0));
    SmartDashboard.putNumber("ActuatorEncoderValue", mEncoder.getPosition());
    SmartDashboard.putNumber("ActuatorEncoderSpeed", mEncoder.getVelocity());
    // This method will be called once per scheduler run
  }
}
