// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RegularConstants;

public class UpperArmToSetpoint extends SubsystemBase {
  /** Creates a new UpperSrmToSetpoint. */
  double lastAngle, lastError;
  Timer deltaTimer;
  double offset = 110;
  double error = 0;
  DutyCycleEncoder armEncoder = new DutyCycleEncoder(9);
  TalonFX armMotor = new TalonFX(3, "Bobby");
  double angle = getAngle();
  public UpperArmToSetpoint() 
  {
    lastAngle = getAngle();
    deltaTimer = new Timer();
    deltaTimer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   //Moves arm to a desired angle
    
  public double SetArm(double desiredAngle)
  {
    angle = getAngle(); 
    error = desiredAngle - angle;
    while(Math.abs(error) > ArmConstants.setPointToleranceDegrees)
    {
      angle = getAngle();
      error = desiredAngle - angle;
      double derivative = FindDerivative(lastError, error);
      double proportional = error;
      double output = (proportional *ArmConstants.kP) + (derivative *ArmConstants.kD);
      lastError = error;
      lastAngle = angle;
      armMotor.set(ControlMode.PercentOutput, output);
    }
      armMotor.set(ControlMode.Disabled, 0);
      return error;
  }

  public double FindDerivative(double lastError,double currentError)
  { 
    double derivative = (lastError - currentError) / deltaTimer.get();
    deltaTimer.reset();
    return derivative;
  }

  public double Refresh(double angle)
  {
    lastAngle = getAngle();
    lastError = angle - lastAngle;
    return lastError;
  }

  public double getAngle() {
    return ((armEncoder.getAbsolutePosition() * 360) + offset) % 360;
  }

}
