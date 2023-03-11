// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RegularConstants;

public class UpperArmToSetpoint extends SubsystemBase {
  /** Creates a new UpperSrmToSetpoint. */
  double LastAngle, LastError;
  double offset = 110;
  double integral;
  double Error = 0;
  DutyCycleEncoder ArmEncoder = new DutyCycleEncoder(9);
  TalonFX ArmMotor = new TalonFX(3, "Bobby");
  double Angle = getAngle();
  public UpperArmToSetpoint() 
  {
    LastAngle = getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   //Moves arm to a desired angle
    
    public double SetArm(double DesiredAngle)
    {
      Angle = getAngle(); 
      Error = DesiredAngle - Angle;
      while(Math.abs(Error) > 2)
      {
        double Angle = getAngle();
        Error = DesiredAngle - Angle;
        double Derivative = FindDerivative(LastError, Error);
        double Proportional = Error;
        integral += Error + LastAngle;
        double output = (Proportional *.01) + (Derivative * .05) +(integral *.004) ;
        SmartDashboard.putNumber("percentOutput", output);
        SmartDashboard.putNumber("armAngle", Angle);
        SmartDashboard.putNumber("Derivative", Derivative * .05);
        SmartDashboard.putNumber("Intergral", integral * .003 );
        LastError = Error;
        LastAngle = Angle;
        ArmMotor.set(ControlMode.PercentOutput,output);
      }
        ArmMotor.set(ControlMode.Disabled, 0);
        return Error;
    }

    //not the real derivative but like it works
    public double FindDerivative(double LastError,double CurrentError)
    { 
      return (LastError - CurrentError)/.2;
    }

    public double Refresh(double angle)
    {
      integral = 0;
      LastAngle = getAngle();
      LastError = angle - LastAngle;
      return LastError;
    }

    public double getAngle() {
      return ((ArmEncoder.getAbsolutePosition() * 360) + offset) % 360;
    }

}
