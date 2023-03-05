// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RegularConstants;

public class UpperArmToSetpoint extends SubsystemBase {
  /** Creates a new UpperSrmToSetpoint. */
  TalonFX ArmMotor = new TalonFX(3, "Bobby");
    public double angle = 0.0;
    public double StartEncoderTicks;
    public double AngleDif;
    double ArmKI;
    double ArmRatio;
    double ArmMax;
    double ArmMin;
    double HomeAngle;
    double UpperAngle; 
  public UpperArmToSetpoint() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   //Moves arm to a desired angle
    
    public void MoveArm (double DesiredAngle)
    {

      double tempOutput;
        ArmKI = RegularConstants.UpperArmKI;
        ArmRatio = RegularConstants.UpperArmRatio;
        ArmMax = RegularConstants.UpperArmMax;
        ArmMin = RegularConstants.UpperArmMin;

      double TempEncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      
      //Calculates angle based on last angle and difference in encoder ticks  
      angle = -(TempEncoderTicks- StartEncoderTicks) / ArmRatio;

      //finds distance from current angle to desired angle
      AngleDif = angle - DesiredAngle;
      double AngleDifAbsolute = Math.abs(AngleDif);
      //Acounts for distance
      if(AngleDifAbsolute > RegularConstants.AnglePIDStarts)
      {
        tempOutput = ArmMax;
      }
      else
      {
        tempOutput = Math.max(ArmMin, AngleDifAbsolute*ArmMax*ArmKI);
      }
  
      //moves arm motor in the direction it should go in
      ArmMotor.set(TalonFXControlMode.PercentOutput, tempOutput*Math.signum(AngleDif)); //optimized
      
      //Makes my variables beholdable
      SmartDashboard.putNumber("encoder Ticks", TempEncoderTicks);
      SmartDashboard.putNumber("AngleDif", AngleDif);
      SmartDashboard.putNumber("Angle", angle);
    }

    public void AngleSet(double angle)
    {
      this.angle = angle;
    } 
}
