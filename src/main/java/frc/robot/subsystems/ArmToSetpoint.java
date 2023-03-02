// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.RegularConstants;
import frc.robot.commands.ArmAtLimit;
public class ArmToSetpoint extends SubsystemBase {

  /*Behold, My variables */
    TalonFX ArmMotor;
    public double angle = 0.0;
    public double StartEncoderTicks;
    public double AngleDif;
    double ArmKI;
    double ArmRatio;
    double ArmMax;
    double ArmMin;
    double HomeAngle;
    double UpperAngle;
    DigitalInput HomeSwitch;
    DigitalInput UpperSwitch;
    DigitalInput LowestSwitch;
    boolean IsUpperArm;
    

    /* Creates a new Pneumatics subsystem */
    public ArmToSetpoint (TalonFX ArmMotor, DigitalInput UpperLimitSwitch, DigitalInput LowerLimitSwitch,boolean UpperArm, DigitalInput LowestSwitch) 
    {
      this.LowestSwitch = LowestSwitch;
      if(UpperArm)
      {
        HomeAngle = 20;
        UpperAngle = 20;
      }
      else
      {
        HomeAngle = 1;
        UpperAngle = 3;
      }
      HomeSwitch = LowerLimitSwitch;
      UpperSwitch = LowerLimitSwitch;
      this.ArmMotor = ArmMotor;
      IsUpperArm =UpperArm;
      ArmMotor.setNeutralMode(NeutralMode.Brake);
      //sets the initil position of the arm segment
      StartEncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    public void moveUp(double speed)
    {
      if(UpperSwitch.get() && IsUpperArm) 
      {
          moveDown(.3);
          return;
      }
      ArmMotor.set(TalonFXControlMode.PercentOutput, speed); 
    }

    public void moveDown(double speed)
    {
      ArmMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }
    
    //Moves arm to a desired angle
    public void MoveArm (double DesiredAngle, boolean ifIsUpperArm)
    {
      double tempOutput;
      if(ifIsUpperArm)
      {
        ArmKI = RegularConstants.UpperArmKI;
        ArmRatio = RegularConstants.UpperArmRatio;
        ArmMax = RegularConstants.UpperArmMax;
        ArmMin = RegularConstants.UpperArmMin;
      }
      else
      {
        ArmKI = RegularConstants.LowerArmKI;
        ArmRatio = RegularConstants.LowerArmRatio;
        ArmMax = RegularConstants.LowerArmMax;
        ArmMin = RegularConstants.LowerArmMin;
      }
      double TempEncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      
      //Calculates angle based on last angle and difference in encoder ticks  
      angle = -(TempEncoderTicks- StartEncoderTicks) / ArmRatio;

      //finds distance from current angle to desired angle
      AngleDif = angle - DesiredAngle;
      double AngleDifAbsolute = Math.abs(AngleDif);
        //Acounts for distance
        if(AngleDifAbsolute > 15)
        {
          tempOutput = ArmMax;
        }
        else
        {
          if(AngleDifAbsolute * ArmMax * ArmKI < ArmMin)
          {
            tempOutput = ArmMin;
            if(AngleDifAbsolute == 0)
            {
              tempOutput = 0;
            }
          }
          else
          {
            tempOutput = AngleDifAbsolute * ArmMax * ArmKI;
          }
        }
  
      //moves arm motor in the direction it should go in
      if((AngleDif)>0)
      {
        ArmMotor.set(TalonFXControlMode.PercentOutput, tempOutput);
      }
      else if (AngleDif < 0)
      {
        ArmMotor.set(TalonFXControlMode.PercentOutput, -tempOutput);
      }
      else
      {
        //yay
      }
      
      //Makes my variables beholdable
      SmartDashboard.putNumber("encoder Ticks", TempEncoderTicks);
      SmartDashboard.putNumber("AngleDif", AngleDif);
      SmartDashboard.putNumber("Angle", angle);
    }

    // zeros the current angle, will be used with limit switches
    public void Set()
    {
      if(IsUpperArm && UpperSwitch.get())
      {
       moveDown(.3);
      }
      if(HomeSwitch.get())
      {
        angle = HomeAngle;
        StartEncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      }
      if(UpperSwitch.get())
      {
       // angle = UpperAngle;
      }
      if(IsUpperArm)
      {
          SmartDashboard.putNumber("Upper Angle", angle);
      }
      else
      {
        SmartDashboard.putNumber("Lower Angle", angle);
      }
    }
    //keeps the arm from never decelerating
    public void stop()
    {
      ArmMotor.set(TalonFXControlMode.Disabled,0);
  }
}
