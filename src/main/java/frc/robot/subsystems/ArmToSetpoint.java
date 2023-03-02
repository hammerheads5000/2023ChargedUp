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
    boolean IsUpperArm;
    DigitalInput Lower_ArmBackwardsSwitch;
    DigitalInput Lower_ArmForwardsSwitch;
    DigitalInput Upper_MaxWhileForwardsSwitch;
    DigitalInput Upper_MaxWhileBackwardsSwitch;
    DigitalInput Upper_BringArmUpSafetySwitch;
    DigitalInput Upper_AtPostSwitch;
    

    /* Creates a new Pneumatics subsystem */
    public ArmToSetpoint 
    (
      TalonFX ArmMotor,
      boolean IsUpperArm, 
      DigitalInput Lower_ArmBackwardsSwitch,  //Switch is triggered when lower arm is back
      DigitalInput Lower_ArmForwardSwitch,  // Switch is triggered when the lower arm is forward
      DigitalInput Upper_MaxWhileForwardSwitch, //switch is triggered when the upper arm is at the angle where it will break height limit if lower arm is forward
      DigitalInput Upper_MaxWhileBackwardsSwitch, //switch is triggered when the upper arm is at the point where it will break height limit if lower arm is back
      DigitalInput Upper_BringArmUpSafetySwitch, //Is triggered when upper arm is at the angle where it is safe to bring lower arm back without breaking height limit
      DigitalInput Upper_AtPostSwitch //This switch will be triggered when the upper arm is at the angle where it will be over the goalpost if the lower arm is forward
      ) 
    {
      this.ArmMotor = ArmMotor;
      this.IsUpperArm =IsUpperArm;
      this.Lower_ArmBackwardsSwitch =Lower_ArmBackwardsSwitch;
      this.Lower_ArmForwardsSwitch = Lower_ArmForwardSwitch;
      this.Upper_MaxWhileForwardsSwitch = Upper_MaxWhileForwardSwitch;
      this.Upper_MaxWhileBackwardsSwitch =Upper_MaxWhileBackwardsSwitch;
      this.Upper_BringArmUpSafetySwitch = Upper_BringArmUpSafetySwitch;
      this.Upper_AtPostSwitch = Upper_AtPostSwitch;

      ArmMotor.setNeutralMode(NeutralMode.Brake);
      
      //sets the initil position of the arm segment
      StartEncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();

       /*sets the preset angles depending on which arm instance it is
      if(isUpperArm)
      {
        HomeAngle = 20;
        UpperAngle = 20;
      }
      else
      {
        HomeAngle = 1;
        UpperAngle = 3;
      } */
    }
    //Moves arm up at a given speed
    public void moveUp(double speed)
    {
      ArmMotor.set(TalonFXControlMode.PercentOutput, speed); 
    }
    //Moves arm down at a given speed
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

    //keeps the arm from never decelerating
    public void stop()
    {
      ArmMotor.set(TalonFXControlMode.Disabled,0);
    }

    //checks if moving the arm in a given direction would resulting in breaking any boundaries 
    public boolean safetyCheck()
    {
      //safety check for upper arm
      if(IsUpperArm)
      {

      }
    }
}
