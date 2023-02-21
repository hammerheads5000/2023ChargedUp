
package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RegularConstants;
public class ArmToSetpoint extends SubsystemBase {

  /*Behold, My variables */
   TalonFX Up_ArmMotor = new TalonFX(3,"Bobby");
   TalonFX Low_ArmMotor = new TalonFX(26,"Bobby");
   public double Up_angle = 0.0;
   public double  Low_angle = 0.0;
   public double Up_StartEncoderTicks;
   public double Low_StartEncoderTicks;
   public double Up_AngleDif;
   public double Low_AngleDif;
   public boolean AtSetpoint;


    /* Creates a new ArmToSetpoint subsystem */
    public ArmToSetpoint () 
    {
      Up_ArmMotor.setNeutralMode(NeutralMode.Brake);
      Low_ArmMotor.setNeutralMode(NeutralMode.Brake);
      //sets the initil position of the arm segment
      Up_StartEncoderTicks = Up_ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      Low_StartEncoderTicks = Up_ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      AtSetpoint = true;
    }
    
    //Moves arm to a desired angle
    public void MoveArm (double UpperDesiredAngle, double LowerDesiredAngle)
    {
      //Theres litteraly so many variables and constants like holy bejesus 
      double Up_ArmKI = RegularConstants.UpperArmKI;
      double Low_ArmKI = RegularConstants.LowerArmKI;
      double Up_ArmRatio = RegularConstants.UpperArmRatio;
      double Low_ArmRatio = RegularConstants.LowerArmRatio;
      double Up_tempOutput;
      double Low_tempOutput;
      double Up_ArmMax = RegularConstants.UpperArmMax;
      double Low_ArmMax = RegularConstants.LowerArmMax;
      double Up_ArmMin = RegularConstants.UpperArmMin;
      double Low_ArmMin = RegularConstants.LowerArmMin;
      double Up_TempEncoderTicks = Up_ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      double Low_TempEncoderTicks = Low_ArmMotor.getSensorCollection().getIntegratedSensorPosition();  

      //Calculates angles based on last anglse and differences in encoder ticks  
      Up_angle = -(Up_TempEncoderTicks- Up_StartEncoderTicks) / Up_ArmRatio;
      Low_angle = -(Low_TempEncoderTicks- Low_StartEncoderTicks) / Low_ArmRatio;

      //finds distance from current angle to desired angle
      Up_AngleDif = Up_angle - UpperDesiredAngle;
      double Up_AngleDifAbsolute = Math.abs(Up_AngleDif);
      Up_AngleDif = Up_angle - UpperDesiredAngle;
      double Low_AngleDifAbsolute = Math.abs(Up_AngleDif);

      //Acounts for distances
      if(Up_AngleDifAbsolute > 15)
      {
        Up_tempOutput = Up_ArmMax;
      }
      else
      {
        if(Up_AngleDifAbsolute * Up_ArmMax /*Changed Up_ArmKp to Up_ArmMax, if it stops working try changing this back */ * Up_ArmKI < Up_ArmMin)
        {
          Up_tempOutput = Up_ArmMin;
          if(Up_AngleDifAbsolute == 0)
          {
            Up_tempOutput = 0;
          }
        }
        else
        {
          Up_tempOutput = Up_AngleDifAbsolute * Up_ArmMax /*This used Kp too */ * Up_ArmKI;
        }
      }

      if(Low_AngleDifAbsolute > 15)
      {
        Low_tempOutput = Low_ArmMax;
      }
      else
      {
        if(Low_AngleDifAbsolute * Low_ArmMax * Low_ArmKI < Low_ArmMin)
        {
          Low_tempOutput = Low_ArmMin;
          if(Low_AngleDifAbsolute == 0)
          {
            Low_tempOutput = 0;
          }
        }
        else
        {
          Low_tempOutput = Low_AngleDifAbsolute * Low_ArmMax * Low_ArmKI;
        }
      }

      //moves arm motors in the directions they should go in and if Motors should even move
      if(Up_AngleDifAbsolute > 1)
      {
       if((Up_AngleDif)>0)
       {
         Up_ArmMotor.set(TalonFXControlMode.PercentOutput, Up_tempOutput);
       }
       else if (Up_AngleDif < 0)
       {
         Up_ArmMotor.set(TalonFXControlMode.PercentOutput, -Up_tempOutput);
       }
       else
        {
          //yay
        }
      } 

      if(Low_AngleDifAbsolute>1)
      {
        if((Low_AngleDif)>0)
        {
          Low_ArmMotor.set(TalonFXControlMode.PercentOutput, Low_tempOutput);
        }
       else if (Low_AngleDif < 0)
        {
         Low_ArmMotor.set(TalonFXControlMode.PercentOutput, -Low_tempOutput);
        }
        else
       {
         // <3
       }
      }

      //determines if subsystem has done its job yet or not
      if(Up_AngleDifAbsolute <= 1 && Low_AngleDifAbsolute <= 1)
      {
        AtSetpoint = true;
      }
      else
      {
        AtSetpoint = false;
      }

      //Makes my variables beholdable
      SmartDashboard.putNumber("Up_encoder Ticks", Up_TempEncoderTicks);
      SmartDashboard.putNumber("Up_AngleDif", Up_AngleDif);
      SmartDashboard.putNumber("Up_Angle", Up_angle);
      SmartDashboard.putNumber("Low_encoder Ticks", Low_TempEncoderTicks);
      SmartDashboard.putNumber("Low_AngleDif", Low_AngleDif);
      SmartDashboard.putNumber("Low_Angle", Low_angle);
    }

    // zeros the current angle, will be used with limit switches
    public void Reset()
    {
      Up_angle = 0.0;
      Up_StartEncoderTicks = Up_ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      Low_angle = 0.0;
      Low_StartEncoderTicks = Low_ArmMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    //keeps the arm from never decelerating
    public void stop()
    {
      Up_ArmMotor.set(TalonFXControlMode.Disabled,0);
      Low_ArmMotor.set(TalonFXControlMode.Disabled,0);
    }
}
