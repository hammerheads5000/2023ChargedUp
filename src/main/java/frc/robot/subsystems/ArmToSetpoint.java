
package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Constants.RegularConstants;
import frc.robot.Constants.RegularConstants;
public class ArmToSetpoint extends SubsystemBase {

  /*Behold, My variables */
   TalonFX ArmMotor = new TalonFX(3,"Bobby");
   public double angle = 0.0;
   public double StartEncoderTicks;
   public double AngleDif;

    /* Creates a new ArmToSetpoint subsystem */
    public ArmToSetpoint () 
    {
      ArmMotor.setNeutralMode(NeutralMode.Brake);
      //sets the initil position of the arm segment
      StartEncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    public void moveUp(double speed)
    {
      ArmMotor.set(TalonFXControlMode.PercentOutput, speed); 
    }

    public void moveDown(double speed)
    {
      ArmMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }
    
    //Moves arm to a desired angle
    public void MoveArm (double DesiredAngle, boolean PID)
    {
    double ArmKI = RegularConstants.UpperArmKI;
    double ArmRatio = RegularConstants.UpperArmRatio;
    double tempOutput;
    double ArmMax = RegularConstants.UpperArmMax;
    double ArmMin = RegularConstants.UpperArmMin;
    double TempEncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
      
      //Calculates angle based on last angle and difference in encoder ticks  
      angle = -(TempEncoderTicks- StartEncoderTicks) / ArmRatio;

      //finds distance from current angle to desired angle
      AngleDif = angle - DesiredAngle;
      double AngleDifAbsolute = Math.abs(AngleDif);

      if(PID == true)
      {
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
      }
      else
      {
        tempOutput = ArmMin;
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
    public void Reset()
    {
      AngleDif = 0.0;
    }

    //keeps the arm from never decelerating
    public void stop()
    {
      ArmMotor.set(TalonFXControlMode.Disabled,0);
  }
}
