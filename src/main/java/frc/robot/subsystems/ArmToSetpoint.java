// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ArmToSetpoint extends SubsystemBase {

   TalonFX ArmMotor = new TalonFX(6, "Bobby");
   public double angle = 0.0;
   public double EncoderTicks;

    /* Creates a new Pneumatics subsystem */
    public ArmToSetpoint () 
    {
      EncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    public void MoveArm (double DesiredAngle)
    {
      angle += (ArmMotor.getSensorCollection().getIntegratedSensorPosition() - EncoderTicks)/ 248000*360;
      if(Math.abs(DesiredAngle-angle)>1)
      {
        if((DesiredAngle-angle)>0)
        {
          ArmMotor.set(TalonFXControlMode.PercentOutput, Constants.RegularConstants.ArmKP);
        }
        else
        {
          ArmMotor.set(TalonFXControlMode.PercentOutput, -Constants.RegularConstants.ArmKP);
        }

      }
      EncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    public void Reset()
    {
      angle = 0.0;
      EncoderTicks = ArmMotor.getSensorCollection().getIntegratedSensorPosition();
    }
//

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */


}