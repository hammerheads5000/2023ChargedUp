// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderCheck extends SubsystemBase {
  /** Creates a new EncoderCheck. */
  DigitalInput Lower_ArmBackwardsSwitch;
  DigitalInput Lower_ArmForwardsSwitch;
  DigitalInput Upper_MaxWhileForwardsSwitch;
  DigitalInput Upper_MaxWhileBackwardsSwitch;
  DigitalInput Upper_BringArmUpSafetySwitch;
  DigitalInput Upper_AtStowSwitch;

  public EncoderCheck
  (
    DigitalInput Lower_ArmBackwardsSwitch,
    DigitalInput Lower_ArmForwardsSwitch,
    DigitalInput Upper_MaxWhileForwardsSwitch,
    DigitalInput Upper_MaxWhileBackwardsSwitch,
    DigitalInput Upper_BringArmUpSafetySwitch,
    DigitalInput Upper_AtStowSwitch
  ) 
  {
    this.Lower_ArmBackwardsSwitch = Lower_ArmBackwardsSwitch;
    this. Lower_ArmForwardsSwitch = Lower_ArmForwardsSwitch;
    this.Upper_MaxWhileForwardsSwitch = Upper_MaxWhileForwardsSwitch;
    this.Upper_MaxWhileBackwardsSwitch = Upper_MaxWhileBackwardsSwitch;
    this.Upper_BringArmUpSafetySwitch = Upper_BringArmUpSafetySwitch;
    this.Upper_AtStowSwitch = Upper_AtStowSwitch;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
  public int CheckEncoders(boolean IsUpperArm)
    {
      SmartDashboard.putBoolean("LowMax", Upper_MaxWhileBackwardsSwitch.get());
      SmartDashboard.putBoolean("UpMax", Upper_MaxWhileForwardsSwitch.get());
      SmartDashboard.putBoolean("AtPlayer", Upper_BringArmUpSafetySwitch.get());
      SmartDashboard.putBoolean("Stow", Upper_AtStowSwitch.get());
      SmartDashboard.putBoolean("LowerBack", Lower_ArmBackwardsSwitch.get());
      SmartDashboard.putBoolean("LowerForward", Lower_ArmForwardsSwitch.get());
      //safety check for upper arm
      if(IsUpperArm)
      {
        if((Lower_ArmBackwardsSwitch.get()&&Upper_MaxWhileBackwardsSwitch.get())||(Lower_ArmForwardsSwitch.get()&&Upper_MaxWhileForwardsSwitch.get()))
        {
          return 1; //makes upper arm go down 
        }
        else if(Upper_BringArmUpSafetySwitch.get())
        {
          return 2; //slows upper arm way down
        }
        else if(Upper_AtStowSwitch.get())
        {
          return 3; //stops upper arm 
        }
        else
        {
          return 4; //allows upper arm to move at requested speed
        }

      }
      else
      {
        if((Lower_ArmForwardsSwitch.get() == true) && (!Upper_BringArmUpSafetySwitch.get()))
        {
          return 11;
        }
        else if(Lower_ArmBackwardsSwitch.get())
        {
          SmartDashboard.putNumber("return", 12);
          return 12; //stops lower arm
        }
        SmartDashboard.putNumber("return", 13);
        return 13;
      }
    }
}
