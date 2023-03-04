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
  boolean lower_ArmBackwardsSwitch;
  boolean lower_ArmForwardsSwitch;
  boolean upper_MaxWhileForwardsSwitch;
  boolean upper_MaxWhileBackwardsSwitch;
  boolean upper_BringArmUpSafetySwitch;
  boolean upper_AtStowSwitch;


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

    lower_ArmBackwardsSwitch = Lower_ArmBackwardsSwitch.get();
    lower_ArmForwardsSwitch = Lower_ArmForwardsSwitch.get();
    upper_MaxWhileForwardsSwitch = Upper_MaxWhileForwardsSwitch.get();
    upper_MaxWhileBackwardsSwitch = Upper_MaxWhileBackwardsSwitch.get();
    upper_BringArmUpSafetySwitch = Upper_BringArmUpSafetySwitch.get();
    upper_AtStowSwitch = Upper_AtStowSwitch.get();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
  public int CheckEncoders(boolean IsUpperArm)
    {
      SmartDashboard.putBoolean("LowMax", upper_MaxWhileBackwardsSwitch);
      SmartDashboard.putBoolean("UpMax", upper_MaxWhileForwardsSwitch);
      SmartDashboard.putBoolean("AtPlayer", upper_BringArmUpSafetySwitch);
      SmartDashboard.putBoolean("Stow", upper_AtStowSwitch);
      SmartDashboard.putBoolean("LowerBack", lower_ArmBackwardsSwitch);
      SmartDashboard.putBoolean("LowerForward", lower_ArmForwardsSwitch);
      //safety check for upper arm
      if(IsUpperArm)
      {
        if((lower_ArmBackwardsSwitch && upper_MaxWhileBackwardsSwitch)||(lower_ArmForwardsSwitch&&upper_MaxWhileForwardsSwitch))
        {
          return 1; //makes upper arm go down 
        }
        else if(upper_BringArmUpSafetySwitch)
        {
          return 2; //slows upper arm way down
        }
        else if(upper_AtStowSwitch)
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
        if((lower_ArmForwardsSwitch == true) && (!upper_BringArmUpSafetySwitch))
        {
          return 11;
        }
        else if(lower_ArmBackwardsSwitch)
        {
          return 12; //stops lower arm
        }
        return 13;
      }
    }
}
