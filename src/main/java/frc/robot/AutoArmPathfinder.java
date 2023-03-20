// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.ArmConstants;
import  frc.robot.subsystems.UpperArmManual;
/** Add your docs here. */
public class AutoArmPathfinder 
{
    public ArmPreset Pathfind(ArmPreset desired, ArmPreset current)
    {
        if (desired.getLowerArmUp() == current.getLowerArmUp()) {
            return desired;
        }
        else if (desired.getLowerArmUp()) {
            if (current.getAngle() > ArmConstants.MaxAngleWhileUp) {
                return new ArmPreset(ArmConstants.MaxAngleWhileUp, current.getLowerArmUp());
            }
            else {
                return new ArmPreset(current.getAngle(), true);
            }
        }
        else {
            if(current.getAngle() < ArmConstants.MinAngleWhileDown){
                return new ArmPreset(ArmConstants.MinAngleWhileDown, current.getLowerArmUp());
            }
            else {
                return new ArmPreset(current.getAngle(), false);
            }
        }
    }
}
