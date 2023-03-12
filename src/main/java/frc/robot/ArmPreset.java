// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;

public class ArmPreset {
    private double angle;
    private boolean lowerArmUp;

    public ArmPreset(double angle, boolean lowerArmUp) {
        this.angle = angle;
        this.lowerArmUp = lowerArmUp;
    }

    public double getAngle() {
        return angle;
    }

    public boolean getLowerArmUp() {
        return lowerArmUp;
    }

    @Override
    public boolean equals(Object otherObject) {
        ArmPreset otherPreset = (ArmPreset)otherObject;
        return (Math.abs(this.getAngle() - otherPreset.getAngle()) < ArmConstants.presetToleranceDegrees) 
                && (this.getLowerArmUp() == otherPreset.getLowerArmUp());
    }
}
