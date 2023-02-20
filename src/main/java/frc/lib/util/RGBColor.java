// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

/** Add your docs here. */
public class RGBColor {
    private int red;
    private int green;
    private int blue;

    public RGBColor(int red, int green, int blue){
        this.red = red;
        this.green = green;
        this.blue = blue;
    }
    public int getRed(){
        return this.red;
    }
    public int getGreen(){
        return this.green;
    }
    public int getBlue(){
        return this.blue;
    }
}
