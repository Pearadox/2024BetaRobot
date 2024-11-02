// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SmarterDashboard;

public class LEDStrip extends SubsystemBase {
   private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private final int numLEDs;
  private int trail_size = 60;
  private ArrayList<String> defaultColors = new ArrayList<>();
  private ArrayList<String> rainColors = new ArrayList<>();
  private boolean rainbowOn = false;
  private double lastShiftedDefault;

  private enum LEDMode {
    kDefault, kCone, kCube, kRainbow
  }

  private LEDMode mode = LEDMode.kDefault;

  public LEDStrip(int numberOfLeds, int port) {
    led = new AddressableLED(port);
    numLEDs = numberOfLeds;
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(numLEDs);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();

    lastShiftedDefault = Timer.getFPGATimestamp();
    loadDefaultColors();
    loadRainbowColors();
  }

  public void loadDefaultColors() {
    for (int i = 0; i < ledBuffer.getLength(); i += 10) {
      if (i == 100) {
        defaultColors.add("Y1");
        defaultColors.add("Y1");
        defaultColors.add("Y2");
        defaultColors.add("Y2");
        defaultColors.add("M");
        defaultColors.add("M");
        defaultColors.add("G2");
        defaultColors.add("G2");
        defaultColors.add("G1");
        defaultColors.add("G1");
      } else if (i % 20 == 0) {
        if (i % 40 == 0) {
          for (int n = 0; n < 10; n++) {
            defaultColors.add("G");
          }
        } else {
          for (int n = 0; n < 10; n++) {
            defaultColors.add("Y");
          }
        }
      } else {
        if ((i + 10) % 40 == 0) {
          defaultColors.add("Y1");
          defaultColors.add("Y1");
          defaultColors.add("Y2");
          defaultColors.add("Y2");
          defaultColors.add("M");
          defaultColors.add("M");
          defaultColors.add("G2");
          defaultColors.add("G2");
          defaultColors.add("G1");
          defaultColors.add("G1");
        } else {
          defaultColors.add("G1");
          defaultColors.add("G1");
          defaultColors.add("G2");
          defaultColors.add("G2");
          defaultColors.add("M");
          defaultColors.add("M");
          defaultColors.add("Y2");
          defaultColors.add("Y2");
          defaultColors.add("Y1");
          defaultColors.add("Y1");
        }
        // for(int n = 0; n < 5; n++){
        // defaultColors.add("O");
        // }
      }
    }
  }

  public void loadRainbowColors() {
    int change = 0;
    int color_size = trail_size / 6;
    for (int i = 0; i <= trail_size; i += color_size) {
      for (int j = 0; j < color_size; j++) {
        switch (change) {
          case 0:
            rainColors.add("R");
            break;
          case 1:
            rainColors.add("O");
            break;
          case 2:
            rainColors.add("Y");
            break;
          case 3:
            rainColors.add("G");
            break;
          case 4:
            rainColors.add("B");
            break;
          case 5:
            rainColors.add("P");
            break;
        }
      }
      change++;
    }
    for (int i = 0; i < ledBuffer.getLength()-rainColors.size()+1; i++) {
      rainColors.add("P");
    }
  }

  public void shiftDefaultColors() {
    if(!rainbowOn){
      String temp = defaultColors.get(defaultColors.size() - 1);
      defaultColors.add(0, temp);
      defaultColors.remove(defaultColors.size() - 1);
    }else{
      String temp2 = rainColors.get(rainColors.size() - 1);
      rainColors.add(0, temp2);
      rainColors.remove(rainColors.size() - 1);
    }
  }

  public void animateRainbow() {
    String last = "ksaasda";
    int inc = 0;
    int color_size = trail_size / 6;
    int color_inc = 127 / color_size;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (last == rainColors.get(i)){
        inc++;
      }else{
        inc = 0;
      }
      switch (rainColors.get(i)) {
        case "R":
          ledBuffer.setRGB(i, 255, 0 + (inc * color_inc), 0);
          last = "R";
          break;
        case "O":
          ledBuffer.setRGB(i, 255, 127 + (inc * color_inc), 0);
          last = "O";
          break;
        case "Y":
          ledBuffer.setRGB(i, 255 - (inc * color_inc * 2), 255, 0);
          last = "Y";
          break;
        case "G":
          ledBuffer.setRGB(i, 0, 255 - (inc * color_inc * 2), 0 + (inc * color_inc * 2));
          last = "G";
          break;
        case "B":
          ledBuffer.setRGB(i, 0 + (inc * color_inc * 2), 0, 255);
          last = "B";
          break;
        case "P":
          ledBuffer.setRGB(i, 255, 0, 255 - (inc * color_inc * 2));
          last = "P";
          break;
        case "N":
          ledBuffer.setRGB(i, 1, 2, 3);
        default:
          ledBuffer.setRGB(i, 50, 50, 50);
      }
    }
  }

  public void animateIdle() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      switch (defaultColors.get(i)) {
        case "G":
          ledBuffer.setRGB(i, 0, 215, 0);
          break;
        case "Y":
          ledBuffer.setRGB(i, 215, 80, 0);
          break;
        case "G1":
          ledBuffer.setRGB(i, 3, 192, 0);
          break;
        case "G2":
          ledBuffer.setRGB(i, 45, 170, 0);
          break;
        case "M":
          ledBuffer.setRGB(i, 88, 147, 0);
          break;
        case "Y2":
          ledBuffer.setRGB(i, 130, 125, 0);
          break;
        case "Y1":
          ledBuffer.setRGB(i, 173, 102, 0);
          break;
        default:
          ledBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  public ArrayList<String> getDefaultColors() {
    return defaultColors;
  }

  public void setDefaultMode() {
    mode = LEDMode.kDefault;
  }

  public void setConeMode() {
    mode = LEDMode.kCone;
  }

  public void setCubeMode() {
    mode = LEDMode.kCube;
  }

  public void setRainbowMode() {
    mode = LEDMode.kRainbow;
  }

  public boolean isRainbowMode(){
    return mode == LEDMode.kRainbow;
  }

  public void setBottomColor(int r, int g, int b) {
    for (int i = 0; i < 25; i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    for(int i = 79; i < 110; i++){
      ledBuffer.setRGB(i, r, g, b);
    }
  }

  public void setTopColor(int r, int g, int b){
    for(int i = 25; i < 79; i++){
      ledBuffer.setRGB(i,r, g, b);
    }
  }

  public void ledHold() {
    if(mode == LEDMode.kRainbow){
      animateRainbow();
      rainbowOn = true;      
    }
     else {
      animateIdle();
      rainbowOn = false;  

     }
  }

  @Override
  public void periodic() {

    if (Timer.getFPGATimestamp() - lastShiftedDefault > 0.01) {
      shiftDefaultColors();
      lastShiftedDefault = Timer.getFPGATimestamp();
    }

    ledHold();
    led.setData(ledBuffer);
    
    SmarterDashboard.putString("LED Mode", mode.toString(), "LEDStrip");
   }
}
