// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallStorage extends SubsystemBase {
  //global variables
  private DigitalInput limitSwitch;

  /** Creates a new BallStorage. */
  public BallStorage() {
    //constructor
    //put code that runs when the ball storage is created
    limitSwitch = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean limitswitch(){
    return limitSwitch.get();
  }
}
