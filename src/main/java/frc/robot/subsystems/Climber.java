// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  TalonFX climberMotor = new TalonFX(6);
  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Command extend() {
    return startEnd(()->climberMotor.set(0.5),()->{});
  }
  public Command stop() {
    return startEnd(()->climberMotor.set(0),()->{});
  }
  public Command retract() {
    return startEnd(()->climberMotor.set(-0.5),()->{});
  }
}
