// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase {
  TalonFX ampMotor = new TalonFX(11);
  /** Creates a new Amp. */
  private final Debouncer currentDebouncer = new Debouncer(.1);
  public Amp() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public boolean
  calcCurrentDebouncer() {
    return currentDebouncer.calculate(ampMotor.getTorqueCurrent().getValueAsDouble()>30);
  }
  public Command getNote() {
    return startEnd(()->ampMotor.set(0.5),()->{}).until(()-> calcCurrentDebouncer());
}
  public Command stop() {
    return startEnd(()->ampMotor.set(0),()->{});
  }
  public Command scoreNote() {
    return startEnd(()->ampMotor.set(-0.6),()->{});
  }
}