// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//import imporms;
package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomShooterMotorLeft extends SubsystemBase {
  private TalonFX PlutoMotor;
  /** Creates a new BottomShooterMotor. */
  
  public BottomShooterMotorLeft() {
    PlutoMotor = new TalonFX(17,"rio");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command spin() {
    return startEnd(()->PlutoMotor.set(1), () -> {});
  }
  
  public Command ampSpin() {
    return startEnd(()->PlutoMotor.set(0.2), ()->{});
  }

  public Command stop(){
        return startEnd(()->PlutoMotor.set(0), () -> {});

  }

  public Command intake(){
       return startEnd(()->PlutoMotor.set(-0.4), () -> {});
  }
}