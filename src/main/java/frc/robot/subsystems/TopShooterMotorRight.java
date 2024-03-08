// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//import important commands
package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TopShooterMotorRight extends SubsystemBase {
  private CANSparkFlex MinnieMotor;
  /** Creates a new TopShooterMotor. */
  public TopShooterMotorRight() {
    MinnieMotor = new CANSparkFlex(1, MotorType.kBrushless);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // -1 so it spins the right way
  public Command spin() {
    return startEnd(()->MinnieMotor.set(-1), ()->{});
  }
  
  public Command ampSpin() {
    return startEnd(()->MinnieMotor.set(-0.1), ()->{});
  }

  public Command stop(){
        return startEnd(()->MinnieMotor.set(0), ()->{});


  }

  public Command intake(){
        return startEnd(()->MinnieMotor.set(0.8), ()->{});

  }
}
