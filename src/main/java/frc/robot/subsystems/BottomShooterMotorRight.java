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

public class BottomShooterMotorRight extends SubsystemBase {
  private TalonFX GoofyMotor;
  /** Creates a new BottomShooterMotor. */
  
  public BottomShooterMotorRight() {
    GoofyMotor = new TalonFX(19, "rio");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // -1 so it spins the right way
  public Command spin() {
    return startEnd(()->GoofyMotor.set(-0.5), () -> {});
  }

  public Command ampSpin() {
    return startEnd(()->GoofyMotor.set(-0.3), ()->{});
  }

  public Command stop(){
        return startEnd(()->GoofyMotor.set(0), () -> {});

  }

  public Command intake(){
       return startEnd(()->GoofyMotor.set(0.4), () -> {});
  }
}