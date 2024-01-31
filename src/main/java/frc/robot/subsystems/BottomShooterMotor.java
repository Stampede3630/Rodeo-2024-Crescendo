// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//import important commands
package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomShooterMotor extends SubsystemBase {
  private CANSparkFlex PlutoMotor;
  /** Creates a new BottomShooterMotor. */
  
  public BottomShooterMotor() {
    PlutoMotor = new CANSparkFlex(2, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command spin() {
    return startEnd(()->PlutoMotor.set(1), () -> {});
  }

  public Command stop(){
        return startEnd(()->PlutoMotor.set(0), () -> {});

  }

  public Command intake(){
       return startEnd(()->PlutoMotor.set(-0.4), () -> {});
  }
}