// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GameSpec.Arm;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GameSpec.Arm.ArmIO.ArmIOStats;

public class Arm extends SubsystemBase {
  public FunctionalCommand testCommand;

  private final ArmIO io;
  public final ArmIOStats stats;
  private final ShuffleboardTab armShuffleboard;

  /* Shuffleboard entrys */
  public GenericEntry armPosition;
  public GenericEntry armDirection;
  private GenericEntry armVelocity;
  private GenericEntry armSupplyCurrent;
  private GenericEntry armStatorCurrent;
  private GenericEntry armTemp;

  public double armCommandedPos;
  
  public Arm(ArmIO io) {
    this.io = io; 

    this.stats = ArmIO.stats;

    this.armShuffleboard = Shuffleboard.getTab("Arm");

    armVelocity = this.armShuffleboard.add("Arm RPM", 0.0).getEntry();
    armPosition = this.armShuffleboard.add("Arm Position", 0.0).getEntry();;
    armSupplyCurrent = this.armShuffleboard.add("Arm Supply Current", 0.0).getEntry();
    armStatorCurrent = this.armShuffleboard.add("Arm Stator Current", 0.0).getEntry();
    armTemp = this.armShuffleboard.add("Arm Temp", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    io.updateStats();
      
    UpdateTelemetry();
  }

  private void UpdateTelemetry() {
    armVelocity.setDouble(stats.armVelocity);
    armPosition.setDouble(stats.armPosition);
    armSupplyCurrent.setDouble(stats.SupplyCurrentAmps);
    armStatorCurrent.setDouble(stats.TorqueCurrentAmps);
    armTemp.setDouble(stats.TempCelsius);
  }

  public Command runToPosition(double position){
    return run(() -> {
        io.setArmMotorControl(position);
    });
  }

  public Command stop(){
    return run(() -> {
        io.stop();
    });  
  }

  @Override
  public void simulationPeriodic() {
    io.periodic();
  }
}