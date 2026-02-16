// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */


public class RobotContainer {
  //controller
  private final inputMove inputmove;
  private final Shooter shooter;
  private final Higher higher;
  private final outputMove outputmove;
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  //private final TankDrive drive = new TankDrive();
//commands
  private final TankDrive drive = new TankDrive(() -> operator.getLeftY(),() -> operator.getRightX());
  //subsystem
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  public RobotContainer() {
    shooter = new Shooter();
    inputmove = new inputMove(shooter);
    outputmove = new outputMove(shooter);
    higher = new Higher(shooter);
    configureBindings();
    
      }
    
    
      private void configureBindings() {
        operator.leftTrigger().whileTrue(inputmove);
        operator.rightTrigger().whileTrue(outputmove);
        operator.leftBumper().toggleOnTrue(higher); 
        //drive.move(operator.getLeftY(), operator.getRightX());
        
}
}