// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveBaseConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.drivebase.SwerveDrive;


public class RobotContainer {
  
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;

  public RobotContainer() {
   
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
  
   

    configureBindings();
  }

  private void configureBindings() {
     swerveDrive.setDefaultCommand(new SwerveJoystickCmd(swerveDrive, mainController));
     mainController.rightBumper()
     .onTrue(Commands.runOnce(() -> swerveDrive
             .setMagnification(DriveBaseConstants.kHighMagnification)));
mainController.leftBumper()
     .onTrue(Commands.runOnce(() -> swerveDrive
             .setMagnification(DriveBaseConstants.kDefaultMagnification)));
      
  }

  public Command getAutonomousCommand() {

    return Commands.print("No autonomous command configured");
  }
}
