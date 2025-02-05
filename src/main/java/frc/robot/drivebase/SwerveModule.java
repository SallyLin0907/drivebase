// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivebase;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;

  private final CANcoder turningEncoder;
  private final RelativeEncoder driveEncoder;

 
  private final PIDController rotController;



  private final String name;

 

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel,
      boolean driveInverted, double chassisAngularOffset, String name) {

    this.name = name;

    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = new CANcoder(turningEncoderChannel);

    CANcoderConfiguration turningEncoderConfiguration = new CANcoderConfiguration();
    turningEncoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.5);
    turningEncoderConfiguration.MagnetSensor.MagnetOffset = chassisAngularOffset;
    turningEncoder.getConfigurator().apply(turningEncoderConfiguration);

    rotController = new PIDController(0.50, 0, 0);
    rotController.enableContinuousInput(-180, 180);
    init();

  }



  public void init() {
    configDriveMotor();
    configTurningMotor();
    resetEncoders();
  }

  public void configDriveMotor() {
    SparkMaxConfig configdriveMotor = new SparkMaxConfig();
    configdriveMotor.idleMode(IdleMode.kBrake);
    configdriveMotor.smartCurrentLimit(10, 80);
    configdriveMotor.closedLoopRampRate(ModuleConstants.kDriveClosedLoopRampRate);
    configdriveMotor.voltageCompensation(ModuleConstants.kMaxModuleDriveVoltage);
    configdriveMotor.signals.primaryEncoderPositionPeriodMs(10);
    configdriveMotor.signals.primaryEncoderVelocityPeriodMs(20);
    driveMotor.configure(configdriveMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configTurningMotor() {
    SparkMaxConfig configturningMotor = new SparkMaxConfig();
    configturningMotor.smartCurrentLimit(20);
    configturningMotor.closedLoopRampRate(ModuleConstants.kTurningClosedLoopRampRate);
    configturningMotor.idleMode(IdleMode.kBrake);
    configturningMotor.voltageCompensation(ModuleConstants.kMaxModuleTurningVoltage);
    turningMotor.configure(configturningMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

 

  public void stopModule() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // to get the single swerveModule speed and the turning rate
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveRate(), new Rotation2d(Math.toRadians(getRotation())));
  }

  // to get the drive distance
  public double getDriveDistance() {
    return (driveEncoder.getPosition() / 6.75) * (2.0 * Math.PI * ModuleConstants.kWheelRadius);

  }

  // calculate the rate of the drive
  public double getDriveRate() {
    return driveEncoder.getVelocity() * 1 / 60.0 / 6.75 * 2.0 * Math.PI
        * ModuleConstants.kWheelRadius;
  }

  // to get rotation of turning motor
  public double getRotation() {
    return turningEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
  }

  // to the get the postion by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveDistance(), new Rotation2d(Math.toRadians(getRotation())));
  }

  public double[] optimizeOutputVoltage(SwerveModuleState goalState, double currentTurningDegree) {
    // 建立目標 SwerveModuleState
    SwerveModuleState desiredState = new SwerveModuleState(
        goalState.speedMetersPerSecond,
        new Rotation2d(Math.toRadians(currentTurningDegree)));

    // 直接對 `desiredState` 進行最佳化
    desiredState.optimize(new Rotation2d(Math.toRadians(currentTurningDegree)));

    // 計算馬達電壓
    double driveMotorVoltage = desiredState.speedMetersPerSecond * ModuleConstants.kDesireSpeedtoMotorVoltage;
    double turningMotorVoltage = rotController.calculate(currentTurningDegree, desiredState.angle.getDegrees());

    return new double[] { driveMotorVoltage, turningMotorVoltage };
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    var moduleState = optimizeOutputVoltage(desiredState, getRotation());
    driveMotor.setVoltage(moduleState[0]);
    turningMotor.setVoltage(moduleState[1]);
    SmartDashboard.putNumber(name + "_voltage", moduleState[0]);
  }

  public Command setDesiredStateCmd(SwerveModuleState state) {
    Command cmd = runOnce(() -> setDesiredState(state));
    cmd.setName("SetDesiredStateCmd");
    return cmd;
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public void resetTurningDegree() {
    double turningDegreeVoltage = rotController.calculate(getRotation(), 0);
    turningMotor.setVoltage(turningDegreeVoltage);
  }

  public void setTurningDegree90() {
    double turningDegreeTo90Voltage = rotController.calculate(getRotation(), 90);
    turningMotor.setVoltage(turningDegreeTo90Voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(name + "_ModuleDistance", getDriveDistance());
    SmartDashboard.putNumber(name + "_ModuleVelocity", getDriveRate());
    SmartDashboard.putNumber(name + "_ModuleRotation", getRotation());
    // SmartDashboard.putData(name + "_rotController", rotController);
  }
}