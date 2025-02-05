// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.drivebase;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.Constants.DriveBaseConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SparkMax driveMotor;
  private final SparkMax turningMotor;

  private final SparkAbsoluteEncoder turningEncoder;
  private final RelativeEncoder driveEncoder;

  private final PIDController driveController;
  private final PIDController rotController;



  public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
  public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

  private double m_chassisAngularOffset = 0;

  private final String name;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(int driveMotorChannel, int turningMotorChannel,
      boolean driveInverted, double chassisAngularOffset, String name) {
    this.name = name;

    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getAbsoluteEncoder();

    driveController = new PIDController(0.50, 0, 0);
    rotController = new PIDController(0.50, 0, 0);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(turningEncoder.getPosition());

    driveEncoder.setPosition(0);
    
    rotController.enableContinuousInput(-180, 180);
    init();

  }

  // public boolean isInverted(){
  // boolean isInverted = driveMotor.configAccessor.getInverted();
  // return isInverted;
  // }

  public void init() {
  configDriveMotor();
  configTurningMotor();
  resetAllEncoder();
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

  public void resetAllEncoder() {
    driveEncoder.setPosition(0);
  }

  public void stopModule() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  // to get the single swerveModule speed and the turning rate
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - m_chassisAngularOffset));
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
    return turningEncoder.getPosition() * 360.0;
  }

  // to the get the postion by wpi function
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - m_chassisAngularOffset));

  }

  // public double[] optimizeOutputVoltage(SwerveModuleState goalState, double
  // currentTurningDegree) {
  // SwerveModuleState desiredState = new SwerveModuleState(
  // goalState.speedMetersPerSecond,
  // new Rotation2d(Math.toRadians(currentTurningDegree)));
  // desiredState.optimize(new Rotation2d(Math.toRadians(currentTurningDegree)));
  // double driveMotorVoltage = desiredState.speedMetersPerSecond *
  // ModuleConstants.kDesireSpeedtoMotorVoltage;
  // double turningMotorVoltage = rotController.calculate(currentTurningDegree,
  // desiredState.angle.getDegrees());

  // return new double[] { driveMotorVoltage, turningMotorVoltage };
  // }

  public double[] optimizeOutputVoltage(SwerveModuleState goalState, double currentTurningDegree) {
    // 建立目標 SwerveModuleState
    SwerveModuleState desiredState = new SwerveModuleState(
        goalState.speedMetersPerSecond, 
        new Rotation2d(Math.toRadians(currentTurningDegree))
    );

    // 直接對 `desiredState` 進行最佳化
    desiredState.optimize(new Rotation2d(Math.toRadians(currentTurningDegree)));

    // 計算馬達電壓
    double driveMotorVoltage = desiredState.speedMetersPerSecond * ModuleConstants.kDesireSpeedtoMotorVoltage;
    double turningMotorVoltage = rotController.calculate(currentTurningDegree, desiredState.angle.getDegrees());

    return new double[]{driveMotorVoltage, turningMotorVoltage};
}

  

 public void setDesiredState(SwerveModuleState desiredState) {
      var moduleState = optimizeOutputVoltage(desiredState, getRotation());
      driveMotor.setVoltage(moduleState[0]);
      turningMotor.setVoltage(moduleState[1]);
      SmartDashboard.putNumber(name + "_voltage", moduleState[0]);
  }

  public Command setDesiredStateCmd(SwerveModuleState state){
    Command cmd = runOnce(()->setDesiredState(state));
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