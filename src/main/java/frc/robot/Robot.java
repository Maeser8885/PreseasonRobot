/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private static final int leftDeviceID = 0; 
  private static final int rightDeviceID = 1;
  //private CANSparkMax m_leftMotor;
  //private CANSparkMax m_rightMotor;
  private Spark m_leftMotor1;
  private Spark m_leftMotor2;
  private Spark m_rightMotor1;
  private Spark m_rightMotor2;
  private XboxController xboxController;
  private ColorSensorV3 m_coColorSensorV3;
  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is pa%SparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    //m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless);
    //m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);

    m_leftMotor1 = new Spark(1);
    //m_leftMotor2 = new Spark(0);
    m_rightMotor1 = new Spark(0);
    m_leftMotor1.setInverted(true);
    //m_rightMotor2 = new Spark(1);
    UsbCamera frontCamera = CameraServer.startAutomaticCapture("front",0);
    //frontCamera.setResolution(320, 240);
    CvSink cvSink = CameraServer.getVideo();

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    //m_leftMotor.restoreFactoryDefaults();
    //m_rightMotor.restoreFactoryDefaults();

    //m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
    m_coColorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);

    xboxController = new XboxController(0);

  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(xboxController.getLeftY(), xboxController.getRightY());
    Color detectedColor = m_coColorSensorV3.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Color", m_coColorSensorV3.getColor().getClass().getName());
  }
}
