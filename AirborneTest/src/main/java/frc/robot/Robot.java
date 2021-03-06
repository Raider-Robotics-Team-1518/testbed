/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.LimeLight;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 5;
  private static final int kRearLeftChannel = 6;
  private static final int kFrontRightChannel = 10;
  private static final int kRearRightChannel = 9;
  private static final int kThrowerMotor = 4;

  private static final int kJoystickChannel = 0;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0, 1, 1);
  private final Color kGreenTarget = ColorMatch.makeColor(0, 1, 0);
  private final Color kRedTarget = ColorMatch.makeColor(1, 0, 0);
  private final Color kYellowTarget = ColorMatch.makeColor(1, 1, 0);

  private CANEncoder m_encoder;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private LimeLight lime = new LimeLight();

  public static boolean DriveByStick = false;
  public JoystickButton btnDriveMode;

  private static final double targetHeight = 18; // Inches
  private static final double optimalDistance = 84; // Inches

  public static Gyro rioGyro = new ADXRS450_Gyro();

  private CANSparkMax throwerMotor1;

  @Override
  public void robotInit() {

    btnDriveMode = new JoystickButton(m_stick, 11);
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    throwerMotor1 = new com.revrobotics.CANSparkMax(kThrowerMotor, MotorType.kBrushless);

    throwerMotor1.restoreFactoryDefaults();

    m_encoder = throwerMotor1.getEncoder();

    rioGyro.calibrate();
    rioGyro.reset();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    // frontLeft.setInverted(true);
    // rearLeft.setInverted(true);
    // frontRight.setInverted(true);
    // rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickChannel); 

    lime.init("limelight");
  }

  @Override
  public void teleopPeriodic() {

    // Thrower Motor by the Throttle
    throwerMotor1.set(m_stick.getThrottle());
    SmartDashboard.putNumber("Thrower Throttle", m_stick.getThrottle());
    // Use button 11 to change drive mode (joystick or LimeLight)
    // btnDriveMode.whenPressed(new ChangeDrive());

    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    // SmartDashboard.putNumber("Gyro Angle", rioGyro.getAngle());

    SmartDashboard.putNumber("Thrower Velocity", m_encoder.getVelocity());

    if (true || DriveByStick) {
      m_robotDrive.driveCartesian(Math.pow(m_stick.getX(), 3) * .75, Math.pow(-m_stick.getY(), 3) * 1,
          Math.pow(m_stick.getZ(), 3) * .5, 0.0);
    } else {
      Double horizontalOffset = lime.getTargetOffsetHorizontal();
      Double verticalOffset = lime.getTargetOffsetVertical();
      Double targetArea = lime.getTargetArea();

      Double minPowerFloor = 0.25;
      Double newX = 0.0;
      Double newY = 0.0;
      Double newZ = 0.0;
      Double targetDistance = optimalDistance;

      // Compute the left or right power to turn the robot to face the target
      if (Math.abs(horizontalOffset) > 8.0) {
        // newZ = minPowerFloor * (int)Math.signum(horizontalOffset);
        newZ = minPowerFloor * (int) Math.signum(horizontalOffset) + Math.sin(horizontalOffset * Math.PI / 180);
      } else {
        newZ = 0.0;
      }

      // Compute the distance to the target based on height and power to move
      // forward/backward
      if (Math.abs(verticalOffset) > 0) {
        targetDistance = targetHeight / Math.tan(verticalOffset * Math.PI / 180);
      } else {
        targetDistance = optimalDistance;
      }
      SmartDashboard.putNumber("targetDistance", targetDistance);
      if (Math.abs(optimalDistance - targetDistance) >= 6) {
        if (targetDistance > optimalDistance) {
          newX = minPowerFloor;
        } else if (targetDistance < optimalDistance) {
          newX = -minPowerFloor;
        }
      } else {
        newX = 0.0;
      }
      SmartDashboard.putNumber("newX", newX);
      SmartDashboard.putNumber("newZ", newZ);
      m_robotDrive.driveCartesian(newY, newX, newZ, 0.0);
    }
  }

}
