/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 2;
  private static final int kFrontRightChannel = 3;
  private static final int kRearRightChannel = 4;
  private static final int kThrowerMotor = 11;
  // private static final int kThrowerMotor2 = 2;

  private static final int kJoystickChannel = 0;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.129, .428, .443);
  private final Color kGreenTarget = ColorMatch.makeColor(.172, .577, .251);
  private final Color kRedTarget = ColorMatch.makeColor(.519, .347, .133);
  private final Color kYellowTarget = ColorMatch.makeColor(.319, .558, .124);

  private final WPI_TalonFX frontLeft = new WPI_TalonFX(kFrontLeftChannel);
  private final WPI_TalonFX rearLeft = new WPI_TalonFX(kRearLeftChannel);
  private final WPI_TalonFX frontRight = new WPI_TalonFX(kFrontRightChannel);
  private final WPI_TalonFX rearRight = new WPI_TalonFX(kRearRightChannel);

  private CANEncoder m_encoder;
  // private CANEncoder m_encoder2;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private LimeLight lime = new LimeLight();

  public static boolean DriveByStick = true;
  public JoystickButton btnDriveMode;

  private static final double targetHeight = 18; // Inches
  private static final double optimalDistance = 84; // Inches

  private static final double clicksPerMotorRotation = 2048.0;
  private static final double gearRatio = 10.71;
  private static final double clicksPerWheelRotation = 21934.08;
  private static final double wheelCircumference = 25.0;
  private static final double autoModeTargetDistance = clicksPerWheelRotation / wheelCircumference * 12 * 3; // 3 ft

  private static int driveDirection = 1;

  public static Gyro rioGyro = new ADXRS450_Gyro();

  private CANSparkMax throwerMotor1;
  // private CANSparkMax throwerMotor2;

  @Override
  public void robotInit() {

    btnDriveMode = new JoystickButton(m_stick, 11);

    throwerMotor1 = new com.revrobotics.CANSparkMax(kThrowerMotor, MotorType.kBrushless);

    throwerMotor1.restoreFactoryDefaults();

    m_encoder = throwerMotor1.getEncoder();
    // m_encoder2 = throwerMotor2.getEncoder();

    rioGyro.calibrate();
    rioGyro.reset();

    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    rearLeft.setSelectedSensorPosition(0);
    rearRight.setSelectedSensorPosition(0);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickChannel);

    lime.init("limelight");
  }

  @Override
  public void autonomousInit() {
    super.autonomousInit();
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    rearLeft.setSelectedSensorPosition(0);
    rearRight.setSelectedSensorPosition(0);

    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    rearLeft.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);

    driveDirection = 1;

  }

  @Override
  public void teleopInit() {
    super.teleopInit();

    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    rearLeft.setSelectedSensorPosition(0);
    rearRight.setSelectedSensorPosition(0);

    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    rearLeft.setNeutralMode(NeutralMode.Coast);
    rearRight.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void autonomousPeriodic() {
    super.autonomousPeriodic();
    double flSP = frontLeft.getSelectedSensorPosition();
    double frSP = frontRight.getSelectedSensorPosition();
    double rlSP = rearLeft.getSelectedSensorPosition();
    double rrSP = rearRight.getSelectedSensorPosition();
    SmartDashboard.putNumber("mFL", flSP);
    SmartDashboard.putNumber("mFR", frSP);
    SmartDashboard.putNumber("mRL", rlSP);
    SmartDashboard.putNumber("mRR", rrSP);

    if (driveDirection == 1) {
      if ((Math.abs(flSP) + Math.abs(frSP)) / 2 < autoModeTargetDistance) {
        m_robotDrive.driveCartesian(0.0, 0.25, 0.0, 0.0);
      } else {
        m_robotDrive.stopMotor();
        driveDirection = -1;
        Timer.delay(.75);

      }
    }

    else if (driveDirection == -1) {
      if (((flSP) + -(frSP)) / 2 > 0) {
        SmartDashboard.putNumber("mPosition", ((Math.abs(flSP) + Math.abs(frSP))/2));
        m_robotDrive.driveCartesian(0.0, -0.25, 0.0, 0.0);
      } else {
        m_robotDrive.stopMotor();
        driveDirection = 0;
      }

    }
    else {
      m_robotDrive.stopMotor();
    }
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

    SmartDashboard.putNumber("mFL", frontLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("mFR", frontRight.getSelectedSensorPosition());
    SmartDashboard.putNumber("mRL", rearLeft.getSelectedSensorPosition());
    SmartDashboard.putNumber("mRR", rearRight.getSelectedSensorPosition());

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

    SmartDashboard.putNumber("xRed", detectedColor.red);
    SmartDashboard.putNumber("xGreen", detectedColor.green);
    SmartDashboard.putNumber("xBlue", detectedColor.blue);
    SmartDashboard.putNumber("xConfidence", match.confidence);
    SmartDashboard.putString("xDetected Color", colorString);
    // SmartDashboard.putString("Match", match.color);

    if (DriveByStick) {
      m_robotDrive.driveCartesian(Math.pow(m_stick.getX(), 3) * .75, Math.pow(-m_stick.getY(), 3) * 1,
          Math.pow(m_stick.getZ(), 3) * .5, 0.0);
    } else {
      Double horizontalOffset = lime.getTargetOffsetHorizontal();
      Double verticalOffset = lime.getTargetOffsetVertical();
      // Double targetArea = lime.getTargetArea();

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
