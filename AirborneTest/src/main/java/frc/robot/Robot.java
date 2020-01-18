/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.LimeLight;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 5;
  private static final int kRearLeftChannel = 8;
  private static final int kFrontRightChannel = 10;
  private static final int kRearRightChannel = 9;

  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private LimeLight lime = new LimeLight();

  public static boolean DriveByStick = false;
  public JoystickButton btnDriveMode;

  private static final double targetHeight = 18;  //Inches
  private static final double optimalDistance = 84; //Inches

  @Override
  public void robotInit() {

    btnDriveMode = new JoystickButton(m_stick, 11);
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
    WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
    WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel);
    WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);

    // Invert the left side motors.
    // You may need to change or remove this to match your robot.
    // frontLeft.setInverted(true);
    // rearLeft.setInverted(true);
    // frontRight.setInverted(true);
    // rearRight.setInverted(true);

    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickChannel);

    lime.init("limelight-front");
  }

  @Override
  public void teleopPeriodic() {
    // Use button 11 to change drive mode (joystick or LimeLight)
    //btnDriveMode.whenPressed(new ChangeDrive());

    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.

    if(DriveByStick){
      m_robotDrive.driveCartesian(Math.pow(m_stick.getX(), 3)*.75, Math.pow(-m_stick.getY(), 3)*1, Math.pow(m_stick.getZ(), 3)*.5, 0.0);
    }
    else{
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
        newZ = minPowerFloor* (int)Math.signum(horizontalOffset) + Math.sin(horizontalOffset*Math.PI/180);
      }
      else {
        newZ = 0.0;
      }

      // Compute the distance to the target based on height and power to move forward/backward
      if(Math.abs(verticalOffset) > 0){
      targetDistance = targetHeight / Math.tan(verticalOffset*Math.PI/180);
      }
      else{targetDistance = optimalDistance;}
      SmartDashboard.putNumber("targetDistance", targetDistance);
      if(Math.abs(optimalDistance - targetDistance) >= 6 ){
        if(targetDistance > optimalDistance) {
          newX = minPowerFloor;
        }
        else if(targetDistance < optimalDistance){
          newX = -minPowerFloor;
        }
      }
      else {newX = 0.0;}
      SmartDashboard.putNumber("newX", newX);
      //Distance tracking by target area - decided too likely to be wrong
      // if (targetArea < 0.45) {
      //   newY = minPowerFloor*1.15;
      // } else if (targetArea > 0.65) {
      //   newY = -minPowerFloor*1.15;
      // }
      m_robotDrive.driveCartesian(newY, newX, newZ, 0.0);
    }
  }

}
