package frc.robot.components;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
  LimeLight wrapper class
*/

public class LimeLight {
  private static NetworkTableInstance table = null;
  private static String tableName = "";

  public static enum LightMode {
    pipeline, off, blink, on
  }
  public static enum CameraMode {
    vision, driver
  }

 
  public final void init(String table_name) {
    if (table == null) {
      table = NetworkTableInstance.getDefault();
    }
    tableName = table_name;
  }

  private static NetworkTableEntry getTableEntry(String key) {
    return table.getTable(tableName).getEntry(key);
  }
  private static void setTableEntry(String key, int value) {
    NetworkTableEntry tableEntry = getTableEntry(key);
    tableEntry.setNumber(value);
  }

  /* PUBLIC METHODS */

  /**
    Determines if target is visible
    @return true or false
  */
  public static boolean isTargetVisible() {
    return getTableEntry("tv").getDouble(0) == 1;
  }

  /**
    Horizontal offset from crosshair to target (LimeLight tx value)
    @return double (+/-27 degrees)
  */
  public static double getTargetOffsetHorizontal() {
    return getTableEntry("tx").getDouble(0.00);
  }

  /**
    Vertical offset from crosshair to target (LimeLight ty value)
    @return double (+/-20.5 degrees)
  */
  public static double getTargetOffsetVertical() {
    return getTableEntry("ty").getDouble(0.00);
  }

  /**
    Target area as percentage of camera FOV (LimeLight ta value)
    @return double (0-100)
  */
  public static double getTargetArea() {
    return getTableEntry("ta").getDouble(0.00);
  }

  /**
    Target skew (LimeLight ts value)
    @return double (-90 to 0 degrees)
  */
  public static double getTargetSkew() {
    return getTableEntry("ts").getDouble(0.00);
  }

  /**
    Pipeline latency (LimeLight tl value)
    @return double (milliseconds)
  */
  public static double getPipelineLatency() {
    return getTableEntry("tl").getDouble(0.00);
  }

  /**
    Total latency (LimeLight tl value + 11 ms for image capture)
    @return double (milliseconds)
  */
  public static double getLatency() {
    Double latency = getTableEntry("tl").getDouble(0.00);
    if (latency == 0) {
      return latency;
    }
    return latency + 11.0;
  }

  /**
    Height of the rough bounding box (LimeLight tvert value)
    @return double (0-320 pixels)
  */
  public static double getBoundingBoxHeight() {
    return getTableEntry("tvert").getDouble(0.00);
  }

  /**
    Width of the rough bounding box (LimeLight thor value)
    @return double (0-320 pixels)
  */
  public static double getBoundingBoxWidth() {
    return getTableEntry("thor").getDouble(0.00);
  }


  /* ***** SETTERS ***** */

  /**
    Set LED ring mode
    @param mode LightMode
    @example
      LimeLight targetingLimeLight = new LimeLight('targeting_limelight');
      targetingLimeLight.setLedMode(LimeLight.LightMode.on);
  */
  public static void setLedMode(LightMode mode) {
    getTableEntry("ledMode").setNumber(mode.ordinal());
  }

  /**
    Set camera mode
    @param mode CameraMode
    @example
      LimeLight targetingLimeLight = new LimeLight('targeting_limelight');
      targetingLimeLight.setCameraMode(LimeLight.CameraMode.vision);
  */
  public static void setCameraMode(CameraMode mode) {
    getTableEntry("camMode").setNumber(mode.ordinal());
  }

  /**
    Set pipeline (integer from 0 to 9)
    @param number int
    @example
      LimeLight targetingLimeLight = new LimeLight('targeting_limelight');
      targetingLimeLight.setPipeline(2);
  */
  public static void setPipeline(int number) {
    getTableEntry("pipeline").setNumber(number);
  }

}