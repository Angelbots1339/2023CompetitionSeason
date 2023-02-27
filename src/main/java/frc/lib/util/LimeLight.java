// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimeLight {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static String lastTarget = "";

    //Getters
    /**
     * @return Whether the limelight has any valid targets (0 or 1)

     */
    public static boolean hasTargets() {
        return table.getEntry("tx").getDouble(0.0) != 0;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    public static double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /**
     * @return Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     */
    public static double getVerticalOffset() {
        return table.getEntry("ty").getDouble(0.0);
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public static double getTargetArea() {
        return table.getEntry("ta").getDouble(0.0);
    }

    /**
     * @return The pipeline’s latency contribution (ms) Add at least 11ms for image
     *         capture latency.
     */
    public static double getLatency() {
        return table.getEntry("tl").getDouble(0.0);
    }

    /**
     * @return Side of target (-1 = left, 0 = no target, 1 = right)
     */
    public static double getSideOfTarget() {
        return table.getEntry("tshort").getDouble(0.0);
    }

    /**
     * @return Whether the limelight has any valid targets (0 or 1)
     */
    public static double getLongSideOfTarget() {
        return table.getEntry("tlong").getDouble(0.0);
    }

    /**
     * @return Horizontal sidelength of shortest side of the fitted bounding box (0 - 320
     *         pixels)
     */
    public static double getShortestSideOfTarget() {
        return table.getEntry("thor").getDouble(0.0);
    }

    /**
     * @return Vertical sidelength of shortest side of the fitted bounding box (0 - 320
     *         pixels)
     */
    public static double getVerticalSideOfTarget() {
        return table.getEntry("tvert").getDouble(0.0);
    }

    /**
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public static double getPipeline() {
        return table.getEntry("getpipe").getDouble(0.0);
    }



    public static String getLastTarget() {
        return lastTarget;
    }
  

    /**
     * @return
     * <table>
     *  <tr>
     *      <th> ledMode </th> <th> Sets limelight’s LED state</th>
     *  </tr>
     *  <tr>
     *      <td> 0 </td> <td> use the LED Mode set in the current pipeline</td>
     *  </tr>
     *  <tr>
     *      <td> 1 </td> <td> force off </td>
     *  </tr>
     *  <tr>
     *     <td> 2 </td> <td> force blink </td>
     *  </tr>
     *  <tr>
     *    <td> 3 </td> <td> force on </td>
     *  </tr>
     *  </table>
     */
    public static double getLedMode() {
        return table.getEntry("ledMode").getDouble(0.0);
    } 

    /**
     * @return
     * <table>
     *  <tr>
     *      <th> camMode </th> <th> Sets limelight’s operation mode</th>
     *  </tr>
     *  <tr>
     *      <td> 0 </td> <td> Vision Processor</td>
     *  </tr>
     *  <tr>
     *      <td> 1 </td> <td> Driver Camera (Increases exposure, disables vision processing)</td>
     *  </tr>
     *  </table>
     */
    public static double getCamMode() {
        return table.getEntry("camMode").getDouble(0.0);
    }
    //Setters
    /**
     * @return
     * <table>
     *  <tr>
     *      <th> ledMode </th> <th> Sets limelight’s LED state</th>
     *  </tr>
     *  <tr>
     *      <td> 0 </td> <td> use the LED Mode set in the current pipeline</td>
     *  </tr>
     *  <tr>
     *      <td> 1 </td> <td> force off </td>
     *  </tr>
     *  <tr>
     *     <td> 2 </td> <td> force blink </td>
     *  </tr>
     *  <tr>
     *    <td> 3 </td> <td> force on </td>
     *  </tr>
     *  </table>
     */
    public static void setLedMode(double ledMode) {
        table.getEntry("ledMode").setNumber(ledMode);
    }

    /**
     * @return
     * <table>
     *  <tr>
     *      <th> camMode </th> <th> Sets limelight’s operation mode</th>
     *  </tr>
     *  <tr>
     *      <td> 0 </td> <td> Vision Processor</td>
     *  </tr>
     *  <tr>
     *      <td> 1 </td> <td> Driver Camera (Increases exposure, disables vision processing)</td>
     *  </tr>
     *  </table>
     */
    public static void setCamMode(double camMode) {
        table.getEntry("camMode").setNumber(camMode);
    }

    /**
     * @return
     * <table>
     *  <tr>
     *      <th> pipeline </th> <th> Sets limelight’s current pipeline</th>
     *  </tr>
     *  <tr>
     *      <td> 0..9 </td> <td> Select pipeline 0..9</td>
     *  </tr>
     *  </table>
     */
    public static void setPipelineMode(double pipelineMode) {
        table.getEntry("pipeline").setNumber(pipelineMode);
    }

    /**
     * @return
     * <table>
     *  <tr>
     *      <th> stream </th> <th> Sets limelight’s streaming mode</th>
     *  </tr>
     *  <tr>
     *      <td> 0 </td> <td> Standard - Side-by-side streams if a webcam is attached to Limelight</td>
     *  </tr>
     *  <tr>
     *      <td> 1 </td> <td> PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream</td>
     *  </tr>
     *  <tr>
     *     <td> 2 </td> <td> PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream</td>
     *  </tr>
     *  </table>
     */
    public static void setStreamMode(double streamMode) {
        table.getEntry("stream").setNumber(streamMode);
    }
    

    /**
     * @return
     * <table>
     *  <tr>
     *      <th> snapshot </th> <th> Sets limelight’s snapshot mode</th>
     *  </tr>
     *  <tr>
     *      <td> 0 </td> <td> Stop taking snapshots</td>
     *  </tr>
     *  <tr>
     *      <td> 1 </td> <td> Take two snapshots per second</td>
     *  </tr>
     *  </table>
     */
    public static void setSnapshotMode(boolean snapshotMode) {
        table.getEntry("snapshot").setBoolean(snapshotMode);
    }

    public static void setLastTarget(String value) {
        lastTarget = value;
    }

  

    

}
