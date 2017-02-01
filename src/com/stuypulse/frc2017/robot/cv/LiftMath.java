package com.stuypulse.frc2017.robot.cv;

import static com.stuypulse.frc2017.robot.CVConstants.CAMERA_Y;
import static com.stuypulse.frc2017.robot.CVConstants.LIFT_TARGET_Y;
import static com.stuypulse.frc2017.robot.CVConstants.CAMERA_FOCAL_LENGTH_Y;
import static com.stuypulse.frc2017.robot.CVConstants.CAMERA_FRAME_PX_HEIGHT;

import com.stuypulse.frc2017.util.Vector;

public class LiftMath {
    /**
     * @param lift_left Position of the left edge of the lift
     * relative to the lift camera
     * @param lift_right Position of the right edge of the lift
     * relative to the lift camera
     * @param intermediate_dist Distance from the peg base (along the
     * lift's normal) which the bot should go to before rotating and
     * approaching the peg head-on.
     * @param final_dist Distance from the peg to stop at
     * @return Path along which the robot should move, expressed as an
     * array of vectors describing discrete linear movements.
     */
    public static Vector[] getPath(
            Vector lift_left,
            Vector lift_right,
            double intermediate_dist,
            double final_dist
            ) {
        // Get average distance between lift targets (approximate position of peg)
        Vector peg = lift_left.plus(lift_right).scaleBy(0.5);
        // Get distance between left and right lift strips
        Vector lift_ltr = lift_right.minus(lift_left);
        // Get vector from peg to point on our path where we stop and turn
        Vector from_peg = lift_ltr.rotateBy(-90).withMagnitude(intermediate_dist);
        // Get vector from current location to point on our path where we stop and turn
        Vector m1 = peg.plus(from_peg);
        // Get vector from m1 to where the bot will stop, right in front of the peg.
        Vector m2 = from_peg.scaleBy(-1.0).withMagnitude(intermediate_dist - final_dist);
        return new Vector[] {m1, m2};
    }
    
    public static double radiansToDegrees(double rads) {
    	return rads / Math.PI * 180.0;
    }

    /**
     * @param stripY Center y-coordinate of reflexite strip.
     * @return Distance from camera to the reflexite strip.
     */
    public static double stripYToDistance(double stripY) {
        //return (LIFT_TARGET_Y - CAMERA_Y) * Math.tan((Math.PI / 180.0) * Camera.frameYPxToDegrees(stripY));
        return ((LIFT_TARGET_Y - CAMERA_Y) * CAMERA_FOCAL_LENGTH_Y)
                / (stripY - (CAMERA_FRAME_PX_HEIGHT / 2) - 0.5);
    }

    /**
     * @param stripX Center x-coordinate of reflexite
     * @return Angle between where the camera is pointing and the reflexite strip.
     */
    public static double stripXToAngle(double stripX) {
        return Camera.frameXPxToDegrees(stripX);
    }

    public static Vector stripFramePosToPhysicalPos(double stripX, double stripY) {
        return Vector.fromPolar(stripXToAngle(stripX), stripYToDistance(stripY));
    }
}
