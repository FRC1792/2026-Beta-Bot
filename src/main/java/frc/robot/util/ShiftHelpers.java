package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;

public class ShiftHelpers {


    public enum ACTIVITY_STATE {
        AUTON,
        TRANSITION_FILL,
        TRANSITION_SHOOT,
        INACTIVE_SNOW_BLOW,
        INACTIVE_FILL_AND_RETURN,
        ACTIVE_FILL_AND_SHOOT
    }

    public enum SHIFT_STATE {
        AUTO_TRANSITION,
        SHIFT_1_RED,
        SHIFT_2_RED,
        SHIFT_3_RED,
        SHIFT_4_RED,

        SHIFT_1_BLUE,
        SHIFT_2_BLUE,
        SHIFT_3_BLUE,
        SHIFT_4_BLUE,

        ENDGAME,

        NONE
    }

    private static SHIFT_STATE currentShiftState = SHIFT_STATE.NONE;

    public static boolean blueWonAuto() {
        String matchInfo = DriverStation.getGameSpecificMessage();
        if (matchInfo != null && matchInfo.length() > 0) {
            Logger.recordOutput("ShiftHelpers/Received Data", true);
            return matchInfo.charAt(0) == 'B';
        }
        Logger.recordOutput("ShiftHelpers/Received Data", false);
        // Safe default if data isn't ready yet
        return false;
    }

    public static int timeLeftInShiftSeconds(double currentMatchTime) {
        if (currentMatchTime >= 130) {
            currentShiftState = SHIFT_STATE.AUTO_TRANSITION;
            return (int)(currentMatchTime - 130);

        } else if (currentMatchTime >= 105 && currentMatchTime <= 130) {// Shift 1
            currentShiftState = blueWonAuto() ? SHIFT_STATE.SHIFT_1_RED : SHIFT_STATE.SHIFT_1_BLUE; // Shift 1 is red if blue won auto, blue if red won auto
            return (int)(currentMatchTime - 105);

        } else if (currentMatchTime >= 80 && currentMatchTime <= 105) {// Shift 2
            currentShiftState = blueWonAuto() ? SHIFT_STATE.SHIFT_2_BLUE : SHIFT_STATE.SHIFT_2_RED; // Shift 2 is blue if blue won auto, red if red won auto
            return (int)(currentMatchTime - 80);

        } else if (currentMatchTime >= 55 && currentMatchTime <= 80) {// Shift 3
            currentShiftState = blueWonAuto() ? SHIFT_STATE.SHIFT_3_RED : SHIFT_STATE.SHIFT_3_BLUE; // Shift 3 is blue if blue won auto, red if red won auto
            return (int)(currentMatchTime - 55);

        } else if (currentMatchTime >= 30 && currentMatchTime <= 55) {// Shift 4
            currentShiftState = blueWonAuto() ? SHIFT_STATE.SHIFT_4_BLUE : SHIFT_STATE.SHIFT_4_RED; // Shift 4 is blue if blue won auto, red if red won auto
            return (int)(currentMatchTime - 30);

        } else {// Endgame
            blueWonAuto(); // Ensure we record whether we received data or not
            currentShiftState = SHIFT_STATE.ENDGAME;
            return (int)currentMatchTime;
        }
    }

    public static boolean isCurrentShiftBlue(double currentMatchTime) {
        if (currentMatchTime >= 105 && currentMatchTime <= 130) {
            return blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 80 && currentMatchTime <= 105) {
            return blueWonAuto() ? true : false;
        } else if (currentMatchTime >= 55 && currentMatchTime <= 80) {
            return blueWonAuto() ? false : true;
        } else if (currentMatchTime >= 30 && currentMatchTime <= 55) {
            return blueWonAuto() ? true : false;
        } else {
            blueWonAuto(); // Ensure we record whether we received data or not
            return true;
        }
    }

    public static boolean currentShiftIsYours() {
        double currentMatchTime = DriverStation.getMatchTime();
        boolean isBlueShift = isCurrentShiftBlue(currentMatchTime);
        if (isBlue()) { // If we're on the Blue alliance
            return isBlueShift;
        } else {
            return !isBlueShift;
        }
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    public static SHIFT_STATE getCurrentShiftState() {
        return currentShiftState;
    }

    /**
     * Returns the current activity state based on match time and whether we won auto.
     * WIN (won auto): Transition Fill/Shoot → Inactive → Active → Inactive → Active
     * LOSS (lost auto): Transition Fill/Shoot → Active → Inactive → Active → Inactive → Active
     */
    public static ACTIVITY_STATE getActivityState(double currentMatchTime) {
        boolean wonAuto = isBlue() == blueWonAuto();

        // Transition phases are the same for win/loss
        if (currentMatchTime > 135) { // 140-135: Transition Fill (5s)
            return ACTIVITY_STATE.TRANSITION_FILL;
        } else if (currentMatchTime > 127) { // 135-127: Transition Shoot (8s)
            return ACTIVITY_STATE.TRANSITION_SHOOT;
        }

        if (wonAuto) {
            // WIN: inactive first, then active
            if (currentMatchTime > 118) return ACTIVITY_STATE.INACTIVE_SNOW_BLOW;       // 127-118 (9s)
            if (currentMatchTime > 108) return ACTIVITY_STATE.INACTIVE_FILL_AND_RETURN;  // 118-108 (10s)
            if (currentMatchTime > 77)  return ACTIVITY_STATE.ACTIVE_FILL_AND_SHOOT;     // 108-77 (31s)
            if (currentMatchTime > 68)  return ACTIVITY_STATE.INACTIVE_SNOW_BLOW;        // 77-68 (9s)
            if (currentMatchTime > 58)  return ACTIVITY_STATE.INACTIVE_FILL_AND_RETURN;  // 68-58 (10s)
            return ACTIVITY_STATE.ACTIVE_FILL_AND_SHOOT;                                 // 58-0 (58s)
        } else {
            // LOSS: active first, then inactive
            if (currentMatchTime > 102) return ACTIVITY_STATE.ACTIVE_FILL_AND_SHOOT;     // 127-102 (25s)
            if (currentMatchTime > 93)  return ACTIVITY_STATE.INACTIVE_SNOW_BLOW;        // 102-93 (9s)
            if (currentMatchTime > 83)  return ACTIVITY_STATE.INACTIVE_FILL_AND_RETURN;  // 93-83 (10s)
            if (currentMatchTime > 52)  return ACTIVITY_STATE.ACTIVE_FILL_AND_SHOOT;     // 83-52 (31s)
            if (currentMatchTime > 43)  return ACTIVITY_STATE.INACTIVE_SNOW_BLOW;        // 52-43 (9s)
            if (currentMatchTime > 33)  return ACTIVITY_STATE.INACTIVE_FILL_AND_RETURN;  // 43-33 (10s)
            return ACTIVITY_STATE.ACTIVE_FILL_AND_SHOOT;                                 // 33-0 (33s)
        }
    }
}