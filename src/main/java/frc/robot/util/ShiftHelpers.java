package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class ShiftHelpers {


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
            return matchInfo.charAt(0) == 'B';
        }
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
}