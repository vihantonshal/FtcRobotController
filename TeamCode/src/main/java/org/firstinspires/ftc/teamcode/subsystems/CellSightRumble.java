package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Tells the driver by feel when the camera starts or stops seeing an up CELL, so they don't have to
 * look at the Driver Hub while turning toward the HIVE.
 *  - CELL comes into view: two short blips.
 *  - CELL out of view for LOST_SECONDS: one long rumble (LOST_RUMBLE_MS; 0 = no rumble when lost).
 * Only the change rumbles, not every loop. A CELL that drops out for a frame or two (shorter than
 * LOST_SECONDS) does not count as lost, so it doesn't buzz on every flicker.
 * Call update() once per loop with the TeleOp's target (vision.aimedUpCell() != null).
 */
@Config
public class CellSightRumble {
    public static boolean ENABLED = true;
    public static int SEEN_BLIPS = 2;
    public static double LOST_SECONDS = 0.5;
    public static int LOST_RUMBLE_MS = 400;

    private boolean seen = false;              // what the driver was last told
    private final ElapsedTime lostTimer = new ElapsedTime();

    public void update(boolean cellInView, Gamepad pad) {
        if (cellInView) {
            lostTimer.reset();
            if (!seen) {
                seen = true;
                if (ENABLED && SEEN_BLIPS > 0) {
                    pad.rumbleBlips(SEEN_BLIPS);
                }
            }
        } else if (seen && lostTimer.seconds() > LOST_SECONDS) {
            seen = false;
            if (ENABLED && LOST_RUMBLE_MS > 0) {
                pad.rumble(LOST_RUMBLE_MS);
            }
        }
    }

    // True while the driver has been told a CELL is in view.
    public boolean isSeen() {
        return seen;
    }
}
