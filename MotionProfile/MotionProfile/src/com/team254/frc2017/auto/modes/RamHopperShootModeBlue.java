package com.team254.frc2017.auto.modes;

import com.team254.frc2017.auto.AutoModeBase;
import com.team254.frc2017.auto.AutoModeEndedException;
import com.team254.frc2017.auto.actions.Action;
import com.team254.frc2017.auto.actions.DrivePathAction;
import com.team254.frc2017.auto.actions.ForceEndPathAction;
import com.team254.frc2017.auto.actions.ParallelAction;
import com.team254.frc2017.auto.actions.ResetPoseFromPathAction;
import com.team254.frc2017.auto.actions.SeriesAction;
import com.team254.frc2017.auto.actions.WaitAction;
import com.team254.frc2017.auto.actions.WaitForPathMarkerAction;
import com.team254.frc2017.paths.PathContainer;
import com.team254.frc2017.paths.StartToHopperBlue;

import java.util.Arrays;

/**
 * Rams the field hopper head on with the robot's intake then waits for the balls to fall into the robot and shoots
 * them.
 * 
 * @see AutoModeBase
 */
public class RamHopperShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        PathContainer hopperPath = new StartToHopperBlue();
        runAction(new ResetPoseFromPathAction(hopperPath));
        runAction(new DrivePathAction(hopperPath)
                ); // Drive to hopper, cancel path once the robot runs into the wall
    }
}
