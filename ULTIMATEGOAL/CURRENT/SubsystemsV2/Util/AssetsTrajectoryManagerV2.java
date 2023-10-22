package org.firstinspires.ftc.teamcode.ULTIMATEGOAL.CURRENT.SubsystemsV2.Util;

import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfigManager;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.io.InputStream;

/**
 * Set of utilities for loading trajectories from assets (the plugin save location).
 */
public class AssetsTrajectoryManagerV2 {
    /**
     * Loads a trajectory config with the given name.
     */
    public static TrajectoryConfig loadConfig(String name) throws IOException {
        InputStream inputStream = AppUtil.getDefContext().getAssets().open("trajectory/" + name + ".yaml");
        return TrajectoryConfigManager.loadConfig(inputStream);
    }
}
