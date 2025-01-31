// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class SparkFlexSetter implements PIDSetter {
    private List<SparkFlexConfig> configs = new ArrayList<SparkFlexConfig>();
    private Gains gains;

    public SparkFlexSetter(SparkFlexConfig... configs) {
        this.configs.addAll(Arrays.asList(configs));
    }

    @Override
    public void setPID(Gains gains) {
        this.gains = gains;
        configs.forEach(config -> config.closedLoop.pidf(gains.P, gains.I, gains.D, gains.F));
    }

    @Override
    public Gains getPID() {
        return gains;
    }

}
