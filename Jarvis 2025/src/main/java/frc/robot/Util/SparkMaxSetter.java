package frc.robot.Util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class SparkMaxSetter implements PIDSetter {

    private List<SparkMaxConfig> configs = new ArrayList<SparkMaxConfig>();
    private Gains gains;
    
    public SparkMaxSetter(SparkMaxConfig... configs){
        this.configs.addAll(Arrays.asList(configs));
    }

    public void addConfigurator(SparkMaxConfig config) {
        configs.add(config);
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
