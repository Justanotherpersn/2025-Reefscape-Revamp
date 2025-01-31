// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */

public class PIDDisplay extends SubsystemBase{

    //shuffeboard stuff

    public static final SendableChooser<PIDSetter> PIDList = new SendableChooser<>();
    public static final WPILibSetter defaultPID = new WPILibSetter(List.of(new PIDController(0, 0, 0)));

    private static ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");

    private static GenericEntry PEntry = PIDTab.add("P", 0).withPosition(0, 1).getEntry();
    private static GenericEntry IEntry = PIDTab.add("I", 0).withPosition(1, 1).getEntry();
    private static GenericEntry DEntry = PIDTab.add("D", 0).withPosition(2, 1).getEntry();
    private static GenericEntry SEntry = PIDTab.add("S", 0).withPosition(3, 1).getEntry();
    private static GenericEntry VEntry = PIDTab.add("V", 0).withPosition(4, 1).getEntry();

    PIDSetter selectedPID;
    PIDSetter lastSelected;

    double PValue;
    double IValue;
    double DValue;
    double SValue;
    double VValue;

    public static void Init() {
        PIDTab.add(PIDList).withPosition(0, 0).withSize(4, 1);
        PIDList.setDefaultOption("Default PID", defaultPID);
    }

    @Override
    public void periodic(){
        //Check if the selected PID config is different. If so update the Dashboard with new numbers
        selectedPID = PIDList.getSelected();
        if (selectedPID == null) return;
        Gains codeCurrentPID = selectedPID.getPID();
        if (codeCurrentPID == null) return;
        
        if(selectedPID != lastSelected){
            PEntry.setDouble(codeCurrentPID.P);
            IEntry.setDouble(codeCurrentPID.I);
            DEntry.setDouble(codeCurrentPID.D);
            SEntry.setDouble(codeCurrentPID.S);
            VEntry.setDouble(codeCurrentPID.V);
        }
        lastSelected = selectedPID;
        
        PValue = PEntry.getDouble(codeCurrentPID.P);
        IValue = IEntry.getDouble(codeCurrentPID.I);
        DValue = DEntry.getDouble(codeCurrentPID.D);
        SValue = SEntry.getDouble(codeCurrentPID.S);
        VValue = VEntry.getDouble(codeCurrentPID.V);

        //Check if any of the dashboard values are different than the PID values in code, write updated values
        if(PValue != codeCurrentPID.P || IValue != codeCurrentPID.I || DValue != codeCurrentPID.D || SValue != codeCurrentPID.S || VValue != codeCurrentPID.V){
            selectedPID.setPID(new Gains(PValue, IValue, DValue, SValue, VValue));
        }
    }

    public interface PIDSetter {
        public void setPID(Gains gains);

        public Gains getPID();
    }
}
