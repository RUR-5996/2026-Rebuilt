package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;


public class Dashboard {
    private static final String TAB_NAME = "Main";

    private final fieldReport fieldReport;

    public Dashboard() {
        fieldReport = new fieldReport();
        fieldReport.reportSwerve();
        
    }
    public void periodic () {
        fieldReport.periodic();
    }   
}
