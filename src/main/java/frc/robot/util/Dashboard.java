package frc.robot.util;

public class Dashboard {
    private static final String TAB_NAME = "Main";

    private final FieldReport fieldReport;

    public Dashboard() {
        fieldReport = new FieldReport();
        fieldReport.reportSwerve();
        
    }
    public void periodic () {
        fieldReport.periodic();
    }   
}
