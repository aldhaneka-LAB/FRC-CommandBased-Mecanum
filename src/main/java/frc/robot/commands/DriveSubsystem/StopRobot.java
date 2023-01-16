package frc.robot.commands.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsytem;

public class StopRobot extends CommandBase {
    private final DriveSubsytem m_Subsytem;

    public StopRobot(DriveSubsytem m_ubsytem) {
        m_Subsytem = m_ubsytem;
        addRequirements(m_Subsytem);
    }
    
    @Override
    public void initialize() {
        // DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");  
        // LocalTime now = LocalTime.now();  
        // System.out.println(dtf.format(now));  

        // m_Subsytem.StopTheRobot();
        System.out.println("STOP");

        m_Subsytem.drive(0, 0, 0);
    }

    // @Override @Override
    // public void execute() {
    //     // TODO Auto-generated method stub
    //     super.execute();
    // }

    @Override
    public boolean isFinished() {
        return true;
    }
}
