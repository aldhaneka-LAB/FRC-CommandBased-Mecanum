package frc.robot.commands.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;

// import java.time.LocalDateTime;
// import java.time.LocalTime;
// import java.time.format.DateTimeFormatter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsytem;

public class MoveFront extends CommandBase {
    private final DriveSubsytem m_Subsytem;
    private int duration;
    private double initTime;
    private boolean isFinished = false;
    private double speed = 0;

    public MoveFront(DriveSubsytem m_ubsytem, int duration, double speed) {
        m_Subsytem = m_ubsytem;
        addRequirements(m_Subsytem);
        this.duration = duration;
        this.speed = speed;
    }

    @Override
    public void initialize() {
       this.initTime= Timer.getFPGATimestamp();
        // DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");  
        // LocalTime now = LocalTime.now();  
        // System.out.println(dtf.format(now));  
     
    }

    @Override
    public void execute() {
        System.out.print("Execute move front");
        double p = Timer.getFPGATimestamp();
        System.out.print(initTime);
        System.out.print(" ");
        System.out.println(p);

        if (p - initTime < duration) {
            m_Subsytem.drive(this.speed, 0, 0);
        } else {
            m_Subsytem.drive(0, 0, 0);
            isFinished = true;
            
        }

    }

    // @Override @Override
    // public void execute() {
    //     // TODO Auto-generated method stub
    //     super.execute();
    // }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
