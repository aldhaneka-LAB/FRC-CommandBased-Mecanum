package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsytem extends SubsystemBase {
    private final CANSparkMax f_rightMotor = new CANSparkMax(DriveConstants.fr_motor, MotorType.kBrushless);
    private final CANSparkMax f_leftMotor = new CANSparkMax(DriveConstants.fl_motor, MotorType.kBrushless);
    private final CANSparkMax r_rightMotor = new CANSparkMax(DriveConstants.br_motor, MotorType.kBrushless);
    private final CANSparkMax r_leftMotor = new CANSparkMax(DriveConstants.bl_motor, MotorType.kBrushless);

    private final RelativeEncoder f_rightMEncoder = f_rightMotor.getEncoder();
    private final RelativeEncoder f_leftMEncoder = f_leftMotor.getEncoder();
    private final RelativeEncoder r_rightMEncoder = r_rightMotor.getEncoder();
    private final RelativeEncoder r_leftMEncoder = r_leftMotor.getEncoder();

    public Field2d m_Field2d;

    // The gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
            DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), new MecanumDriveWheelPositions(
                    f_leftMEncoder.getPosition(), f_rightMEncoder.getPosition(), r_leftMEncoder.getPosition(),
                    r_rightMEncoder.getPosition()));

   private final MecanumDrive m_drive = new MecanumDrive(f_leftMotor, r_leftMotor, f_rightMotor, r_rightMotor);

    public DriveSubsytem(Field2d m_Field2dP) {

        m_Field2d = m_Field2dP;

        // InchToMeters
        // Utils
        // InchTO

        r_rightMotor.restoreFactoryDefaults();
        r_leftMotor.restoreFactoryDefaults();
        f_rightMotor.restoreFactoryDefaults();
        f_leftMotor.restoreFactoryDefaults();
        r_rightMotor.setIdleMode(IdleMode.kCoast);
        r_leftMotor.setIdleMode(IdleMode.kCoast);
        f_rightMotor.setIdleMode(IdleMode.kCoast);
        f_leftMotor.setIdleMode(IdleMode.kCoast);

        f_rightMEncoder.setPositionConversionFactor(DriveConstants.w);
        f_leftMEncoder.setPositionConversionFactor(DriveConstants.w);
        r_rightMEncoder.setPositionConversionFactor(DriveConstants.w);
        r_leftMEncoder.setPositionConversionFactor(DriveConstants.w);


        
        // m_drive.rese

        r_rightMotor.setInverted(true);
        f_rightMotor.setInverted(true);

        // f_rightMotor.getEncoder()

        // f_rightMotor.setVoltage(0);
        // f_rightMotor.set(0);
        // f_leftMotor.set(0);
        // r_rightMotor.set(0);
        // r_leftMotor.set(0);
    }


    public MecanumDriveWheelPositions getWheelPosition() {
        return new MecanumDriveWheelPositions(f_leftMEncoder.getPosition(), f_rightMEncoder.getPosition(), r_leftMEncoder.getPosition(),
        r_rightMEncoder.getPosition());
    }

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
            f_leftMEncoder.getVelocity(),
            r_leftMEncoder.getVelocity(),
            f_rightMEncoder.getVelocity(),
            f_leftMEncoder.getVelocity()
        );
    }

    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        f_leftMotor.setVoltage(volts.frontLeftVoltage);
        r_leftMotor.setVoltage(volts.rearLeftVoltage);
        f_rightMotor.setVoltage(volts.frontRightVoltage);
        r_rightMotor.setVoltage(volts.rearRightVoltage);
    }
    
    @Override
    public void periodic() {
        // System.out.println("Periodic on Drive Subsystems!");
        // TODO Auto-generated method stub
        // super.periodic();

        m_odometry.update(m_gyro.getRotation2d(), this.getWheelPosition());
                
        Pose2d robotPose2d = m_odometry.getPoseMeters();
        m_Field2d.setRobotPose(robotPose2d);

        SmartDashboard.putNumber("X", robotPose2d.getX());
        SmartDashboard.putNumber("Y", robotPose2d.getY());
        SmartDashboard.putNumber("Z", robotPose2d.getRotation().getDegrees());
        // SmartDashboard.putNumber("Z", robotPose2d.getRotation().getDegrees());

        SmartDashboard.putNumber("X", robotPose2d.getX());
        SmartDashboard.putNumber("Y", robotPose2d.getY());
        SmartDashboard.putNumber("Z", robotPose2d.getRotation().getDegrees());
        
    }


    public void driveEachModule(double fl, double fr, double rl, double rr) {
        f_leftMotor.set(fl);
        f_rightMotor.set(fr);
        r_leftMotor.set(rl);
        r_rightMotor.set(rr);
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        // if (fieldRelative) {
        // m_drive.driveCartesian(ySpeed, xSpeed, rot);
        // } else {
        // m_drive(xSpeed);
        // }
        // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d());

        // var mecanumModuleStates = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);

        // // Get the individual wheel speeds

        // double forward = mecanumModuleStates;
        // double sideways = mecanumModuleStates.vyMetersPerSecond;
        // double angular = mecanumModuleStates.omegaRadiansPerSecond;
        // SmartDashboard.putData("frontLeft", String(frontLeft));
// 
        // m_drive.driveCartesian(xSpeed, xSpeed, rot);
        double m_deadband = 0.02;
        double m_maxOutput = 1.0;

        xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband);
    
        var speeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, rot);
    
        f_leftMotor.set(speeds.frontLeft * m_maxOutput);
        f_rightMotor.set(speeds.frontRight * m_maxOutput);
        r_leftMotor.set(speeds.rearLeft * m_maxOutput);
        r_rightMotor.set(speeds.rearRight * m_maxOutput);

        // System.out.println("FRONT LEFT", speeds.frontLeft);
        // System.out.println("FRONT LEFT", speeds.frontLeft);

        SmartDashboard.putNumber("FRONT LEFT", speeds.frontLeft);
        SmartDashboard.putNumber("FRONT RIGHT", speeds.frontRight);
        SmartDashboard.putNumber("REAR LEFT", speeds.rearLeft);
        SmartDashboard.putNumber("REAR RIGHT", speeds.rearRight);


    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }


    public void resetPose() {
        // MecanumDriveWhee
        // MecanumDriveWheelPositions
        m_odometry.resetPosition(new Rotation2d(), getWheelPosition(), getPose());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                this.getWheelPosition(),
                pose);
    }

    public void StopModule() {
        m_drive.stopMotor();
    }

    // public MecanumDrive

    // public CommandBase MoveFront() {
    // // f_rightMotor.set(0.5);
    // // f_leftMotor.set(0.5);
    // return run(
    // () -> {
    // f_rightMotor.set(0.5);
    // f_leftMotor.set(0.5);
    // System.out.println("Move Front for 10 seconds");
    // // Commands.waitSeconds(10);
    // // Commands.wait

    // }
    // );
    // }

    // public void StopTheRobot() {
    // f_rightMotor.set(0);
    // f_leftMotor.set(0);
    // }

    // public CommandBase Stop() {
    // return runOnce(() -> {
    // System.out.println("Move Frnt for 10 seconds done!");

    // f_rightMotor.set(0);
    // f_leftMotor.set(0);
    // });
    // }

    // public CommandBase AndtThen() {
    // return runOnce(() -> {
    // System.out.println("And then??");
    // });
    // }

}
