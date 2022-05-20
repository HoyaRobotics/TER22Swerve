 package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.Constants;

//import java.util.function.Double;

public class DefaultDriveCommand extends CommandBase {
    private final DriveBase m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DriveBase drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        
        
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
 
 //       this.m_translationXSupplier = modifyAxis(translationXSupplier.getAsDouble()*DriveBase.MAX_VELOCITY_METERS_PER_SECOND);
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void execute() {
      // test
//      schedule();
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        System.out.println(m_translationXSupplier.getAsDouble());
        
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                  modifyAxis(m_translationXSupplier.getAsDouble()*DriveBase.MAX_VELOCITY_METERS_PER_SECOND),
                  modifyAxis(m_translationYSupplier.getAsDouble()*DriveBase.MAX_VELOCITY_METERS_PER_SECOND),
                  modifyAxis(m_rotationSupplier.getAsDouble()*DriveBase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
//        SmartDashboard.putNumber("XJoystick", m_translationXSupplier);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    private double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            System.out.println("Value > 0");
            return (value - deadband) / (1.0 - deadband);
          } else {
            System.out.println("Value <=0");
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          System.out.println("Abs Value < deadband");
          return 0.0;
        } 
      }
    
      private double modifyAxis(double value) {
        System.out.println(value);
        // Deadband
        value = deadband(value, 0.05);
    
        // Square the axis
        value = Math.copySign(value * value, value);
       System.out.println(value);
       System.out.println("*************************************");
       System.out.println("Offset FL");
       System.out.println(Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
        return value; 
      }
} 