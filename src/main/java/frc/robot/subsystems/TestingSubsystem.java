package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestingSubsystem extends SubsystemBase {

    public double shootingVelocity = 4000;

    public TestingSubsystem() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Testing/Shooting Velocity", shootingVelocity);
    } 
}
