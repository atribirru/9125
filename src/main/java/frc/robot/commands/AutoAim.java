package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightVision;
import frc.robot.Robot;

public class TurretRotationCommand extends Command {

    private final LimelightVision limelight;
    private final PIDController pidController;

    private static final double kP = 0.02;  // TODO: tune
    private static final double kI = 0.0;   // TODO: tune
    private static final double kD = 0.0;   // TODO: tune
    private static final double MAX_SPEED = 0.5;  // TODO: tune - hopefully iy can be 1.0 but probably not 
    private static final double TOLERANCE = 1.0;  // how close ot target angle is acceptable ;)

    public TurretRotationCommand(LimelightVision limelight) {
        this.limelight = limelight;
        this.pidController = new PIDController(kP, kI, kD);
        this.pidController.setSetpoint(0.0); // target: tag centered (tx = 0)
        this.pidController.setTolerance(TOLERANCE);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            double hubX = limelight.getTX();
            double hubY = limelight.getTL();
            // double hubZ = limelight.getTZ(); // we don't care about this

            double turretEncoderPosition = Robot.turretMotor.getEncoder().getPosition();
            double robotYaw = Robot.gyro.getYaw();
            double currentTurretAngle = turretEncoderPosition - robotYaw;

            double targetAngle = Math.atan2(hubX, hubY);

            pidController.setSetpoint(targetAngle);

            double output = pidController.calculate(currentTurretAngle);

            output = MathUtil.clamp(output, -MAX_SPEED, MAX_SPEED);
            Robot.turretMotor.set(output);

            System.out.println("Target position: " + targetAngle); // Debugging output
            System.out.println("Turret angle: " + currentTurretAngle); // Debugging output

            System.out.println("Motor power: " + output); // Debugging output
        } else {
            Robot.turretMotor.set(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.turretMotor.set(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted
    }
}