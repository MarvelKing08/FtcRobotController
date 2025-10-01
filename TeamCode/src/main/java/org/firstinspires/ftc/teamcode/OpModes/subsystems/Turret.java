//package org.firstinspires.ftc.teamcode.OpModes.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
////import com.nextftc.lib.subsystems.Subsystem;
//import com.nextftc.lib.commands.Command;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class Turret extends subsystems {
//
//    private final DcMotorEx turretMotor;
//
//    // PID constants (tune these!)
//    private final double kP = 0.02;
//    private final double kI = 0.0;
//    private final double kD = 0.001;
//
//    private double integral = 0;
//    private double lastError = 0;
//    private ElapsedTime timer = new ElapsedTime();
//
//    public Turret(HardwareMap hardwareMap) {
//        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
//        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    /**
//     * Returns a command that will continuously track an AprilTag
//     * using Limelight tx as input to a PID loop.
//     */
//    public Command trackTarget() {
//        return new Command() {
//
//            @Override
//            public void initialize() {
//                timer.reset();
//                integral = 0;
//                lastError = 0;
//            }
//
//            @Override
//            public void execute() {
//                // ✅ Step 1: Get Limelight values
//                double tx = Limelight.getTx();  // horizontal offset
//
//                // ✅ Step 2: PID calculations
//                double error = 0 - tx; // goal: tx = 0 (centered)
//                double dt = timer.seconds();
//                timer.reset();
//
//                integral += error * dt;
//                double derivative = (error - lastError) / dt;
//                double output = kP * error + kI * integral + kD * derivative;
//
//                // clamp power so motor doesn’t slam
//                output = Math.max(-0.5, Math.min(0.5, output));
//
//                // ✅ Step 3: Apply to turret motor
//                turretMotor.setPower(output);
//
//                // ✅ Step 4: Save error
//                lastError = error;
//            }
//
//            @Override
//            public void end(boolean interrupted) {
//                turretMotor.setPower(0);
//            }
//
//            @Override
//            public boolean isFinished() {
//                // This runs until canceled, so always return false
//                return false;
//            }
//        };
//    }
//}
