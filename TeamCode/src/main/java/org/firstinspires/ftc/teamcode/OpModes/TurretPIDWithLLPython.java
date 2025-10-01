package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import edu.wpi.first.networktables.NetworkTableInstance;

@TeleOp(name = "TurretPIDWithLLPython", group = "Examples")
public class TurretPIDWithLLPython extends LinearOpMode {
    private DcMotor turretMotor;
    private double Kp = 0.01;  // Tune these
    private double Ki = 0.0001;
    private double Kd = 0.005;
    private double integral = 0.0;
    private double previousError = 0.0;
    private double setpoint = 0.0;  // Center the tag
    private double tx = 0.0;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Initialize NetworkTables
//        NetworkTableInstance inst = NetworkTableInstance.getDefault();
//        inst.startClient4("TurretClient");
//        inst.setServer("limelight");  // Ensure Limelight hostname is correct
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // Get tx from llpython
//            tx = getLimelightTx();
            double error = setpoint - tx;
            // PID calculations
            double proportional = Kp * error;
            integral += error * 0.02;
            double integralTerm = Ki * integral;
            double derivative = Kd * (error - previousError) / 0.02;
            previousError = error;
            double output = proportional + integralTerm + derivative;
            output = Math.max(-1.0, Math.min(1.0, output));
            turretMotor.setPower(output);
            telemetry.addData("tx", tx);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.update();
        }
    }
//    private double getLimelightTx() {
//        // Fetch llpython array from Limelight
//        double[] llpython = NetworkTableInstance.getDefault()
//                .getTable("limelight")
//                .getEntry("llpython")
//                .getDoubleArray(new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//        return llpython[0];  // Assuming tx is in the first slot
//    }
}