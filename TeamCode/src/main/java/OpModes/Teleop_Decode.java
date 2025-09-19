/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package OpModes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop_Decode", group="Robot")
public class Teleop_Decode extends LinearOpMode {

    /* Declare OpMode members. */
    // ======================= MOTORS =======================

//    MotorEx motorEx = new MotorEx(() -> DcMotorEx);

    public DcMotorEx leftFront   = null;
    public DcMotorEx leftBack   = null;
    public DcMotorEx  rightFront  = null;
    public DcMotorEx  rightBack  = null;
    public DcMotor liftMotor = null;
    public DcMotor extensionMotor = null;
    public DcMotor ascentMotor1 = null;
    public DcMotor ascentMotor2 = null;

    // ======================= SERVOS =======================
    public Servo claw = null;
    double clawOffset = 0;
    public Servo intake = null;
    public Servo PivotServo = null;
    public Servo basketServo = null;

    // ======================= STATE MACHINES =======================
    int liftMotorStateMachine = 1;
    int intakeDropStateMachine = 0;
    int intakeReturnStateMachine = 0;
    int clawStateMachine = 1;
    int scaleSpeedStateMachine = 1;
    int basketDumpStateMachine = 1;
    double scaleTurningSpeed = .8;
    double scaleFactor = 1; //.5;
    int direction = -1;

    // ======================= SENSORS =======================
    public DistanceSensor distanceSensor;
    HardwareMap hwMap = null;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime currentTime = new ElapsedTime();
    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ; // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    double v1, v2, v3, v4;
    public CameraName Webcam1;
    public VoltageSensor myControlHubVoltageSensor;
    public int VoltageSensor;
    GoBildaPinpointDriver pinpoint;


    @Override
    public void runOpMode() {
        ElapsedTime intakeTimer = new ElapsedTime();

        double left;
        double right;
        double drive;
        
        double turn;
        double max;
        double liftMotorPower = 0;

        ElapsedTime liftTimer = new ElapsedTime();

        boolean rightStickButtonPushed;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        float hsvValuesFloor[] = {0F,0F,0F};
//        int colorSensorState = 0, pixels = 0;

        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotorEx.class, "leftRear");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Only use these if necessary for configuration:
        ascentMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy

        // Define and initialize ALL installed servos.
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(Servo.class, "intake");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        // ==============================================================================
        // * * * * * * * * * * * * PINPOINT CONFIGURATION * * * * * * * * * * * * * * * *
        // ==============================================================================

        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));


        // Send telemetry message to signify robot waiting;

        telemetry.addLine("__ Qualifier - Teleop Code Initialized");
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.addData("Current Voltage: ", myControlHubVoltageSensor.getVoltage());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        basketServo.setPosition(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentTime.reset();

            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

// ===============================================================================
// * * * * * * * * * * * * * * * DRIVER CONTROLLER * * * * * * * * * * * * * * * *
// ===============================================================================


// ===================== ASCENT MOTOR =====================
            if (gamepad2.left_trigger > 0) {
                ascentMotor1.setPower(1);
                ascentMotor2.setPower(-1);
            } else if (gamepad2.right_trigger > 0) {
                ascentMotor1.setPower(-1);
                ascentMotor2.setPower(1);
            } else {
                ascentMotor1.setPower(0);
                ascentMotor2.setPower(0);
            }


// ===================== EXTENSION MOTOR =====================
            if (gamepad2.dpad_up) {
                // Extension motor out
                extensionMotor.setPower(0.8);
            } else if (gamepad2.dpad_down) {
                // Extension motor in
                extensionMotor.setPower(-0.8);
            } else {
                extensionMotor.setPower(0);
            }

// ===================== INTAKE SERVOS =====================
            if (gamepad1.x) {
                intake.setPosition(0.5); // opens intake claw
            } else if (gamepad1.b) {
                intake.setPosition(0.1); // closes intake claw
            }

            switch (intakeDropStateMachine) {
                case 0: {
                    if (gamepad1.left_trigger > 0) { //check for first button hit
                        PivotServo.setPosition(0.5);
                        intake.setPosition(0.5); // opens intake claw
                        intakeTimer.reset();
                        intakeDropStateMachine++;
                    }
                    break;
                }
                case 1: {
                    if (intakeTimer.milliseconds() >= 250) { // if timer is greater than 450 milisec
                        intake.setPosition(0.5); // opens intake claw
                        intakeTimer.reset();
                        intakeDropStateMachine++;
                    }
                    break;
                }
                case 2: {
                    if (intakeTimer.milliseconds() >= 450) { // if timer is greater than 450 milisec
                        PivotServo.setPosition(0);
                        intake.setPosition(0.5); // opens intake claw
                        // Drop Pivot Servo
                        intakeDropStateMachine = 0;
                    }
                    break;
                }
            }

            switch (intakeReturnStateMachine) {
                case 0: {
                    if ((gamepad1.right_trigger > 0) && (intakeTimer.milliseconds() > 250)) { //check for first button hit, but in the case of short circuit back to state 0, make a delay for debouncing
                        intake.setPosition(0.25); // closes intake claw
                        intakeTimer.reset();
                        intakeReturnStateMachine++;
                    }
                    break;
                }
                case 1: {
                    if (intakeTimer.milliseconds() >= 500) { // if timer is greater than 500 milisec
                        PivotServo.setPosition(.86); //raises claw to point to robot
                        intakeTimer.reset();
                        intakeReturnStateMachine++;
                    }
                    break;
                }
                case 2: {
                    if (intakeTimer.milliseconds() >= 1000) { // if timer is greater than 600 milisec
                        if (extensionMotor.getCurrentPosition() < 15) {
                            // Return Pivot Servo
                            intake.setPosition(0.5); // opens intake claw
                            intakeReturnStateMachine++;
                        } else {
                            intake.setPosition(0.25); //closes intake claw
                            intakeReturnStateMachine = 0;
                        }
                        intakeTimer.reset();
                    }
                    break;
                }
                case 3: {
                    if (intakeTimer.milliseconds() >= 400) { // if timer is greater than 400 milisec
                        intakeTimer.reset();
                        intakeReturnStateMachine++;
                    }
                    break;
                }
                case 4: {
                    if (intakeTimer.milliseconds() >= 250) { // if timer is greater than 450 milisec
                        PivotServo.setPosition(0.5); // pivot arm away from basket
                        intakeTimer.reset();

                        intakeReturnStateMachine = 0;
                    }
                    break;
                }
            }

            double dpad_y = 0, dpad_x = 0;
            if (gamepad1.dpad_left) {
                dpad_x = -2;
            }
            if (gamepad1.dpad_right) {
                dpad_x = 2;
            }
            if (gamepad1.dpad_up) {
                dpad_y = -2;
            }
            if (gamepad1.dpad_down) {
                dpad_y = 2;
            }

            double r, robotAngle, rightX;
            r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            scaleFactor = 1;


            if ((liftMotor.getCurrentPosition() > 200) || (extensionMotor.getCurrentPosition() > 300)) {
                scaleTurningSpeed = .4;
            } else {
                scaleTurningSpeed = .8;
            }

            // TODO: 2/5/2025 modify motor speeds to make sure the robot strafes straight
            // Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {
                v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1); //*1.2);
                rightFront.setPower(v2); //* 1.15);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }

            liftMotorPower = 0;
            if (gamepad2.right_stick_y < -0.03) { //checks out
                telemetry.addData("right_stick_y < -0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = 0.5; //.setPower(0.5);
            } else if (gamepad2.right_stick_y > 0.03) {
                telemetry.addData("right_stick_y < -0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = -0.3; //liftMotor.setPower(-0.4);
            }
            else if (gamepad2.left_stick_y < -0.03) { //checks out
                telemetry.addData("left_stick_y < -0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = 1; //.setPower(0.5);
            } else if (gamepad2.left_stick_y > 0.03) {
                telemetry.addData("left_stick_y > -0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = -1; //liftMotor.setPower(-0.4);
            }
            liftMotor.setPower(liftMotorPower);

// ===================== CLAW =====================
            // FIND CLAW SET POSITIONS!!!
            if (gamepad2.left_bumper) {
                claw.setPosition(.7); // Open claw
            }
            else if (gamepad2.right_bumper) {
                claw.setPosition(.9);
            }

// ===================== BASKET SERVO =====================
            if (gamepad2.a) {
                basketServo.setPosition(0); // Return basket servo
            }
            else if (gamepad2.y) {
                basketServo.setPosition(1); // Dump basket servo
            }


            // ==================================== TELEMETRY =========================================
            // Send telemetry message to signify robot running;
            // For configuration SET UP TELEMETRY FOR EACH ATTACHMENT !!!

            telemetry.addData("currentTime", currentTime.milliseconds());
            // ======================= MOTORS =======================
            telemetry.addData("leftFront: ", leftFront.getCurrentPosition());
            telemetry.addData("leftRear: ", leftBack.getCurrentPosition());
            telemetry.addData("rightRear: ", rightBack.getCurrentPosition());
            telemetry.addData("rightFront: ", rightFront.getCurrentPosition());
            telemetry.addData("v1", v1);
            telemetry.addData("v2", v2);
            telemetry.addData("v3", v3);
            telemetry.addData("v4", v4);

            telemetry.addData("liftMotor pos: ", liftMotor.getCurrentPosition());
            telemetry.addData("liftMotor spd: ", liftMotor.getPower());
            telemetry.addData("extensionMotor ", extensionMotor.getCurrentPosition());

            // ======================= SERVOS =======================
            telemetry.addData("PivotServo: ", PivotServo.getPosition());
            telemetry.addData("intakeServo: ", intake.getPosition());
            telemetry.addData("claw: ", claw.getPosition());
            telemetry.addData("basketDumpStateMachine", basketDumpStateMachine);

            // ======================= PINTPOINT ODOMETRY WHEELS =======================
            telemetry.addData("X coordinate (DRIVE)", pose2D.getX(DistanceUnit.INCH));
            telemetry.addData("Y coordinate (STRAFE)", pose2D.getY(DistanceUnit.INCH));
            telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }
    }
    public void liftMotor_Claw_Reset () {
        claw.setPosition(0); // leave claw open
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void liftMotor_basketServo_Reset () {
        basketServo.setPosition(0); // return basket
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void basket_place_sequence () {
        extensionMotor.setTargetPosition(85); // find the set position for the 1 inch adjustment
        basketServo.setPosition(1); // dump basket
        sleep(500);
        basketServo.setPosition(0); // pull back basket
        extensionMotor.setTargetPosition(0); // return extension motor
    }
    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

}

//public class Claw extends Subsystem {
//    public static final Claw INSTANCE = new Claw();
//    private  Claw() { }
////    private ServoImplEx servo = new ServoImplEx("claw_servo");
//    private Servo servo = new Servo("claw_servo");
//
//    public Command open = new ServoToPosition(servo, 0.1).requires(this);
//    public Command close = new ServoToPosition(servo, 0.2).requires(this);
//}