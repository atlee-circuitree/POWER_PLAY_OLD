package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Covenant Demo", group="Linear Opmode")
public class Covenant_Demo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor armExtend = null;
    private DcMotor feeder = null;
    private Servo kickout = null;
    private CRServo leftDucky = null;
    private CRServo rightDucky = null;
    private CRServo tapeArm = null;
    private Servo odometryLift1 = null;
    private Servo armTurn = null;

    //public final static double ARM_DEFAULT = 0.5; //Unslash this if you want armTurn servo using joystick back
    public final static double ARM_MIN_RANGE = 0.46;
    public final static double ARM_MAX_RANGE = 0.53;

    //double armTurnPosition = ARM_DEFAULT;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        leftArm = hardwareMap.get(DcMotor.class, "left_Arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_Arm");
        armExtend = hardwareMap.get(DcMotor.class, "extend_Arm");
        feeder = hardwareMap.get(DcMotor.class, "feeder");

        armTurn = hardwareMap.get(Servo.class, "armTurn");

        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armTurn.setPosition(ARM_DEFAULT); //Unslash this if you want armTurn servo using joystick back

        //Slow Mode Variables

        double SD = 1;
        double SA = 1;

        //Arm Turn Variables
        //double armTurn = ARM_DEFAULT;  //Unslash this if you want armTurn servo using joystick back
        final double ARM_SPEED = 0.005;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the batter;
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Drive Modes
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Show encoder values on the phone
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Left Dead Encoder", frontLeft.getCurrentPosition());
            telemetry.addData("Right Dead Encoder", backRight.getCurrentPosition());
            telemetry.addData("Rear Dead Encoder", backLeft.getCurrentPosition());
            telemetry.addData("Arm Turn Position", armTurn.getPosition());
            //telemetry.addData("Arm Turn Position", "%.2f", armTurnPosition);
            telemetry.update();

            //Mecanum Drive Code
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            frontLeft.setPower(v1 * SD);
            backLeft.setPower(v3 * SD);
            frontRight.setPower(v2 * SD);
            backRight.setPower(v4 * SD);

            //Controller 1
            if (gamepad1.left_stick_button) {
                SD = .25;
            } else {
                SD = 1;
            }

            //Controller 2
            //Manually turns arm :)
            if (gamepad1.left_trigger > .05) {
                leftArm.setPower(gamepad1.left_trigger);
                rightArm.setPower(gamepad1.left_trigger);

            } else if (gamepad1.right_trigger > .05) {
                leftArm.setPower(-gamepad1.right_trigger);
                rightArm.setPower(-gamepad1.right_trigger);

            } else {
                leftArm.setPower(0);
                rightArm.setPower(0);
            }

            if (gamepad1.b) {
                feeder.setPower(.5); //Turns Feeder Motor Inward

            } else if (gamepad1.a) {
                feeder.setPower(-1); //Turns Feeder Motor Outward

            } else { 
                feeder.setPower(0);
            }

            //Extends and Retracts the arm
            if (gamepad1.x) {
                armExtend.setPower(.3);
            } else if (gamepad1.y) {
                armExtend.setPower(-.3);
            } else {
                armExtend.setPower(0);
            }

            //Turns feeder box (Higher number = left, Lower number = right)
             if (gamepad1.dpad_left) {
                armTurn.setPosition(.53);
            } else if (gamepad1.dpad_right) {
                armTurn.setPosition(.46);
            } else if (gamepad1.dpad_up) {
                armTurn.setPosition(.50);
            }

             //Turns feeder box variably with joystick  //Unslash this section if you want armTurn servo using joystick back.
            /*if (gamepad2.right_stick_x > .05)         // Also slash out button armTurn , it's right above this function.
                armTurnPosition -= ARM_SPEED;
            else if (gamepad2.right_stick_x < -.05)
                armTurnPosition += ARM_SPEED;

            armTurnPosition = Range.clip(armTurnPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
            armTurn.setPosition(armTurnPosition); */

        }
    }

    public int degreesBore(int input) {

        final int COUNTS_PER_BORE_MOTOR_REV = 8192;    // eg: GOBUILDA Motor Encoder
        int COUNTS_TICKS_PER_REV_PER_DEGREE = (COUNTS_PER_BORE_MOTOR_REV) / 360 * 2;

        return COUNTS_TICKS_PER_REV_PER_DEGREE * input;

    }
}