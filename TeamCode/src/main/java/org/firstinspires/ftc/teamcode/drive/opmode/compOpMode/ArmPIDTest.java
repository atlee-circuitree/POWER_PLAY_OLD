package org.firstinspires.ftc.teamcode.drive.opmode.compOpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.Bases.BaseOpMode;

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

@Config
@TeleOp(name="Arm PID Test", group="Linear Opmode")
public class ArmPIDTest extends BaseOpMode {
    @Override
    public void runOpMode() {

        GetHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            horizArmPIDLoop();
            vertArmPIDLoop();
            angleArmPIDLoop();

            double y_stick = gamepad1.left_stick_y;
            double x_stick = gamepad1.left_stick_x;

            //Field Orientation Code
            double pi = 3.1415926;
            double gyro_degrees = navx_centered.getYaw();
            double gyro_radians = gyro_degrees * pi / 180;
            double temp = y_stick * Math.cos(gyro_radians) + x_stick * Math.sin(gyro_radians);
            x_stick = -y_stick * Math.sin(gyro_radians) + x_stick * Math.cos(gyro_radians);

            /* double temp = y_stick * Math.cos(gyro_radians) + x_stick * Math.sin(gyro_radians);
            x_stick = -y_stick * Math.sin(gyro_radians) + x_stick * Math.cos(gyro_radians); */ //Original code from last year

            //Mecanum Drive Code
            double r = Math.hypot(x_stick, y_stick);
            double robotAngle = Math.atan2(y_stick, -x_stick) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = (r * Math.cos(robotAngle) + rightX);
            final double v2 = (r * Math.sin(robotAngle) - rightX);
            final double v3 = (r * Math.sin(robotAngle) + rightX);
            final double v4 = (r * Math.cos(robotAngle) - rightX);

            frontLeft.setPower(v1 * SD);
            rearLeft.setPower(v3 * SD);
            frontRight.setPower(v2 * SD);
            rearRight.setPower(v4 * SD);

            telemetry.addData("Horiz Arm Target", horizArmTarget);
            telemetry.addData("Horiz Arm Pos", horizArm.getCurrentPosition());
            telemetry.addData("Vert Arm Target", vertArmTarget);
            telemetry.addData("Vert Arm Pos", vertArm.getCurrentPosition());
            telemetry.addData("Angle Arm Target", angleArmTarget);
            telemetry.addData("Angle Arm Pos", angleArm.getCurrentPosition());
            telemetry.update();


            //Controller 1 Auto Tele-op
            //Slows movement
            if (gamepad1.left_stick_button) {
                SD = .25;
            } else {
                SD = 1;
            }

            //Resets NavX heading
            if (gamepad1.back) {
                zeroGyro();
            }

            //Extends horizArm
            if (gamepad1.x) {
                horizArmTarget = 500;
            }

            //Retracts horizArm
            if (gamepad1.a) {
                horizArmTarget = 0;
            }

            /*if (gamepad1.y) {
                horizArmTarget =
            }*/

            //Opens horizClaw
            if (gamepad1.dpad_left) {
                horizClaw.setPosition(HORIZONTAL_CLAW_OPEN);
            }

            //Closes horizClaw
            if (gamepad1.dpad_right) {
                horizClaw.setPosition(HORIZONTAL_CLAW_CLOSE);
            }

            if (gamepad1.right_bumper) {
                angleArmTarget = 200;
            }

            if (gamepad1.left_bumper) {
                angleArmTarget = 0;
            }

            //Moves angleArm up and down
            if (gamepad1.right_trigger > .5) {
                angleArm.setPower(1);
            } else if (gamepad1.left_trigger > .5) {
                angleArm.setPower(-1);
            } else {
                angleArm.setPower(0);
            }

            if (gamepad2.x) {
                vertArmTarget = 500;
            }

            if (gamepad2.a) {
                vertArmTarget = 0;
            }

            //Opens and Closes Transfer Claw
            //Opens transfer claw
            if (gamepad2.y) {
                transferClaw.setPosition(TRANSFER_CLAW_OPEN);
            }

            //Close transfer claw
            if (gamepad2.b) {
                transferClaw.setPosition(TRANSFER_CLAW_CLOSE);
            }

            //Moves transferArmBottom to front
            if (gamepad2.dpad_up) {
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_FRONT);
            }

            //Moves transferArmBottom to back
            if (gamepad2.dpad_down) {
                transferArmBotttom.setPosition(TRANSFER_ARM_BOTTOM_BACK);
            }

            //Moves transferArmTop to front
            if (gamepad2.right_bumper) { //gamepad1.dpad_up
                transferArmTop.setPosition(TRANSFER_ARM_TOP_FRONT);
            }

            //Moves transferArmTop to back
            if (gamepad2.left_bumper) { //gamepad1.dpad_down
                transferArmTop.setPosition(TRANSFER_ARM_TOP_BACK);
            }
        }
    }
}