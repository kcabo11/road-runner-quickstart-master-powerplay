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

package org.firstinspires.ftc.teamcode.drive.opmode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftBack     = null;
    public DcMotor  rightBack   = null;
    //public Servo    leftClaw    = null;
    //public Servo    rightClaw   = null;
    public DcMotor liftMotor = null;
    public DcMotor extensionMotor = null;



    Servo servo;

    double clawOffset = 0;
    double scaleTurningSpeed = .8;
    double scaleFactor = .8;

    double lift_power = 0;

    double extension_power = 0;

    int direction = -1;




    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    int claw_state = 0;
    int speed_state = 0;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftMotor    = hardwareMap.get(DcMotor.class, "liftMotor");
        extensionMotor    = hardwareMap.get(DcMotor.class, "extensionMotor");
        servo = hardwareMap.get(Servo.class, "claw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionMotor.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hardwareMap.get(Servo.class, "leftClaw");
        //rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;



            //Servo
            //ROBOT TURNING AND DRIVING SPEED BELOW:
            switch (speed_state) {
                case 0:
                {
                    if (gamepad1.left_bumper) {
                        speed_state = 1;
                        scaleFactor = (.5 * scaleFactor);
                        scaleTurningSpeed = (.98 * scaleTurningSpeed);
                    }
//
//                    }
                    break;
                }

                case 1:
                {
                    if (!gamepad1.left_bumper){
                        speed_state = 2;
                    }
//                    servo.setPosition(0.5);
//                    claw_state = 2;
                    break;
                }

                case 2:
                {
                    if (gamepad1.left_bumper){
                        scaleFactor = (scaleFactor * 2);
                        speed_state = 3;
                        scaleTurningSpeed = (scaleTurningSpeed * (100/98));
                    }
                    break;
                }

                case 3:
                {
                    if (!gamepad1.left_bumper){
                        speed_state = 0;
                    }
                    break;
                }
            }

// When the direction value is reversed this if statement inverts the addition and subtraction for turning.
// Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v3);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {
                final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }

            //
            //
            // L I F T    M O T O R
            //
            //

            int HIGH_JUNC = -4051;
            int MED_JUNC = -2953;
            int LOW_JUNC = -1660;


//            if (gamepad2.a && !liftMotor.isBusy()) {
//                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor.setTargetPosition(LOW_JUNC);
//                liftMotor.setPower(-1);
//            }
//
//            if (gamepad2.x && !liftMotor.isBusy()) {
//                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor.setTargetPosition(MED_JUNC);
//                liftMotor.setPower(-1);
//            }
//
//            if (gamepad2.y && !liftMotor.isBusy()) {
//                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                liftMotor.setTargetPosition(HIGH_JUNC);
//                liftMotor.setPower(-1);
//            }



            //Setting the power to the lift
            if (gamepad2.dpad_down) {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(.1);
            }
            else if (gamepad2.dpad_up) {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(-1);
            }
//            else if (liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//                liftMotor.setPower(0);
//            }
            else {
                if (!liftMotor.isBusy()) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
                    liftMotor.setPower(.1);
                }
            }

            //Robot Extension
//            if (gamepad2.a)
//                lift_power = 1;
//            else if (gamepad2.b)
//                lift_power = -1;
//            else
//                lift_power = 0;
//
//            extensionMotor.setPower(_power);


            //Robot Extension
            if (gamepad1.a)
                extension_power = 1;
            else if (gamepad1.b)
                extension_power = -1;
            else
                extension_power = 0;

            extensionMotor.setPower(extension_power);



            //Servo
            switch (claw_state) {
                case 0:
                {
                    if (gamepad2.left_bumper) {
                        claw_state = 1;
                    }
                    if (gamepad2.right_bumper){
                        claw_state = 3;
                    }
                    break;
                }
                case 1:
                {
                    servo.setPosition(0);
                    claw_state = 2;
                    break;
                }

                case 2:
                {
                    if (!gamepad2.left_bumper){
                        claw_state = 0;
                    }
                    break;
                }

                case 3:
                {
                    servo.setPosition(1.0);
                    claw_state = 4;
                    break;
                }
                case 4:
                {
                    if (!gamepad2.right_bumper){
                        claw_state = 0;
                    }
                    break;
                }
            }


//            if (gamepad1.x){
//                servo.setPosition(0.5);
//            }
//            if (gamepad1.y){
//                servo.setPosition(1.0);
//            }


            // Move both servos to new position.  Assume servos are mirror image of each other.
            //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            //leftClaw.setPosition(MID_SERVO + clawOffset);
            //rightClaw.setPosition(MID_SERVO - clawOffset);


            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "%.2f", servo.getPosition());
            telemetry.addData("claw_state", claw_state);
            telemetry.addData("encoder", liftMotor.getCurrentPosition());
            telemetry.addData("lift motor --pwr", liftMotor.getPower());
            telemetry.addData("lift motor --tgt pos", liftMotor.getTargetPosition());
            telemetry.addData("lift motor --is busy", liftMotor.isBusy());
            telemetry.addData("right bumper", gamepad2.right_bumper);
            telemetry.addData("left bumper", gamepad2.left_bumper);
            telemetry.addData("extension motor power", extensionMotor.getPowerFloat());
            telemetry.addData("servo", servo.getPosition());
            telemetry.addData("scaleFactor", scaleFactor);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);

        }
    }
}
