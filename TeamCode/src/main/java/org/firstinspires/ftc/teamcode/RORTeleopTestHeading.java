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

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="12357: Teleop Test Heading")
//@Disabled
public class RORTeleopTestHeading extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareROR robot           = new HardwareROR();   // Use a Pushbot's hardware


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        double heading = 0.0;
        double targetHeading = 0.0;

        double turnPower = 0.8;
        double accuracy = 3.0;
        double headingAdjust = 0.0;

        boolean moved = false;

        robot.init(hardwareMap, this, telemetry, false);
        robot.initNavigationIMU();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Ready for start");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Select direction.
            if (gamepad1.a){
                targetHeading = 0.0;
                Log.d("12357: Target ", "Set to 0");
                Log.d("12357: CurrentHead ", Double.toString(robot.getGlobalNavigationHeading()));
                robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0,accuracy, headingAdjust);
                moved = true;
            }
            else if (gamepad1.b){
                Log.d("12357: Target ", "Set to 180");
                targetHeading = 180.0;
                Log.d("12357: CurrentHead: ", Double.toString(robot.getGlobalNavigationHeading()));
                robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                moved = true;
            }

                // Select direction.
            else if (gamepad1.x){
                    targetHeading += 45.0;
                    Log.d("12357: CurrentHead: ", Double.toString(robot.getGlobalNavigationHeading()));
                    Log.d("12357: Turn to: ", Double.toString(targetHeading) + " Converted: " + Double.toString(robot.convertAngleToHeading(targetHeading)));

                    robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                    moved = true;
                }
                else if (gamepad1.y){
                targetHeading -= 45.0;
                Log.d("12357: CurrentHead: ", Double.toString(robot.getGlobalNavigationHeading()));
                Log.d("12357: Turn to: ", Double.toString(targetHeading) + " Converted: " + Double.toString(robot.convertAngleToHeading(targetHeading)));
                robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                moved = true;
            }

            // Left bumper starts left rotation.
            if ( gamepad1.left_bumper ) {
                targetHeading = 5;
                robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                targetHeading = 0;
                boolean done = false;
                while (!done){
                    robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                    telemetry.addData("Target: ", targetHeading);
                    telemetry.addData("Current heading: ", robot.getGlobalNavigationHeading());
                    telemetry.update();
                    robot.inconceivableWait(0.3);

                    targetHeading += 20.0;
                    robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                    telemetry.addData("Target: ", targetHeading);
                    telemetry.addData("Current heading: ", robot.getGlobalNavigationHeading());
                    telemetry.update();
                    robot.inconceivableWait(0.3);
                    targetHeading -= 10.0;

                    if (gamepad1.dpad_up){
                        done = true;
                    }

                }

            }
            // Right bumper starts right rotation.
            else if (gamepad1.right_bumper){
                targetHeading = 5;
                robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                targetHeading = 0;
                boolean done = false;
                while (!done){
                    robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                    telemetry.addData("Target: ", targetHeading);
                    telemetry.addData("Current heading: ", robot.getGlobalNavigationHeading());
                    telemetry.update();
                    robot.inconceivableWait(0.3);

                    targetHeading -= 20.0;
                    robot.turnToHeadingGlobal(turnPower, targetHeading, 3.0, accuracy, headingAdjust);
                    telemetry.addData("Target: ", targetHeading);
                    telemetry.addData("Current heading: ", robot.getGlobalNavigationHeading());
                    telemetry.update();
                    robot.inconceivableWait(0.3);
                    targetHeading += 10.0;

                    if (gamepad1.dpad_up){
                        done = true;
                    }

                }

            }








            if (moved){
                robot.getGlobalNavigationHeading();
                telemetry.addData("Target: ", targetHeading);
                telemetry.addData("Global heading: ", robot.navigationAngles.firstAngle);
                telemetry.update();
                // Pace this loop so jaw action is reasonable speed.
                moved = false;

            }
        }
    }
}
