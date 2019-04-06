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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This is NOT an opmode.
 *
 * This class handles all autonomous modes.  It is called by opmodes InconceivableAuton and InconceivableDepot.
 *
 */
public class AutonRORHangDetect {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    HardwareROR robot   = new HardwareROR();   // Use a Pushbot's hardware
    RORDetection detect = new RORDetection();


    public enum autonPosition {
        CRATER, DEPOT;
        }

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private Telemetry hwTelemetry;

    LinearOpMode opmode;

    /* Constructor */
    public AutonRORHangDetect(){

    }
    private autonPosition currentAutonPosition;

    // TEST MODE
    // set to false for a normal run.
    private boolean testMode = false;

    private enum samplePositions {
        LEFT, CENTER, RIGHT, NONE;
    }
    private samplePositions samplePositionFound;

//    private double batteryVoltage;
//    private double robot.turnPower = .45;
//    private double robot.drivePower = .40;

    // heading adjustment based on hang.
    private double headingAdjust = 5.0;

    // headingAdjust for starting after hang & park.
    //private double headingAdjust = -35.0;

    // headingAdjust for deploy marker & park only.
    //private double headingAdjust = 45.0;

    private int intakeEncoderPos;

    private int goldPositionFromHang = -1;

    public void autonRun(HardwareMap ahwMap, LinearOpMode opMode, Telemetry telemetry, autonPosition position, boolean testMode ) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        this.opmode = opMode;

        hwTelemetry = telemetry;

        currentAutonPosition = position;
        this.testMode = testMode;

        double headingTarget;

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(ahwMap, this.opmode, telemetry, testMode);
        detect.init(ahwMap, this.opmode, telemetry, robot, testMode);

        //Initialize imaging code.

        detect.initVuforia();
        detect.initTfod();
        detect.turnOnFlash();

        robot.initNavigationIMU();

        // Read the current heading.
        robot.resetNavigationIMU();

        robot.adjustPowerByVoltage();


        Log.d("12357: Turn power:", Double.toString(robot.turnPower));
        Log.d("12357: Drive power:", Double.toString(robot.drivePower));
        if (currentAutonPosition == autonPosition.DEPOT){
            Log.d("Current Auton Position:", "DEPOT");
        }
        else
        if (currentAutonPosition == autonPosition.CRATER) {
            Log.d("Current Auton Position:", "CRATER");
        }

//        intakeEncoderPos = 0;
//        robot.intakeLift.setTargetPosition(intakeEncoderPos);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status:  Ready to run position:", currentAutonPosition);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        this.opmode.waitForStart();

//        robot.intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.intakeLift.setPower(1);

        if (true && opMode.opModeIsActive()) {

            // Decide where gold mineral is located.
            boolean done = false;
            int goldX = -1;

            runtime.reset();
            samplePositionFound = samplePositions.NONE;
            while (!done && opmode.opModeIsActive() && (runtime.seconds() < 4.0)) {
//                goldX = detect.getGoldPosition();
//                if (goldX > -1) {
//                    Log.d("SamplePos", Integer.toString(goldX));
//                    if (goldX < 445) {
//                        samplePositionFound = samplePositions.LEFT;
//                    } else if(goldX >= 446 && goldX < 460){
//                        samplePositionFound = samplePositions.CENTER;
//                    } else {
//                        samplePositionFound = samplePositions.RIGHT;
//                    }
//
//                    done = true;
//                }
                String pos = detect.goldLocationStr();
                if(pos.equals("Left")) {
                    samplePositionFound = samplePositions.LEFT;
                } else if(pos.equals("Center")) {
                    samplePositionFound = samplePositions.CENTER;
                } else {
                    samplePositionFound = samplePositions.RIGHT;
                }
            }
            if (samplePositionFound == samplePositions.NONE) {
                samplePositionFound = samplePositions.LEFT;
            }

            Log.d("12357:Sample Found: ", String.valueOf(samplePositionFound));

            if (testMode) {
                robot.waitForKey("After detect");
            }
            detect.turnOffFlash();

            //Keep the intake in
//            robot.setIntakeLiftPower(-0.2f);

            // Land.
            // Just this one in auton.
            //robot.hangForCounts(-1, 9525);
            robot.sendHangLiftToPos(3100, -1.0f, 2);
            // Drive Forward a tiny bit
            robot.driveCountsMantainHeading(1, 150, robot.adjustHeading());

            // Strafe away from lander
            robot.encoderStrafeNew(0.75f, 20, 5);
            //robot.stopMotors();

            //if (1==1) return;

            robot.inconceivableWait(0.5);
            //robot.hangForCounts(1, 200);
            //robot.inconceivableWait(0.2);

            robot.driveCountsMantainHeading(0.75f, 1500, robot.adjustHeading());


            if (testMode) {
                robot.waitForKey("After drop");
            }


            // Twist
            robot.turnToHeadingGlobal(0.75f, (robot.adjustHeading() + 34.5), 5);


            if (testMode) {
                robot.waitForKey("After turnToHeading");
            }


            // Move the Intake Forward
            robot.moveIntakeToPos(-555, 0.5f, 3);
            robot.waitMills(500);

            robot.intakeDown();
            robot.waitMills(750);
            robot.sweeperPower(-1.0f);
            robot.waitMills(1500);
            robot.sweeperPower(0.0f);


            if (testMode) {
                robot.waitForKey("After Moving Intake Out.");
            }

//            robot.intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while(opMode.opModeIsActive() && robot.intakeLift.getCurrentPosition() < 0) {
                robot.intakeLift.setPower(-0.5f);
            }
            robot.intakeLift.setPower(0.0f);
//            robot.intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.intakeLift.setPower(1);
            robot.intakeUp();

            if (testMode) {
                robot.waitForKey("After Moving Intake Back In.");
            }

            robot.turnToHeadingGlobal(0.5f, 5, 3);

            if (testMode) {
                robot.waitForKey("Finished Turn Towards Lander");
            }

            robot.driveCountsMantainHeading(-1.0f, 2000, robot.adjustHeading());

            if (testMode) {
                robot.waitForKey("At Lander Turn towards sample");
            }

            robot.turnToHeadingGlobal(0.5f, -87.5, 3);

            if (testMode) {
                robot.waitForKey("Lined up for sampling");
            }


        }


        // For testing, force sample.
        //samplePositionFound = samplePositions.RIGHT;



        // Knock off gold mineral
        knockGoldMineral();

        if (testMode){
            robot.waitForKey("After knockGoldMineral");
        }




        // On the depot side deposit item.

        driveToDepotDeployPark(currentAutonPosition);

        if (testMode) {
            robot.waitForKey( "After Drive to Depot & park");
        }


        hwTelemetry.update();

        //robot.hangForCounts(.7, 9780);

        // Temporary return
        if (1==1) return;
        return;


    }

    private void knockGoldMineral(){

        //RIGHT
        if (samplePositionFound == samplePositions.RIGHT){
            Log.d("12357:Knock Right", "");

            robot.driveCounts(robot.drivePower, 800);
            robot.turnToHeadingGlobal(robot.turnPower, 50.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(robot.drivePower, 800, 50.0);
            robot.driveCountsMantainHeading(-robot.drivePower, 600, 50.0);
            robot.turnToHeadingGlobal(robot.turnPower, 0.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(-robot.drivePower, 2200, 0.0 );
            robot.turnToHeadingGlobal(robot.turnPower, -046.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(-robot.drivePower, 450, -46.0);

        }
        // CENTER
        else if (samplePositionFound == samplePositions.CENTER){
            Log.d("12357:Knock Center", "");
            robot.turnToHeadingGlobal(robot.turnPower, 70.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(robot.drivePower, 1200, 70.0);
            robot.driveCountsMantainHeading(-robot.drivePower, 400, 70.0 );

            robot.turnToHeadingGlobal(robot.turnPower, 0.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(-robot.drivePower, 1600, 0.0);
            robot.turnToHeadingGlobal(robot.turnPower, -046.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(-robot.drivePower, 500, -46.0);


        }
        // LEFT
        else {
            Log.d("12357:Knock Left", "");
            robot.turnToHeadingGlobal(robot.turnPower, 110.0, 2.0, 1.5, headingAdjust);
            robot.driveCountsMantainHeading(robot.drivePower, 1200, 110.0);
            robot.driveCountsMantainHeading(-robot.drivePower, 400, 110.0);

            robot.turnToHeadingGlobal(robot.turnPower, 0.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(-robot.drivePower, 1300, 0.0);
            robot.turnToHeadingGlobal(robot.turnPower, -046.0, 2.0, 1.0, headingAdjust);
            robot.driveCountsMantainHeading(-robot.drivePower, 500, -46.0);
        }


    }


    /*
    *
    *   This method calculates the needed sign of the power for turn to heading prior to depositing the marker, and then calls "robot.turnToHeading".
    *
    *
    *
    */

    private void turnToHeading(double power, double targetHeading, double timeOutSeconds) {
        double accuracy = 3.0;
        turnToHeading( power, targetHeading, timeOutSeconds, accuracy);
    }
    private void turnToHeading(double power, double targetHeading, double timeOutSeconds, double accuracy){


        robot.turnToHeadingGlobal(power, targetHeading, timeOutSeconds, accuracy, headingAdjust);
        robot.inconceivableWait(0.1);
    }



    private void driveToDepotDeployPark(autonPosition position){
        Log.d("12357: driveToDep", "Start");

        //adjustPowerByVoltage();

        double targetHeading = -45.0;

        // CRATER
        if (position == autonPosition.CRATER){
            // Turn to face
            targetHeading = -135;

        }
        // DEPOT side
        else {
            targetHeading = 45;

        }
        turnToHeading(robot.turnPower, targetHeading, 2.0, 3.0 );


        double turnDelta = 0.0;
        double turnDelta2 = 0.0;
        if (position == autonPosition.CRATER){
            turnDelta = -5.0;
            turnDelta2 = -10.0;
        }
        else {
            turnDelta = 2.0;
            turnDelta2 = 5.0;
        }

        if (position == autonPosition.CRATER) {

            turnToHeading(robot.turnPower, targetHeading + turnDelta2, 2.0, 1.0 );
            robot.driveCountsMantainHeading(robot.drivePower, 300, targetHeading + turnDelta2);
//            turnToHeading(robot.turnPower, targetHeading + turnDelta2, 2.0, 1.0 );
//            robot.driveCounts(robot.drivePower, 100);
            //robot.driveCounts(1.0, 200);


            // Temporary take out tilt.
            //robot.tiltStart(-0.1);
            turnToHeading(robot.turnPower, targetHeading + turnDelta2, 2.0, 1.0 );
        }
        else
        {
            turnToHeading(robot.turnPower, targetHeading + turnDelta2, 2.0, 1.0 );
            robot.driveCountsMantainHeading(robot.drivePower, 500, targetHeading + turnDelta2);
//            turnToHeading(robot.turnPower, targetHeading + turnDelta2, 2.0, 1.0 );
//            robot.driveCounts(robot.drivePower, 200);
            //robot.driveCounts(1.0, 200);

            // Temporary take out tilt.
            //robot.tiltStart(-0.1);
            turnToHeading(robot.turnPower, targetHeading + turnDelta2, 2.0, 1.0 );
        }

        // Lower Arm.
        runtime.reset();
        boolean done = false;


        robot.outTakeForTime(2.0, 0.7);


        if (position == autonPosition.CRATER) {
            turnDelta = -2.0;
            turnDelta2 = -5.0;
        }
        else {
            turnDelta = 2.0;
            turnDelta2 = 7.0;
        }

        if (position == autonPosition.CRATER) {
            turnToHeading(robot.turnPower, targetHeading, 2.0, 1.5 );
            robot.driveCounts(-robot.drivePower, 500);
            turnToHeading(robot.turnPower, targetHeading - turnDelta, 2.0, 1.5 );
            robot.driveCounts(-robot.drivePower, 300);
           turnToHeading(robot.turnPower, targetHeading - turnDelta2, 2.0, 1.5 );
            robot.driveCounts(-robot.drivePower, 300);
        }
        else {
            robot.driveCounts(-robot.drivePower, 200);
            turnToHeading(robot.turnPower, targetHeading - turnDelta2, 2.0, 1.5 );
            robot.driveCountsMantainHeading(-robot.drivePower, 1000, targetHeading - turnDelta2);
        }

        // Deploy Marker
        if ( 1 == 1 ) {

            if (testMode) {
                robot.waitForKey("End of deploy marker");
            }
        }

        return;
    }

}
