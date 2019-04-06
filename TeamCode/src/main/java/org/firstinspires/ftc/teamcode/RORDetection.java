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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;

import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;


/**
* This class contains all data and methods necessary for object detection.
 */
public class RORDetection
{

    private Telemetry detectionTelemetry;
    private ElapsedTime runtime = new ElapsedTime();

    LinearOpMode opmode;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";



    public double hangStartPosition = 0;





    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    private static final String VUFORIA_KEY = "AblsY3//////AAABmXQvZpcpzEg5vTOTdVfuI4Rrdxd46KSBiTBaWcwsgHAF2EFFntXT89U+nDwImiSoaCtnCcGKOyfuW5FufviI9r25AWEr+hM0FTxt2SiVYJ3AXQ15WHdU2/yCgBHaRyAEs029SuzYwvoD1htpPqzT6LMeu/Mvq5Ayhpc3au0cBwY3wzsWkma/RpdHvODMCFD4cd1R7lVP4wF212XB1YF0+0p7CLfDzJ4HgcJZ+/7UJvcrp1sFsY0+jgWvIMO6HAwsC778I5zk9XnOkYzI7jOmLlR40sMwabYNaId5rYHN1aGIG5pwWm67RPOQA90pM5Go/lO4yY0vFIFpmnWmTKr7CzMQPI2PCq87k3DIDkaCszVq";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private HardwareROR robot;
    private boolean detectionTestMode = false;

    /* Constructor */
    public RORDetection(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode, Telemetry telemetry, HardwareROR robot, boolean testMode ) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        this.opmode = opMode;

        detectionTelemetry = telemetry;

        this.robot = robot;
        detectionTestMode = testMode;
        
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        Parameters parameters = new Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL );

        if (tfod != null) {
            tfod.activate();
        }
    }

    public int goldMineralX = -1;
    public double goldMineralConfidence = -1.0;
    public int silverMineral1X = -1;
    public int silverMineral2X = -1;
    public int goldMineralLeft = -1;
    public int goldMineralRight = -1;
    public int goldMineralTop = -1;
    public int goldMineralBottom = -1;
    public int goldMineralWidth = -1;
    public int goldMineralHeight = -1;
    public double goldMineralAngle = -1;


    public int getGoldPosition(){

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        runtime.reset();
        int i = 0;

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            if (updatedRecognitions.size() > 0 ) {
                detectionTelemetry.addData("# Object Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    detectionTelemetry.addData("Label: ", recognition.getLabel());

                    // Be sure this is a gold block.  The reference to "getLeft" makes sure we are not seeing objects out of the arena.
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {

                        goldMineralTop =  (int) recognition.getTop();
                        goldMineralBottom = (int) recognition.getBottom();
                        goldMineralLeft = (int) recognition.getLeft();
                        goldMineralRight = (int) recognition.getRight();
                        goldMineralWidth = (int) recognition.getWidth();
                        goldMineralHeight = (int) recognition.getHeight();
                        goldMineralConfidence = recognition.getConfidence();
                        goldMineralAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);

                        Log.d("12357: GoldDet Right: ", Integer.toString(goldMineralRight));
                        Log.d("12357: GoldDet Left: ", Integer.toString(goldMineralLeft));
                        Log.d("12357: GoldDet Top: ", Integer.toString(goldMineralTop));
                        Log.d("12357: GoldDet Bottom: ", Integer.toString(goldMineralBottom));
                        Log.d("12357: GoldDet Width: ", Integer.toString(goldMineralWidth));
                        Log.d("12357: GoldDet Height: ", Integer.toString(goldMineralHeight));
                        Log.d("12357: GoldDet Conf: ", Double.toString(goldMineralConfidence));
                        Log.d("12357: GoldDet Angle: ", Double.toString(goldMineralAngle));




                        detectionTelemetry.addData("Gold Top: ",goldMineralTop);
                        detectionTelemetry.addData("Gold Bottom: ", goldMineralBottom);
                        detectionTelemetry.addData("Gold Left", goldMineralLeft);
                        detectionTelemetry.addData("Gold Right", goldMineralRight);
                        detectionTelemetry.addData("Gold Width", goldMineralWidth);
                        detectionTelemetry.addData("Gold Height", goldMineralHeight);
                        detectionTelemetry.addData("Gold Confidence: ", goldMineralConfidence);
                        detectionTelemetry.addData("Gold Angle: ", goldMineralAngle);

/*                        if ((goldMineralConfidence > 0.4) &&
                                (goldMineralWidth > 90) && (goldMineralWidth < 500) &&
                                (goldMineralHeight > 90) && (goldMineralHeight < 500)) {*/

//                        if ((goldMineralConfidence > 0.3) && (goldMineralLeft > 150)) {
                        if (goldMineralConfidence > 0.3) {
                                ///*(goldLeft > 250)*/) {
                            // Camera is vertical.  "Top" is actually left in this orientation of the camera.

                            goldMineralX = goldMineralLeft;
                            detectionTelemetry.addData("Accepted Gold Detection: ", goldMineralX);
                            detectionTelemetry.update();
                            Log.d("12357: ", "Accepted gold");
                            /*if (detectionTestMode){
                                robot.waitForKey("Accepted gold "+ Integer.toString(goldMineralX));
                            }*/

                        } else{
                            Log.d("12357: ", "Rejected gold");

                            detectionTelemetry.addData("Rejected Gold Detection","");
                            if (goldMineralConfidence <= 0.4) detectionTelemetry.addData("Confidence","");
                            if ( (goldMineralWidth < 90) || (goldMineralWidth > 250)) detectionTelemetry.addData("Width: ", goldMineralWidth);
                            if ( (goldMineralHeight < 90) || (goldMineralHeight >= 250)) detectionTelemetry.addData("Height: ", goldMineralHeight);
                            //if (goldLeft <= 250) detectionTelemetry.addData("Left: ", goldLeft);
                            detectionTelemetry.update();
                            /*if (detectionTestMode){
                                robot.waitForKey("Rejected gold "+ Integer.toString(goldMineralX));
                            }*/
                        }
                    }
//                    else if (silverMineral1X == -1) {
//                        // "Top" is actually left in this orientation of the camera.
//                        silverMineral1X = (int) recognition.getTop();
//                        detectionTelemetry.addData("S1 Top: ", recognition.getTop());
//                        detectionTelemetry.addData("S1 Width", recognition.getWidth() );
//                        detectionTelemetry.addData("S1 Height", recognition.getHeight() );
//                        detectionTelemetry.addData("S1 Confidence: ", recognition.getConfidence());
//
//                    }
//                    else {
//                        // "Top" is actually left in this orientation of the camera.
//                        silverMineral2X = (int) recognition.getTop();
//                        detectionTelemetry.addData("S2 Top: ", recognition.getTop());
//                        detectionTelemetry.addData("S2 Width", recognition.getWidth() );
//                        detectionTelemetry.addData("S2 Height", recognition.getHeight() );
//                        detectionTelemetry.addData("S2 Confidence: ", recognition.getConfidence());
//                    }

                }
                detectionTelemetry.update();
                if (goldMineralX != -1) {
                    return (goldMineralX);

                }
            }
            detectionTelemetry.update();
        }

        return (goldMineralX);
    }



    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void turnOnFlash(){
        CameraDevice.getInstance().setFlashTorchMode(true);
    }
    public void turnOffFlash(){
        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    String location;
    int timeout = 3;
    //ElapsedTime runtime = new ElapsedTime();
    public String goldLocationStr() {
        runtime.reset();
        ArrayList<Float> avgPos = new ArrayList<Float>();
        float avg = 0.0f;
        float temp = 0;
        while (opmode.opModeIsActive()) {

            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            for(int i = 0; i < updatedRecognitions.size(); i++) {

                float left = updatedRecognitions.get(i).getTop();
                float right = updatedRecognitions.get(i).getBottom();
                float bottom = updatedRecognitions.get(i).getLeft();
                float screenHeight = updatedRecognitions.get(i).getImageWidth();
                float[] dimensions =  {updatedRecognitions.get(i).getWidth(), updatedRecognitions.get(i).getHeight()};
                float confidence = updatedRecognitions.get(i).getConfidence();
                if (updatedRecognitions.get(i).toString().contains("Gold")) {

                    Log.d("5667:","Found, Gold");
                    Log.d("5667:", "Confidence: " + Float.toString(confidence));
                    Log.d("5667:", "Left: " + Float.toString(left));
                    Log.d("5667:", "Right: " + Float.toString(right));
                    Log.d("5667:", "Width: " + Float.toString(dimensions[0]));
                    Log.d("5667:", "Height: " + Float.toString(dimensions[1]));

                    avgPos.add(left);
                    for (int t = 0; t < avgPos.size(); t++) {
                        temp += avgPos.get(t);
                    }
                    avg = temp / avgPos.size();

                    Log.d("5667:", "Left Avg" + Float.toString(avg));

                    if(left > -20 && left < 794) {
                        location = "Left";
                    } else if(left > 785 && left < 1000) {
                        location = "Center";
                    } else {
                        location = "Undetermined";
                    }

                    Log.d("5667:", "Location: " + location);

                    if(confidence > 0.785) {
                        break;
                    }

                }
            }
            if(runtime.seconds() < timeout) {
                location = "Right";
                break;
            }
            if(location != "None") {
                break;
            }
        }
        return location;
    }



}

