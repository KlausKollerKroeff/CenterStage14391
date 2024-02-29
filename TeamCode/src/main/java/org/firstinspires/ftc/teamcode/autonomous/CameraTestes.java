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

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "CameraTestes", group = "Autonomous")
public class CameraTestes extends LinearOpMode {

    OpenCvWebcam webcam = null;
    String pos;

    public static final double DELAY = 0.5;
    public static final double DELAYGRANDE = 2;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //intake
    public class Intake {
        private DcMotor intakeMotor;

        private CRServo intakeServo;

        public Intake (HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            intakeServo = hardwareMap.get(CRServo.class, "servoIntake");
        }

        public class StartIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeServo.setPower(-1);
                    intakeMotor.setPower(-1);
                    initialized = true;
                }

                intakeServo.setPower(-1);
                intakeMotor.setPower(-1);

                return false;
            }
        }
        public Action StartIntake() {
            return new StartIntake();
        }

        /////////////////////////////////////////////////////////////////////////

        public class StopIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeServo.setPower(0);
                    intakeMotor.setPower(0);
                    initialized = true;
                }

                intakeServo.setPower(0);
                intakeMotor.setPower(0);

                return false;
            }
        }
        public Action StopIntake() {
            return new StopIntake();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //pinça
    public class Claw {
        private Servo garra;

        public Claw(HardwareMap hardwareMap) {
            garra = hardwareMap.get(Servo.class, "pincaEsquerda");
        }

        public class CloseClaw implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    garra.setPosition(0.5);
                    initialized = true;
                }

                garra.setPosition(0.5);


                double pos = garra.getPosition();

                packet.put("ClawPos", pos);
                return false;
            }
        }
        public Action CloseClaw() {
            return new CloseClaw();
        }

        //////////////////////////////////////////////////////////////////

        public class OpenClaw implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    garra.setPosition(0);
                    initialized = true;
                }

                garra.setPosition(0);


                double pos = garra.getPosition();

                packet.put("ClawPos", pos);
                return false;
            }
        }
        public Action OpenClaw() {
            return new OpenClaw();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //braço
    public class Lift {
        private DcMotorEx lift;

        private Servo pulsoServo;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "braço");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pulsoServo= hardwareMap.get(Servo.class, "pulsoOuttake");

        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    telemetry.addData("INICIALIZADO LIFT:", true);
                    lift.setDirection(DcMotorSimple.Direction.REVERSE);
                    lift.setTargetPosition(1025);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setVelocity(750);
                    initialized = true;
                }

                lift.setTargetPosition(1025);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setVelocity(750);

                double pos = lift.getCurrentPosition();

                packet.put("liftPos", pos);
                if(lift.isBusy()) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        ////////////////////////////////////////

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setDirection(DcMotorSimple.Direction.REVERSE);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setVelocity(750);
                    initialized = true;
                }

                lift.setTargetPosition(0);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setVelocity(750);
                pulsoServo.setPosition(0.18);

                double pos = lift.getCurrentPosition();
                telemetry.addData("posicao", pos);
                packet.put("liftPos", pos);
                if (lift.isBusy()) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //pulso
    public class PulsoOuttake {
        private Servo pulsoServo;

        public PulsoOuttake(HardwareMap hardwareMap) {
            pulsoServo = hardwareMap.get(Servo.class, "pulsoOuttake");

        }

        public class PulsoUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    telemetry.addData("INICIALIZADO PULSO:", true);
                    pulsoServo.setPosition(1);
                    initialized = true;
                }

                pulsoServo.setPosition(1);


                double pos = pulsoServo.getPosition();

                packet.put("PulsoPos", pos);
                return false;
            }
        }

        public Action PulsoUp() {
            return new PulsoUp();
        }
    }

        @Override
        public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);



        webcam.setPipeline(new Pipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.5, 60, Math.toRadians(270)));
        Lift lift = new Lift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        PulsoOuttake pulsoOuttake = new PulsoOuttake(hardwareMap);

        Action trajectoryActionM = drive.actionBuilder(drive.pose)
                //Meio
                //////////////////////////////////////////////
                .afterTime(0, claw.CloseClaw())
                .waitSeconds(DELAY)
                .afterTime(1.5, lift.liftUp())
                .waitSeconds(DELAYGRANDE)
                .afterTime(1, pulsoOuttake.PulsoUp())
                .afterTime(5, claw.OpenClaw())
                .waitSeconds(DELAY)
                .afterTime(5, lift.liftDown())
                .waitSeconds(DELAY)
                .afterTime(0, claw.CloseClaw())
                .build();

        Action trajectoryActionE = drive.actionBuilder(drive.pose)
                //Esquerda
                //////////////////////////////////////////////////
                .afterTime(0, claw.CloseClaw())
                .waitSeconds(DELAY)
                .afterTime(1.5, lift.liftUp())
                .waitSeconds(DELAYGRANDE)
                .afterTime(1, pulsoOuttake.PulsoUp())
                .afterTime(5, claw.OpenClaw())
                .waitSeconds(DELAY)
                .afterTime(5, lift.liftDown())
                .waitSeconds(DELAY)
                .afterTime(0, claw.CloseClaw())
                .build();

        Action trajectoryActionD = drive.actionBuilder(drive.pose)
                //Direita Vermelho
                //////////////////////////////////////////////////
                .afterTime(0, claw.CloseClaw())
                .waitSeconds(DELAY)
                .afterTime(1.5, lift.liftUp())
                .waitSeconds(DELAYGRANDE)
                .afterTime(1, pulsoOuttake.PulsoUp())
                .afterTime(5, claw.OpenClaw())
                .waitSeconds(DELAY)
                .afterTime(5, lift.liftDown())
                .waitSeconds(DELAY)
                .afterTime(0, claw.CloseClaw())
                .build();
        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;
        Action trajectoryActionChosen;
        if (pos == "Meio") {
            trajectoryActionChosen = trajectoryActionM;
        } else if (pos == "Esquerda") {
            trajectoryActionChosen = trajectoryActionE;}
        else {
            trajectoryActionChosen = trajectoryActionD;
        }

        Actions.runBlocking(trajectoryActionChosen);
    }
    class Pipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(255.0, 255.0, 255.0);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipeline rodando");

            Rect leftRect = new Rect(0, 400, 250, 250);
            Rect rightRect = new Rect(1000, 400, 250, 250);
            Rect midRect = new Rect(570, 400, 150, 150);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);
            Imgproc.rectangle(output, midRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);
            Core.extractChannel(midCrop, midCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            telemetry.addData("Direita", leftavgfin);
            telemetry.addData("Meio", midavgfin);
            telemetry.addData("Esquerda", rightavgfin);
            telemetry.update();

            if (leftavgfin > rightavgfin && leftavgfin > midavgfin){
                telemetry.addLine("Esquerda");
                pos = "Esquerda";

            }else if (rightavgfin > midavgfin){
                telemetry.addLine("Direita");
                pos = "Direita";
            }else{
                telemetry.addLine("Meio");
                pos = "Meio";
            }

            return (output);
        }
    }
}
