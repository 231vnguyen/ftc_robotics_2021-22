package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "New Competition Auto", group = "Competition")

public class NewCompAuto extends LinearOpMode {

    BarcodePipeline.FreightFrenzyPipeline pipeline;
    OpenCvCamera webcam;

    //dynamic variables
    private int autoStart = 0;
    private int autoMode = 0;
    private int initialDelay = 0;
    private int barcodeValue = 0;

    private final double    wheelDiameter = 3.77953,
            wheelGearRatio = 1.0,
            wheelMotorTicks = 383.6,
            wheelRPM = 435,
            wheelMaxVelocity = wheelRPM * wheelMotorTicks / 60,
            botDiagonal = 1.4 * Math.sqrt(Math.pow(16.5, 2) + Math.pow(13.375, 2)),
            avgPower = .4;


    //
    private double  currentHeading = 0,
            targetHeading = 0;


    private final double maxVerticalSlideTicks = 384.5 * 4;

    private final double intakeDown = .5;
    private final double intakeUp = .3;
    private final double intakeDrop = .65;

    private final double dropdownDown = .55;
    private final double dropdownUp = .1;
    private final double dropdownHalf = .3;
    private final double dropdownDownLeft = .3;
    private final double dropdownUpLeft = .055;

    private final double right = 1;
    private final double middle = .7;
    private final double left = .33;
    private final double forward = 0;

    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3

    //inverse drive variables
    private boolean xPressed = false;
    private int inverse = 1;

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};

    private int slidePosition = 0;
    private final double[] slideValues = {
            maxVerticalSlideTicks * 1.3, //top level
            maxVerticalSlideTicks * .2, //shared
            maxVerticalSlideTicks * .3, //low level
            maxVerticalSlideTicks * .6, //middle level

    };


    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //create motor/servo objects

    private DcMotor spinny;
    private CRServo intakeServo;
    private DcMotorEx verticalSlide;

    private Servo rotateIntakeServo;
    private Servo rightDropdown;
    private Servo leftDropdown;
    private Servo slideRotation;
    private Servo horizontalServo;

    private ColorSensor color;

    public enum CarouselRedState {
        STARTING_POSITION,
        SHIPPING_COORDINATE,
        SHIPPING_LOCALIZATION,

    }






    public void spinnyspin(double seconds, double power) {
        spinny.setPower(power);
        sleep((long) seconds * 1000);
        spinny.setPower(0);
    }



    public void intakePosition(double position) {
        rotateIntakeServo.setPosition(position);

    }

    public void intakePower(double power, double seconds) {
        intakeServo.setPower(power);
        sleep((long) (seconds * 1000));
    }

    public void senseFreight() {
        rotateIntakeServo.setPosition(intakeDown);
        leftDropdown.setPosition(dropdownDown);
        rightDropdown.setPosition(dropdownDown);
        intakeServo.setPower(1);

        if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 4.5) {
            intakeServo.setPower(1);
            rotateIntakeServo.setPosition(intakeUp);
            leftDropdown.setPosition(dropdownUp);
            rightDropdown.setPosition(dropdownUp);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);



        pipeline = new BarcodePipeline.FreightFrenzyPipeline();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //initialize imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        //imu hardware map




        //create variable to store visual information
        String[] display = {"Carousel-Blue", "Warehouse-Blue", "Carousel-Red",
                "Warehouse-Red", "Deliver Freight", "Park"};
        String[] Barcode = {"Right", "Left", "Outside"};
        //for loop to allow selection of autonomous situation
        for (boolean buttonPressed = false; !gamepad2.a && !opModeIsActive() && !isStopRequested();) {

            //change starting line
            if (gamepad2.dpad_left && autoStart > 0 && !buttonPressed) {
                autoStart--;
                buttonPressed = true;
            } else if (gamepad2.dpad_right && autoStart < 3 && !buttonPressed) {
                autoStart++;
                buttonPressed = true;
                //change automode
            }  /*else if (gamepad1.dpad_down && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad1.dpad_up && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }*/
            //change initial delay
            else if (gamepad2.dpad_down && initialDelay > 0 && !buttonPressed) {
                initialDelay--;
                buttonPressed = true;
            } else if (gamepad2.dpad_up && initialDelay < 25 && !buttonPressed) {
                initialDelay++;
                buttonPressed = true;
            }
            //choose which autonomous mode
            else if (gamepad2.left_bumper && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad2.right_bumper && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }
            //choose barcode position
            else if (gamepad2.square && barcodeValue > 0 && !buttonPressed) {
                barcodeValue--;
                buttonPressed = true;
            } else if (gamepad2.circle && barcodeValue < 2 && !buttonPressed) {
                barcodeValue++;
                buttonPressed = true;
            }

            //wait until buttons are not pressed
            else if (buttonPressed && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up &&
                    !gamepad2.dpad_down && !gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.square && !gamepad2.circle)
                buttonPressed = false;

            //output telemetry
            telemetry.addData("Auto Start (Right/Left Dpad)", display[autoStart]);
            telemetry.addData("Auto Mode (Bumpers)", display[autoMode + 4]);
            telemetry.addData("initialDelay (Up/Down Dpad)", initialDelay);
            telemetry.addData("Barcode Position", Barcode[barcodeValue]);

            telemetry.addData("Analysis1", pipeline.avg1);
            telemetry.addData("Analysis2", pipeline.avg2);
            telemetry.addData("Position", BarcodePipeline.position);
            telemetry.addData("BarcodeValue", barcodeValue);

            //robot position notes
//            if (autoStart == 0 && autoMode == 0) //CAROUSEL BLUE SPIN AND PARK
//                telemetry.addLine("Spinny - Bottom Right");
//            else if (autoStart == 1 && autoMode == 0) //WAREHOUSE BLUE PARK
//                telemetry.addLine("Spinny - Bottom Left");
//            else if (autoStart == 2 && autoMode == 0) //CAROUSEL RED SPIN AND PARK
//                telemetry.addLine("Spinny - Bottom Left");
//            else if (autoStart == 3 && autoMode == 0) //WAREHOUSE RED PARK
//                telemetry.addLine("Spinny - Top Right");






            telemetry.update();

        }

        //setup driving motors

        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        rotateIntakeServo = hardwareMap.get(Servo.class, "rotateIntakeServo");
        rightDropdown = hardwareMap.get(Servo.class, "rightDropdown");
        leftDropdown = hardwareMap.get(Servo.class, "leftDropdown");

        slideRotation = hardwareMap.get(Servo.class, "slideRotation");
        horizontalServo = hardwareMap.get(Servo.class, "horizontalServo");

        verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setTargetPosition(0);
        verticalSlide.setPower(0);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        rightDropdown.setDirection(Servo.Direction.REVERSE);
        leftDropdown.setDirection(Servo.Direction.FORWARD);
        rotateIntakeServo.setDirection(Servo.Direction.FORWARD);
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        color = hardwareMap.colorSensor.get("color");





        if (isStopRequested()) return;

        waitForStart();

        if (position == BarcodePipeline.BarcodePosition.LEFT)
            barcodeValue = 0;
        if (position == BarcodePipeline.BarcodePosition.MIDDLE)
            barcodeValue = 1;
        if (position == BarcodePipeline.BarcodePosition.RIGHT)
            barcodeValue = 2;



        //intake Servo
        verticalSlide.setVelocity(wheelMaxVelocity);

        slideRotation.setPosition(middle);
        rotateIntakeServo.setPosition(.1);
        leftDropdown.setPosition(dropdownUp);
        rightDropdown.setPosition(dropdownUp);

        TrajectorySequence warehouseRedPark = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))

                .strafeRight(30)

                .build();

        TrajectorySequence warehouseBluePark = drive.trajectorySequenceBuilder(new Pose2d(-35, 62, Math.toRadians(270)))

                .strafeLeft(30)

                .build();

        TrajectorySequence carouselRedTop = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))

                .lineToSplineHeading(new Pose2d(-53, -50, Math.toRadians(250)))
                .forward(5)
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(-60, -34, Math.toRadians(180)))


               /* .addTemporalMarker(.5, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(1, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(1.5, () -> {
                    slideRotation.setPosition(right);
                })
                .addTemporalMarker(2, () -> {
                    rotateIntakeServo.setPosition(0);
                })
                .addTemporalMarker(3, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeUp);
                })
                .addTemporalMarker(4.5, () -> {
                    verticalSlide.setTargetPosition(0);
                })*/
                .addTemporalMarker(3.5, () -> {
                    spinny.setPower(-.5);
                })
                .addTemporalMarker(7.7, () -> {
                    spinny.setPower(0);
                })


                .build();

        TrajectorySequence carouselBlueTop = drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-25, -34, Math.toRadians(45)), Math.toRadians(90))
                .waitSeconds(3) //wait for freight dropoffaaa
                .lineToSplineHeading(new Pose2d(-53, -55, Math.toRadians(250)))
                .waitSeconds(3)
                .lineToSplineHeading(new Pose2d(-60, -34, Math.toRadians(180)))


                .addTemporalMarker(.5, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(1, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(1.5, () -> {
                    slideRotation.setPosition(right);
                })
                .addTemporalMarker(2, () -> {
                    rotateIntakeServo.setPosition(0);
                })
                .addTemporalMarker(3, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeUp);
                })
                .addTemporalMarker(4.5, () -> {
                    verticalSlide.setTargetPosition(0);
                })
                .addTemporalMarker(9, () -> {
                    spinny.setPower(-.5);
                })
                .addTemporalMarker(13, () -> {
                    spinny.setPower(0);
                })


                .build();

        TrajectorySequence WarehouseRedTop = drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))

                .splineTo(new Vector2d(0, -34), Math.toRadians(140))


                .splineToSplineHeading(new Pose2d(12, -62, Math.toRadians(180)), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(54.2, -64.3), Math.toRadians(0))

                .lineToSplineHeading(new Pose2d(12, -64, Math.toRadians(180)))

                .splineTo(new Vector2d(0, -34), Math.toRadians(140))


                .splineToSplineHeading(new Pose2d(12, -62, Math.toRadians(180)), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(54.2, -64.3), Math.toRadians(0))

                .lineToSplineHeading(new Pose2d(12, -64, Math.toRadians(180)))

                .splineTo(new Vector2d(0, -34), Math.toRadians(140))


                .splineToSplineHeading(new Pose2d(12, -62, Math.toRadians(180)), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(54.2, -64.3), Math.toRadians(0))










                .addTemporalMarker(0, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(.5, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(1 , () -> {
                    slideRotation.setPosition(forward);
                })
                .addTemporalMarker(1.7, () -> {
                    rotateIntakeServo.setPosition(intakeDrop);
                })
                .addTemporalMarker(2.5, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeDown);

                })
                .addTemporalMarker(3.2, () -> {
                    verticalSlide.setTargetPosition(0);
                })
                .addTemporalMarker(3.7, () -> {
                    leftDropdown.setPosition(dropdownHalf);
                    rightDropdown.setPosition(dropdownHalf);
                })
                /*.addTemporalMarker(5.5, () -> {
                    rotateIntakeServo.setPosition(intakeDown);
                })*/
                .addTemporalMarker(4.7, () -> {

                    senseFreight();
                })
               /* .addTemporalMarker(8, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);

                    intakeServo.setPower(1);

                })*/
                .addTemporalMarker(6.7, () -> {

                    rotateIntakeServo.setPosition(intakeUp);
                    intakeServo.setPower(1);
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);

                })
                .addTemporalMarker(7, () -> {
                    intakeServo.setPower(0);

                })

                .addTemporalMarker(10.3, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(10.8, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(11.3, () -> {
                    slideRotation.setPosition(forward);
                })
                .addTemporalMarker(12, () -> {
                    rotateIntakeServo.setPosition(intakeDrop);
                })
                .addTemporalMarker(12.8, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeDown);
                })
                .addTemporalMarker(13.5, () -> {
                    verticalSlide.setTargetPosition(0);
                })
                .addTemporalMarker(14, () -> {
                    leftDropdown.setPosition(dropdownHalf);
                    rightDropdown.setPosition(dropdownHalf);
                })
//                .addTemporalMarker(16.5, () -> {
//                    rotateIntakeServo.setPosition(intakeDown);
//                })
                .addTemporalMarker(15, () -> {

                    senseFreight();
                })
                /* .addTemporalMarker(8, () -> {
                     leftDropdown.setPosition(dropdownDown);
                     rightDropdown.setPosition(dropdownDown);

                     intakeServo.setPower(1);

                 })*/
                .addTemporalMarker(17, () -> {

                    rotateIntakeServo.setPosition(intakeUp);
                    intakeServo.setPower(1);
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);

                })
                .addTemporalMarker(17.3, () -> {
                    intakeServo.setPower(0);

                })
                //TODO
                .addTemporalMarker(20.5, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(21, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(21.5, () -> {
                    slideRotation.setPosition(forward);
                })
                .addTemporalMarker(22.2, () -> {
                    rotateIntakeServo.setPosition(intakeDrop);
                })
                .addTemporalMarker(23, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeUp);
                })
                .addTemporalMarker(23.7, () -> {
                    verticalSlide.setTargetPosition(0);
                })
                .addTemporalMarker(24.2, () -> {
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);

                })




                .build();

        TrajectorySequence WarehouseBlueTop = drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(10, 34, Math.toRadians(210)))

                .forward(8)
                .waitSeconds(.5) //wait for freight dropoffaaa

                .lineToSplineHeading(new Pose2d(12, 65, Math.toRadians(180)))

                .addDisplacementMarker(() -> {
                    drive.setPoseEstimate( new Pose2d(12, 62, Math.toRadians(180)));

                })

                .back(40)


                .addTemporalMarker(.5, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(1, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(2 , () -> {
                    slideRotation.setPosition(forward);//eeffsf
                })
                .addTemporalMarker(3, () -> {
                    rotateIntakeServo.setPosition(intakeDrop);
                })
                .addTemporalMarker(3.5, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeUp);
                })
                .addTemporalMarker(4.5, () -> {
                    verticalSlide.setTargetPosition(0);
                })
                .addTemporalMarker(5, () -> {
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);
                })
                .addTemporalMarker(5.5, () -> {
                    rotateIntakeServo.setPosition(intakeDown);
                })
                .addTemporalMarker(6, () -> {

                    senseFreight();
                })
                /* .addTemporalMarker(8, () -> {
                     leftDropdown.setPosition(dropdownDown);
                     rightDropdown.setPosition(dropdownDown);

                     intakeServo.setPower(1);

                 })*/
                .addTemporalMarker(8.3, () -> {

                    rotateIntakeServo.setPosition(intakeUp);
                    intakeServo.setPower(1);
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);

                })
                .addTemporalMarker(9, () -> {
                    intakeServo.setPower(0);

                })

                .addTemporalMarker(11.5, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(12, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(13, () -> {
                    slideRotation.setPosition(forward);
                })
                .addTemporalMarker(14.7, () -> {
                    rotateIntakeServo.setPosition(intakeDrop);
                })
                .addTemporalMarker(15, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeUp);
                })
                .addTemporalMarker(15.5, () -> {
                    verticalSlide.setTargetPosition(0);
                })
                .addTemporalMarker(16, () -> {
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);
                })
                .addTemporalMarker(16.5, () -> {
                    rotateIntakeServo.setPosition(intakeDown);
                })
                .addTemporalMarker(17, () -> {

                    senseFreight();
                })
                /* .addTemporalMarker(8, () -> {
                     leftDropdown.setPosition(dropdownDown);
                     rightDropdown.setPosition(dropdownDown);

                     intakeServo.setPower(1);

                 })*/
                .addTemporalMarker(19.5, () -> {

                    rotateIntakeServo.setPosition(intakeUp);
                    intakeServo.setPower(1);
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);

                })
                .addTemporalMarker(20.2, () -> {
                    intakeServo.setPower(0);

                })

                .addTemporalMarker(22.5, () -> {
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                })
                .addTemporalMarker(23, () -> {
                    verticalSlide.setTargetPosition((int) slideValues[0]);
                })
                .addTemporalMarker(24, () -> {
                    slideRotation.setPosition(forward);
                })
                .addTemporalMarker(25.5, () -> {
                    rotateIntakeServo.setPosition(intakeDrop);
                })
                .addTemporalMarker(26, () -> {
                    slideRotation.setPosition(middle);
                    rotateIntakeServo.setPosition(intakeUp);
                })
                .addTemporalMarker(26.5, () -> {
                    verticalSlide.setTargetPosition(0);
                })
                .addTemporalMarker(27, () -> {
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);
                    rotateIntakeServo.setPosition(intakeUp);
                })




                .build();


        //TODO Carousel Blue

 /*       if (autoStart == 0 && autoMode == 0) { //Carousel Blue, deliver freight
            sleep(initialDelay * 1000);

        } else if (autoStart == 0 && autoMode == 1) { //Carousel Blue park

        }

        //TODO Warehouse Blue
        else if (autoStart == 1 && autoMode == 0) { //Warehouse Blue, deliver freight
            sleep(initialDelay * 1000);
            drive.setPoseEstimate( new Pose2d(12, 62, Math.toRadians(270)));
            drive.followTrajectorySequence(WarehouseBlueTop);

        } else if (autoStart == 1 && autoMode == 1 && barcodeValue == 2) { //Warehouse Blue park

            sleep(initialDelay * 1000);
            drive.setPoseEstimate(new Pose2d(12, 62, Math.toRadians(270)));
            drive.followTrajectorySequence(warehouseBluePark);
        }

        //TODO Carousel Red
        else if (autoStart == 2 && autoMode == 0) { //Carousel Red, deliver freight top, noncompleted
            sleep(initialDelay * 1000);
            drive.setPoseEstimate( new Pose2d(-35, -62, Math.toRadians(90)));
            drive.followTrajectorySequence(carouselRedTop);

        } else if (autoStart == 2 && autoMode == 1) { //Carousel Red park

        }

        //TODO Warehouse Red
        else if (autoStart == 3 && autoMode == 0) { //Warehouse Red, deliver freight
            sleep(initialDelay * 1000);
            drive.setPoseEstimate( new Pose2d(-35, -62, Math.toRadians(90)));
            drive.followTrajectorySequence(WarehouseRedTop);


        } else if (autoStart == 3 && autoMode == 1 && barcodeValue == 2) { //Warehouse Red park

        }*/



        //TODO Deliver Freight and Park


        drive.setPoseEstimate( new Pose2d(12, 62, Math.toRadians(270)));
        drive.followTrajectorySequence(WarehouseBlueTop);






    }

    /*
     * An enum to define the Team shipping element or duck position
     */
    public enum BarcodePosition
    {
        MIDDLE,
        RIGHT,
        LEFT
    }

    // Volatile since accessed by OpMode thread w/o synchronization
    public static volatile BarcodePipeline.BarcodePosition position = BarcodePipeline.BarcodePosition.RIGHT;



    public static class FreightFrenzyPipeline extends OpenCvPipeline
    {


        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         * If you want to enlarge or reduce the size of the regions, it should be done here.
         */
        static final Point REGION1_TOPMIDDLE_ANCHOR_POINT = new Point(80,98);
        static final Point REGION2_TOPMIDDLE_ANCHOR_POINT = new Point(230,98);
        //        static final Point REGION3_TOPMIDDLE_ANCHOR_POINT = new Point(253,98);
        static final int REGION_WIDTH = 60;
        static final int REGION_HEIGHT = 100;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPMIDDLE_ANCHOR_POINT.x,
                REGION1_TOPMIDDLE_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPMIDDLE_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPMIDDLE_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPMIDDLE_ANCHOR_POINT.x,
                REGION2_TOPMIDDLE_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPMIDDLE_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPMIDDLE_ANCHOR_POINT.y + REGION_HEIGHT);
//        Point region3_pointA = new Point(
//                REGION3_TOPMIDDLE_ANCHOR_POINT.x,
//                REGION3_TOPMIDDLE_ANCHOR_POINT.y);
//        Point region3_pointB = new Point(
//                REGION3_TOPMIDDLE_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION3_TOPMIDDLE_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        //        int avg1, avg2, avg3;
        int avg1, avg2;


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
//            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because the ducks, and often team shipping elements,
             * will be a brighter color, such as yellow, and contrast strongly.
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over area. After this, we find which has the highest
             * concentration of color. That will be our element.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the element.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 tape markers, and
             * be small enough such that only the duck or team shipping element is sampled,
             * and not any of the surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
//            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region3_pointA, // First point which defines the rectangle
//                    region3_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines


            /*
             * Find the min of the 3 averages
             */
//            int minOneTwo = Math.min(avg1, avg2);
//            int min = Math.min(minOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            ////TODO if (Math.abs(avg1-avg2) > 15 && avg2 > avg1) then RIGHT
            if(Math.abs(avg1-avg2) > 10 && avg2 > avg1) // Was it from region 1?
            {
                position = BarcodePipeline.BarcodePosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            }
            //TODO if (Math.abs(avg1-avg2) > 15 && avg1 > avg2) then MIDDLE
            else if(Math.abs(avg1-avg2) > 10 && avg1 > avg2) // Was it from region 2?
            {
                position = BarcodePipeline.BarcodePosition.MIDDLE; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            //TODO if (Math.abs(avg1-avg2) < 15) then LEFT
            else if(Math.abs(avg1-avg2) < 10) // Was it from region 3?
            {
                position = BarcodePipeline.BarcodePosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region3_pointA, // First point which defines the rectangle
//                        region3_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public BarcodePipeline.BarcodePosition getAnalysis()
    {
        return position;
    }

}

