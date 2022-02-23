package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "Port1ExpansionMotorTest", group = "Testing")



public class TestMotorPort1 extends LinearOpMode {


    //static values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = 1.1; //to match rotation with driving
    private final double wheelMaxVelocity = 1;
    private final double maxVerticalSlideTicks = 384.5 * 3.64;
    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;
    private final double spinnyTicks = 537.7;
    private final double maxSpinnyVelocity = 312 * spinnyTicks / 60;

    private final double intakeDropFreight = 0;
    private final double intakeDown = .5;
    private final double intakeUp = .3;
    private final double intakeDrop = .65;

    private final double dropdownDown = .55;
    private final double dropdownUp = .1;
    private final double dropdownDownLeft = .3;
    private final double dropdownUpLeft = .055;

    private final double right = 1;
    private final double middle = .7;
    private final double left = .33;
    private final double forward = 0;


    //gear change variables
    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3


    //inverse drive variables
    private boolean inversePressed = false;
    private int inverse = 1;

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};

    private int horizontalPosition = 2;
    private final double[] horizontalValues = {
            -240,
            -120,
            0,

    };
    private boolean horizontalPlayerControlled = false;
    private boolean horizontalMoving = false;
    private boolean horizontalSwitch = false;



    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //timers
//    private ElapsedTime spinnyTime = new ElapsedTime();
    private ElapsedTime autoIntakeTime = new ElapsedTime();

    //create motor/servo objects
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotorEx spinny;
    private CRServo intakeServo;
    private CRServo carouselServo;

    private Servo rotateIntakeServo;
    private DcMotorEx verticalSlide;

    private Servo rightDropdown;
    private Servo leftDropdown;
    private Servo slideRotation;
    private Servo horizontalServo;

    private ColorSensor color;


    //create arm objects


    //create arm variables
    private final double maxSlideTicks = 360;

    //arm position values
    private int slidePosition = 0;
    private final double[] slideValues = {
            maxVerticalSlideTicks * 1, //top level
            maxVerticalSlideTicks * .55, //shared
            maxVerticalSlideTicks * .3, //low level
            maxVerticalSlideTicks * .6, //middle level

    };
    String[] slideLevel = {"Top level", "Lowest level", "Middle level", "shared shipping"};

    //intake servo position values
    private int intakePosition = 1;
    private final double[] intakeValues = {
            .1, .4, .8, 1
    };

    //dropdown servos position values
    private int dropdownPosition = 1;
    private final double[] dropdownValues = {
            .55, .1
    };


    //toggle booleans
    private boolean slideMoving = false;
    private boolean slidePlayerControlled = true;
    private boolean slideSwitch = false;
    private boolean autoIntakeActive = false;

    private boolean intakeMoving = false;
    private boolean dropdownMoving = false;


    public enum AutoIntakeState {
        DEFAULT_POSITION,
        INTAKE_ACTIVE,
        ACTIVE_INTAKE_NO_SENSOR,
        REVERSE_INTAKE,
        TOP_LEVEL,
        HORIZONTAL_ROTATION_RIGHT,
        HORIZONTAL_ROTATION_LEFT,
        HORIZONTAL_ROTATION_FORWARD,
        DROP_FREIGHT,
        SLIDE_DOWN,
        SHARED_RIGHT,
        SHARED_LEFT
    }

    AutoIntakeState autointakeState = AutoIntakeState.DEFAULT_POSITION;

    public enum LiftState {
        DEFAULT_POSITION,
        TOP_LEVEL, //DPAD up = top, down = shared, left = middle, right = bottom
        HORIZONTAL_ROTATION_RIGHT, //circle = rotate right, square = rotate left
        HORIZONTAL_ROTATION_LEFT,
        DROP_FREIGHT, //automatically after horizontal rotation
        SLIDE_DOWN, //automatically after dropping freight, rotate intake to default, LiftState liftState = LiftState.DEFAULT_POSITION;
        SHARED_RIGHT,
        SHARED_lEFT
    }

    LiftState liftState = LiftState.DEFAULT_POSITION;


    //method to change gears
    private void changeGears() {

        //if statement to change active gear count
        if (gamepad1.right_trigger > 0 && !bumpersPressed && activeGear < gearValues.length - 1) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            gamepad1.rumble(0.9, 0, 200);
            //add 1 to active gear
            activeGear++;

        } else if (gamepad1.left_trigger > 0 && !bumpersPressed && activeGear > 0) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            gamepad1.rumble(0.9, 0, 200);
            //subtract 1 from active gear
            activeGear--;

        } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
            bumpersPressed = false;

    }

    public void changeSlidePosition() {
        if (gamepad2.dpad_up) {
            slidePosition = 0;
        } else if (gamepad2.dpad_right) {
            slidePosition = 2;
        } else if (gamepad2.dpad_left) {
            slidePosition = 1;
        } else if (gamepad2.dpad_down) {
            slidePosition = 3;
        }
    }


  /*  public void slideControl() {

        if (slidePlayerControlled) {
            if (gamepad2.right_stick_y > 0)
                verticalSlide.setTargetPosition((int) (verticalSlide.getCurrentPosition() + (maxSlideTicks * .2)));
            else if (gamepad2.right_stick_y < 0)
                verticalSlide.setTargetPosition((int) (verticalSlide.getCurrentPosition() - (maxSlideTicks * .2)));
            else if (gamepad2.right_stick_y == 0)
                verticalSlide.setTargetPosition(verticalSlide.getCurrentPosition());

        } else {
            verticalSlide.setTargetPosition((int) slideValues[slidePosition]);
        }

        //increase and decrease current position
        if (gamepad2.dpad_down && slidePosition > 0 && !slideMoving) {
            //decrease
            slidePosition--;
            slideMoving = true;
        } else if (gamepad2.dpad_up && slidePosition < 3 && !slideMoving) {
            //increase
            slidePosition++;
            slideMoving = true;
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down && slideMoving)
            slideMoving = false;

        //toggle slide control to and from player
        if (gamepad2.x && !slideSwitch && !slidePlayerControlled) {
            //make slide player controlled
            slidePlayerControlled = true;
            slideSwitch = true;
        } else if (gamepad2.x && !slideSwitch && slidePlayerControlled) {
            //remove player control from slide
            slidePlayerControlled = false;
            slideSwitch = true;
        } else if (slideSwitch && !gamepad2.x)
            slideSwitch = false;




        verticalSlide.setVelocity(wheelMaxVelocity);


    }*/

    public void intakeRotationControl() {
        if (gamepad1.square && intakePosition > 0 && !intakeMoving) {
            //decrease position
            intakePosition--;
            intakeMoving = true;
        } else if (gamepad1.triangle && intakePosition < 3 && !intakeMoving) {
            //increase position
            intakePosition++;
            intakeMoving = true;
        } else if (!gamepad1.square && !gamepad1.triangle && intakeMoving)
            intakeMoving = false;

        rotateIntakeServo.setPosition(intakeValues[intakePosition]);

    }

    public void dropdownControl() {
        if (gamepad2.a && dropdownPosition > 0 && !dropdownMoving) {
            //decrease position
            dropdownPosition--;
            dropdownMoving = true;
        } else if (gamepad2.y && dropdownPosition < 1 && !dropdownMoving) {
            //increase position
            dropdownPosition++;
            dropdownMoving = true;
        } else if (!gamepad2.a && !gamepad2.y && dropdownMoving)
            dropdownMoving = false;

        rightDropdown.setPosition(dropdownValues[dropdownPosition]);
        leftDropdown.setPosition(dropdownValues[dropdownPosition]);

    }

    public void rotateHorizontal() {
        if (horizontalPlayerControlled) {
            if (-gamepad2.left_stick_y > 0)
                spinny.setTargetPosition(spinny.getCurrentPosition() + 50);
            else if (-gamepad2.left_stick_y < 0)// && wobbleMotor.getCurrentPosition() > wobbleValues[0] + 50)
                spinny.setTargetPosition(spinny.getCurrentPosition() - 50);
            else if (gamepad2.left_stick_y == 0)
                spinny.setTargetPosition(spinny.getCurrentPosition());
        } else {
            spinny.setTargetPosition((int) horizontalValues[horizontalPosition]);
        }

        if (gamepad2.dpad_left && horizontalPosition > 0 && !horizontalMoving) {
            horizontalPosition--;
            horizontalMoving = true;
        } else if (gamepad2.dpad_right && horizontalPosition < 2 && !horizontalMoving) {
            horizontalPosition++;
            horizontalMoving = true;
        } else if (!gamepad2.dpad_left && !gamepad2.dpad_right && horizontalMoving) {
            horizontalMoving = false;
        }

        if (gamepad2.share && !horizontalSwitch && !horizontalPlayerControlled) {
            //make wobble player controlled
            horizontalPlayerControlled = true;
            horizontalSwitch = true;
        } else if (gamepad2.share && !horizontalSwitch && horizontalPlayerControlled) {
            //remove player control from wobble
            horizontalPlayerControlled = false;
            horizontalSwitch = true;
        } else if (horizontalSwitch && !gamepad2.share)
            horizontalSwitch = false;

        if (gamepad2.options) {

//            spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        }

    }


    @Override
    public void runOpMode() throws InterruptedException {


        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");

        //setup other objects
        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselServo = hardwareMap.get(CRServo.class, "carouselServo");

        slideRotation = hardwareMap.get(Servo.class, "slideRotation");
        horizontalServo = hardwareMap.get(Servo.class, "horizontalServo");

        rotateIntakeServo = hardwareMap.get(Servo.class, "rotateIntakeServo");

        rightDropdown = hardwareMap.get(Servo.class, "rightDropdown");
        leftDropdown = hardwareMap.get(Servo.class, "leftDropdown");

        //servo direction
        rightDropdown.setDirection(Servo.Direction.REVERSE);
        leftDropdown.setDirection(Servo.Direction.FORWARD);

        rotateIntakeServo.setDirection(Servo.Direction.FORWARD);
        slideRotation.setDirection(Servo.Direction.FORWARD);

        //motor direction
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        //zero power behavior
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        spinny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armMotor encoders
        verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setTargetPosition(0);
        verticalSlide.setVelocity(0);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinny.setTargetPosition(0);
        spinny.setVelocity(0);
        spinny.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        color = hardwareMap.colorSensor.get("color");


        waitForStart();


        verticalSlide.setVelocity(maxSlideVelocity);
        spinny.setVelocity(maxSpinnyVelocity * .2);
        //rotateIntakeServo.setPosition(.5);
        /*leftDropdown.setPosition(.1);
        rightDropdown.setPosition(.1);
        rotateIntakeServo.setPosition(.4);*/


        //while loop
        while (opModeIsActive() && !isStopRequested()) {


            verticalSlide.setPower(gamepad1.right_stick_y  );

        }

    }
}



