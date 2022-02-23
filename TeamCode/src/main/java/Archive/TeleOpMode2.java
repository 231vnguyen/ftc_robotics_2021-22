package Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

@Disabled
@TeleOp(name = "Zero Servo Port 0", group = "Testing")
public class TeleOpMode2 extends LinearOpMode {

//test
    //static values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = 1.1; //to match rotation with driving
    private final double wheelMaxVelocity = wheelMotorRPM * wheelMotorTicks / 60;

    private final double servoLeftStart= .7;
    private final double servoLeftEnd= .1;
    private final double servoRightStart = .2;
    private final double servoRightEnd= .75;

    //gear change variables
    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3


    //inverse drive variables
    private boolean inversePressed = false;
    private int inverse = -1;

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};


    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //timers
    private ElapsedTime spinnyTime = new ElapsedTime();

    //create motor/servo objects
    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;
    private DcMotor spinny;
    private CRServo intakeServo;

    private Servo rotateIntakeServo;
    private DcMotor verticalSlide;
    private Servo rightDropdown;
    private Servo leftDropdown;



    //create arm objects


    //create arm variables
    private final double maxSlideTicks = 360;

    //arm position values
    private int slidePosition = 0;
    private final double[] slideValues = {
            0,
            maxSlideTicks * .25,
            maxSlideTicks * .5,
            maxSlideTicks * 1
    };

    //intake servo position values
    private int intakePosition = 1;
    private final double[] intakeValues = {
            0, .1, .4, .8, 1
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

    private boolean moveDPad = false;
    private int slidePostion = 0;

    private boolean intakeMoving = false;
    private boolean dropdownMoving = false;
    


    //method to change gears
    private void changeGears() {

        //if statement to change active gear count
        if (gamepad1.right_trigger > 0 && !bumpersPressed && activeGear < gearValues.length - 1) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            //add 1 to active gear
            activeGear++;

        } else if (gamepad1.left_trigger > 0 && !bumpersPressed && activeGear > 0) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            //subtract 1 from active gear
            activeGear--;

        } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
            bumpersPressed = false;

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
        if (gamepad1.a && intakePosition > 0 && !intakeMoving) {
            //decrease position
            intakePosition--;
            intakeMoving = true;
        } else if (gamepad1.y && intakePosition < 4 && !intakeMoving) {
            //increase position
            intakePosition++;
            intakeMoving = true;
        } else if (!gamepad1.a && !gamepad1.y && intakeMoving)
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





    @Override
    public void runOpMode() throws InterruptedException {




        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");

        //setup other objects
        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        rotateIntakeServo = hardwareMap.get(Servo.class, "rotateIntakeServo");
        
        rightDropdown = hardwareMap.get(Servo.class, "rightDropdown");
        leftDropdown = hardwareMap.get(Servo.class, "leftDropdown");

        //servo direction
        rightDropdown.setDirection(Servo.Direction.REVERSE);
        leftDropdown.setDirection(Servo.Direction.FORWARD);

        rotateIntakeServo.setDirection(Servo.Direction.REVERSE);

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
        spinny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armMotor encoders
        verticalSlide = hardwareMap.get(DcMotor.class, "verticalSlide");
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setTargetPosition(0);
        verticalSlide.setPower(0);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlide.setDirection(DcMotorSimple.Direction.REVERSE);






        waitForStart();

        //rotateIntakeServo.setPosition(.5);
        /*leftDropdown.setPosition(.1);
        rightDropdown.setPosition(.1);
        rotateIntakeServo.setPosition(.4);*/

        verticalSlide.setPower(.5);





        //while loop
        while (opModeIsActive()) {



            //TODO Gamepad 1


            //update controller variables
            gamepad1LY = gamepad1.left_stick_y;
            gamepad1LX = gamepad1.left_stick_x;
            gamepad1RX = gamepad1.right_stick_x;

            //event methods
            changeGears();




            //-----------------------------------------------------------------------------
            //Inverse drive with start
            if (gamepad1.start && !inversePressed)
                inverse *= -1;
            inversePressed = gamepad1.start;


            //-----------------------------------------------------------------------------
            //spinny with b and a

            if (gamepad1.b) {
                //spinnyTime.reset();
                spinny.setPower(-(Math.pow(10, (.2 * spinnyTime.seconds()) - .5)));

            } else if (gamepad1.x) {
                spinny.setPower((Math.pow(10, (.2 * spinnyTime.seconds()) - .5)));

            } else if (!gamepad1.x || !gamepad1.b) {
                spinnyTime.reset();
                spinny.setPower(0);
            } else
                spinny.setPower(0);



            //---------------------------------------------------------------------
            //Mecanum Drivetrain Control
            //if statement to ensure robot won't move unless control sticks are moved
            if (Math.abs(gamepad1LY) > .05 || Math.abs(gamepad1LX) > .05 || Math.abs(gamepad1RX) > .05) {
                //control logic: multiply by gear array for different speeds,
                //multiply right stick movement by rotation fraction to reduce uncontrolability
                flMotor.setVelocity((gearValues[activeGear] * (-1 * inverse * (gamepad1LY - gamepad1LX)) -  gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                frMotor.setVelocity((gearValues[activeGear] * (-1 * inverse * (gamepad1LY + gamepad1LX)) +  gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                blMotor.setVelocity((gearValues[activeGear] * (-1 * inverse * (gamepad1LY + gamepad1LX)) -  gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                brMotor.setVelocity((gearValues[activeGear] * (-1 * inverse * (gamepad1LY - gamepad1LX)) +  gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
            } else if (gamepad1.left_bumper && !gamepad1.right_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity( .3 * wheelMaxVelocity);
                frMotor.setVelocity(-.3 * wheelMaxVelocity);
                blMotor.setVelocity( .3 * wheelMaxVelocity);
                brMotor.setVelocity(-.3 * wheelMaxVelocity);
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity(-.3 * wheelMaxVelocity);
                frMotor.setVelocity( .3 * wheelMaxVelocity);
                blMotor.setVelocity(-.3 * wheelMaxVelocity);
                brMotor.setVelocity( .3 * wheelMaxVelocity);
            } else if (gamepad1.dpad_down) {
                flMotor.setVelocity(inverse * (-.3 * wheelMaxVelocity));
                frMotor.setVelocity(inverse * (-.3 * wheelMaxVelocity));
                blMotor.setVelocity(inverse * (-.3 * wheelMaxVelocity));
                brMotor.setVelocity(inverse * (-.3 * wheelMaxVelocity));

            } else if (gamepad1.dpad_up) {
                flMotor.setVelocity(inverse * (.3 * wheelMaxVelocity));
                frMotor.setVelocity(inverse * (.3 * wheelMaxVelocity));
                blMotor.setVelocity(inverse * (.3 * wheelMaxVelocity));
                brMotor.setVelocity(inverse * (.3 * wheelMaxVelocity));
            } else {
                flMotor.setVelocity(0);
                frMotor.setVelocity(0);
                blMotor.setVelocity(0);
                brMotor.setVelocity(0);
            }
            //-----------------------------------------------------------------------------------
            //tubey dropdown servos

            //dropdownControl();

            /*if (gamepad1.a) {
                rightDropdown.setPosition(.63);
                leftDropdown.setPosition( .63);
            } else if (gamepad1.y) {
                rightDropdown.setPosition(0);
                leftDropdown.setPosition(0);
            }*/


            //TODO Gamepad 2

            //event methods
            //slideControl();
            intakeRotationControl();

            /*if (gamepad2.x) {
                leftDropdown.setPosition(0);
                rightDropdown.setPosition(0);
            } else if (gamepad2.b) {
                leftDropdown.setPosition(.55);
                rightDropdown.setPosition(.55);
            }*/
            dropdownControl();


            if (gamepad2.right_trigger > 0) {
                intakeServo.setPower(-1);

            } else if (gamepad2.left_trigger > 0) {
                intakeServo.setPower(1);
            } else
                intakeServo.setPower(0);

            //verticalSlide.setPower(gamepad2.left_stick_y);
            intakeServo.setPower(gamepad2.right_stick_y);

            if (!moveDPad && gamepad2.dpad_up)
                slidePostion++;
            else if (!moveDPad && gamepad2.dpad_down)
                slidePostion--;
            moveDPad = (gamepad2.dpad_down || gamepad2.dpad_up);

            slidePostion = Math.max(0, Math.min(2, slidePostion));

            switch (slidePostion)
            {
                //384.5 encoder count
                case 0:
                    verticalSlide.setTargetPosition(0);
                    break;
                case 1:
                    verticalSlide.setTargetPosition(385 * 3);
                    break;
                case 2:
                    verticalSlide.setTargetPosition(385 * 6);
                    break;
            }


            //send what gear is active
            telemetry.addData("Gear", activeGear + 1);
            telemetry.addData("Inverse Value", inverse);
            telemetry.addData("Servo Power", intakeServo.getPower());
            telemetry.addData("Spinny Power", spinny.getPower());
            telemetry.addData("SpinnyTime", spinnyTime.seconds());
            telemetry.addData("Slide Tick Position", verticalSlide.getCurrentPosition());
            telemetry.addData("Intake Rotation", intakeValues[intakePosition]);
            telemetry.addData("Intake Rotation Servo", rotateIntakeServo.getPosition());
            telemetry.addData("Dropdown Position", rightDropdown.getPosition());
            telemetry.addData("Dropdown Array Value", dropdownValues[dropdownPosition]);

            telemetry.update();



        }

    }
}


