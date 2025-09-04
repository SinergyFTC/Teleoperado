//Importações:
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="TeleOpGeral")
public class TeleOpGeral extends LinearOpMode {

    // Motores do drivetrain
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    // Motores auxiliares
    private DcMotor motor;
    private DcMotor motor2;
    private DcMotor motor3;
    private CRServo motorgarra1;
    private CRServo motorgarra2;
    private Servo servoGarra;

    // Variáveis de controle
    private int targetPosition = 0;
    private final int INCREMENTO = 1000;
    private boolean garraAtiva = false;
    private int duracao = 0;
    private double power1 = 0;
    private double power2 = 0;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;

    // Timer
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Mapeamento dos motores
        leftFrontDrive  = hardwareMap.dcMotor.get("FL");
        leftBackDrive   = hardwareMap.dcMotor.get("BL");
        rightFrontDrive = hardwareMap.dcMotor.get("FRONT RIGHT");
        rightBackDrive  = hardwareMap.dcMotor.get("BR");

        // Ajuste das direções
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Motores auxiliares
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motorgarra1 = hardwareMap.get(CRServo.class, "MotorGarra1");
        motorgarra2 = hardwareMap.get(CRServo.class, "MotorGarra2");
        servoGarra = hardwareMap.get(Servo.class, "ServoGarra");

        // Configuração dos encoders
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        timer.reset();

        // Inicializa posição do braço
        targetPosition = motor.getCurrentPosition();

        while (opModeIsActive()) {
            // -------------------- DRIVER 1 (Movimentação) --------------------
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double leftFrontPower = drive + strafe + turn;
            double leftBackPower = drive - strafe + turn;
            double rightFrontPower = drive - strafe - turn;
            double rightBackPower = drive + strafe - turn;

            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);

            // -------------------- DRIVER 2 --------------------

            // Controle do braço com botões A, B e Y
            if (gamepad2.a && !lastA) {
                targetPosition += INCREMENTO;
                motor.setTargetPosition(targetPosition);
                motor2.setTargetPosition(targetPosition);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1.0);
                motor2.setPower(1.0);
            }
            lastA = gamepad2.a;

            if (gamepad2.b && !lastB) {
                targetPosition -= INCREMENTO;
                motor.setTargetPosition(targetPosition);
                motor2.setTargetPosition(targetPosition);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1.0);
                motor2.setPower(1.0);
            }
            lastB = gamepad2.b;

            if (gamepad2.y && !lastY) {
                targetPosition = motor.getCurrentPosition();
                motor.setTargetPosition(targetPosition);
                motor2.setTargetPosition(targetPosition);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);
                motor2.setPower(0.5);
            }
            lastY = gamepad2.y;

            // Controle da garra com X, A, Y, B (timer.reset no início)
            if (gamepad2.x) {
                timer.reset();
                garraAtiva = true;
                power1 = 0.5;
                power2 = 0.9;
                duracao = 500;
            } else if (gamepad2.a) {
                timer.reset();
                garraAtiva = true;
                power1 = -1.0;
                power2 = 1.0;
                duracao = 300;
            } else if (gamepad2.y) {
                timer.reset();
                garraAtiva = true;
                power1 = 1.0;
                power2 = -1.0;
                duracao = 300;
            } else if (gamepad2.b) {
                timer.reset();
                garraAtiva = true;
                power1 = -0.9;
                power2 = -1.0;
                duracao = 450;
            }

            if (garraAtiva) {
                if (timer.milliseconds() < duracao) {
                    motorgarra1.setPower(power1);
                    motorgarra2.setPower(power2);
                } else {
                    motorgarra1.setPower(0);
                    motorgarra2.setPower(0);
                    garraAtiva = false;
                }
            }

            // Controle do servoGarra com gatilhos
            if (gamepad2.left_trigger > 0) {
                servoGarra.setPosition(0);
            } else if (gamepad2.right_trigger > 0) {
                servoGarra.setPosition(0.3);
            }

            // Telemetria
            telemetry.addData("Posição do braço", motor.getCurrentPosition());
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Garra Ativa", garraAtiva);
            telemetry.update();
        }
    }
}
