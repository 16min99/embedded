//방향 바꾸기 테스트
//PORTA 선언
#define PORTAO (*((volatile unsigned char*)0x404)) //output
#define PORTA_DIR (*((volatile unsigned char*)0x400)) //DIRA
#define PORTAI (*((volatile unsigned char*)0x408)) //input
#define PORTDO (*((volatile unsigned char*)0x464))
#define PORTD_DIR (*((volatile unsigned char*)0x460))

//UART register pc랑 usb통신용
#define UART1_RXDATA (*((volatile unsigned char*)0x820)) 
#define UART1_TXDATA (*((volatile unsigned char *)0x822))
#define UART1_STATUS (*((volatile unsigned char *)0x824))
#define UART1_CTRLB (*((volatile unsigned char *)0x826))
#define UART1_CTRLC (*((volatile unsigned char *)0x827))
#define UART1_BAUDL (*((volatile unsigned char *)0x828))
#define UART1_BAUDH (*((volatile unsigned char *)0x829))
//PORTC의 tx부분 pc0핀 출력으로 설정해줘야함 PORTC_DIR

//UART register bluetooth 통신용
#define UART2_TXDATA (*((volatile unsigned char *)0x842))
#define UART2_RXDATA (*((volatile unsigned char*)0x840)) 
#define UART2_STATUS (*((volatile unsigned char *)0x844))
#define UART2_CTRLB (*((volatile unsigned char *)0x846))
#define UART2_CTRLC (*((volatile unsigned char *)0x847))
#define UART2_BAUDL (*((volatile unsigned char *)0x848))
#define UART2_BAUDH (*((volatile unsigned char *)0x849))
//PORTF의 tx부분 pc0핀 출력으로 설정해줘야함 PORTF_DIR 

#define PORTC_DIR (*((volatile unsigned char *)0x440))
#define PORTF_DIR (*((volatile unsigned char *)0x4A0))

//ADC register
#define ADC_CTRLA (*((volatile unsigned char *)0x600))
#define ADC_CTRLC (*((volatile unsigned char *)0x602))
#define ADC_MUXPOS (*((volatile unsigned char *)0x606))
#define ADC_COMMAND (*((volatile unsigned char *)0x608))
#define ADC_INTFLAGS (*((volatile unsigned char *)0x60b))
#define ADC_RESL (*((volatile unsigned char *)0x610))

//TWI 통신
#define TWI0_MBAUD (*((volatile unsigned char*)0x8A6))
#define TWI0_MCTRLB  (*((volatile unsigned char*)0x8A4))
#define TWI0_MSTATUS (*((volatile unsigned char*)0x8A5))
#define TWI0_MADDR  (*((volatile unsigned char*)0x8A7))
#define TWI0_MCTRLA  (*((volatile unsigned char*)0x8A3))
#define TWI0_MADDR  (*((volatile unsigned char*)0x8A7))
#define TWI0_MDATA  (*((volatile unsigned char*)0x8A8))
#define PORTMUX_TWISPIROUTEA  (*((volatile unsigned char*)0x5E3))

float filterCMP(float argv1, float argv2);
float getAngleA(float axsum, float azsum);
void getAngleG();


//FUNCTION TCA
void setup_TCA();

//FUNCTION TWI
void TWI_Init(char baud);
char TWI_Start(char addr);
char TWI_Read(char addr, char ack);
void TWI_Write(char addr, char data);
void TWI_Stop();

void getRawdata();
void datafilter();
void IMU_setup();
void ITOA(unsigned long a);

//FUNCTION SETUP
void setup_PORT();

//FUNCTION ADC
void ADC_init();
uint8_t readADC1();
uint8_t readADC2();

//FUNCTION UART
void sendChar1(char d);
void sendChar2(char d);
void UART1_init();
void UART2_init();
unsigned char uart1_rec(void);
unsigned char uart2_rec(void);

//FUNCTION FILTER 
void filter_cal();
void filter_axis();
void filter_gyro();
void getDT();

//VARIABLES
volatile byte gyro_addr = 0x68; //MPU6050 WHO_AM_I value
int16_t a_x;
int16_t a_y;
int16_t a_z;
int16_t temp;
int16_t g_x;
int16_t g_y;
int16_t g_z; //초기에 i2c로 받아오는 값을 정제한 것을 담아두는 변수

float init_data[6];

float aAngle_x;
double gAngle = 0;
float filter_x = 0;
float gX, gZ;

unsigned long t_now = 0;
unsigned long t_prev;
float dt = 0;

float Result = 0;

float filter_gyro_Result = 0;
//  Result = ALPHA * (Result + argv2 * dt) + (1-ALPHA) * A;



int16_t aa;
int16_t bb;
int16_t cc;
int16_t dd;
int16_t ee;
int16_t ff;
int16_t gg;

float inita_x = 0;
float inita_y = 0;
float inita_z = 0;
float initg_x = 0;
float initg_y = 0;
float initg_z = 0;

char buf[10];     //character buffer to itoa()

float a_xsum = 0;
float a_ysum = 0;
float a_zsum = 0;
float g_xsum = 0;
float g_ysum = 0;
float g_zsum = 0;

uint64_t now = 0;
uint64_t past = 0;


uint8_t f1, f2;
uint8_t df1, df2;

void setup() {
    setup_PORT();
    UART1_init();
    UART2_init();
    PORTA_DIR |= B00001000;
    TWI_Init(0x1F);
    IMU_setup();
    DT_setup();
    ADC_init();

    f1 = readADC1();

}
int real_temp;
int real_result;

void loop() {

    ////////////////////////////adc휨센서값 보정 중지 f1 엄지가 f2
    f1 = readADC1();
    f2 = readADC2();

    if (f1 > 192) df1 = 250;
    else if (f1 < 150) df1 = 0;
    else  df1 = (f1 - 150) * 6;

    if (f2 > 212) df2 = 250;
    else if (f2 < 150) df2 = 0;
    else df2 = (f2 - 150) * 4;
    //////////////////////////


    float temp;
    getDT();
    datafilter(); // get filtered acc, gyro data
    temp = getAngleA(a_xsum, a_zsum);
    ///////////////////////////////////////////////////
    if (Result > 90) real_result = 90;
    else if (Result < (-90)) real_result = -90;
    else real_result = Result;



    ////////// 상보필터로 구한 각도값을 정지상태에서의 가속도센서로
    ////////// 구한 각도값으로 초기화시켜 적분오차를 최소화하는 알고리즘
    if ((g_xsum < 3) && (g_xsum > -3) && (g_ysum < 3) && (g_ysum > -3) && (g_zsum < 3) && (g_zsum > -3))
    {
        filter_gyro_Result = temp;
    }
    getAngleG();
    filterCMP(temp, g_ysum);

    sendChar2(df1);
    sendChar2(df2);
    sendChar2(real_result);


    ///////////////////////////////시리얼모니터로 확인 
    ITOA(df1);
    sendChar1(':');
    ITOA(df2);
    sendChar1(':');

    ITOA(real_result);

    sendChar1('\n');

}

//FUNCTION DECLARATION=============================================

//FUNCTION UART
unsigned char uart1_rec(void) {
    while ((UART1_STATUS & 0x80) == 0); // 데이터 수신확인까지 기다린다.
    return UART1_RXDATA;
}
unsigned char uart2_rec(void) {
    while ((UART2_STATUS & 0x80) == 0); // 데이터 수신확인까지 기다린다.
    return UART2_RXDATA;
}
void UART1_init() {
    PORTC_DIR |= B00000001;
    UART1_BAUDL = 0x0b;
    UART1_BAUDH = 0x1a;
    UART1_CTRLB = B11000000;
    UART1_CTRLC = B00000011;
}
void UART2_init() {
    PORTF_DIR |= B00000001;
    UART2_BAUDL = 0x0b;
    UART2_BAUDH = 0x1a;
    UART2_CTRLB = B11000000;
    UART2_CTRLC = B00000011;
}
void sendChar1(char d) {
    while (!(UART1_STATUS & B00100000));
    UART1_TXDATA = d;
}
void sendChar2(char d) {
    while (!(UART2_STATUS & B00100000));
    UART2_TXDATA = d;
}

//FUNCTION ADC
//readADC1 =방향pd7 , 2=속도 pd6
uint8_t readADC1() {
    ADC_MUXPOS = B00000111;
    ADC_COMMAND = B00000001;              //start conversion
    while (!(ADC_INTFLAGS & B00000001)); //if result is empty, wait
    ADC_INTFLAGS = B00000001;
    return ADC_RESL;
}
uint8_t readADC2() {
    ADC_MUXPOS = B00000110;
    ADC_COMMAND = B00000001;              //start conversion
    while (!(ADC_INTFLAGS & B00000001)); //if result is empty, wait
    ADC_INTFLAGS = B00000001;
    return ADC_RESL;
}
void ADC_init() {
    ADC_CTRLA = B00000101;      //8bit resolution, ADC is enabled
    ADC_CTRLC = B00010011;      //Vref = VDD(5V), CLK_PER divided by 8
}

//FUNCTION TWI
void TWI_Init(char baud)
{
    PORTMUX_TWISPIROUTEA |= B00000000;//use TWI
    TWI0_MBAUD = baud; // (F_CPU/F_SCL)-10
    TWI0_MCTRLB |= B00001000;//flush 1,
    TWI0_MSTATUS |= (B10000000 | B01000000);//RIF 1,WIF1,
    TWI0_MCTRLA = 1 << 1 | 1 << 0; //SMEN,EN
    TWI0_MSTATUS |= (0x01 << 0);  /* Bus is Idle */

    TWI_Start(0xD0);
    TWI_Write(0x6B);
    TWI_Write(0x00);
    TWI_Stop();
    TWI_Start(0xD0);
    TWI_Write(0x1A);
    TWI_Write(0x00);
    TWI_Stop();
    TWI_Start(0xD0);
    TWI_Write(0x1B);
    TWI_Write(0x18);
    TWI_Stop();
    //자이로센서관련

}
char TWI_Start(char addr)
{
    TWI0_MCTRLB &= ~(1 << 2);
    TWI0_MADDR = addr;

    if (addr & 1) { while (!(TWI0_MSTATUS & 0x80)); }
    else { while (!(TWI0_MSTATUS & 0x40)); }
}

char TWI_Read(char ack)
{
    while (!(TWI0_MSTATUS & 0x80));
    if (ack) { TWI0_MCTRLB &= ~(1 << 2); }//send ack
    else { TWI0_MCTRLB |= 1 << 2; }

    return TWI0_MDATA;
}

void TWI_Write(char data) {

    while (!((TWI0_MSTATUS & 0x40) | (TWI0_MSTATUS & 0x10)));
    TWI0_MDATA = data;
}

void TWI_Stop()
{
    TWI0_MCTRLB |= (0x03 << 0);//STOP GC
}

void setup_PORT() {
    PORTA_DIR = B11111111;
}

void getRawdata() {
    byte IMU_data[14];

    TWI_Start(0xD0);
    TWI_Write(0x3B);
    TWI_Start(0xD1);

    for (int i = 0; i < 13; i++) {
        IMU_data[i] = TWI_Read(1);
    }
    IMU_data[13] = TWI_Read(0);
    TWI_Stop();

    a_x = (int)IMU_data[0] << 8 | (int)IMU_data[1];
    a_y = (int)IMU_data[2] << 8 | (int)IMU_data[3];
    a_z = (int)IMU_data[4] << 8 | (int)IMU_data[5];

    g_x = (int)IMU_data[8] << 8 | (int)IMU_data[9];
    g_y = (int)IMU_data[10] << 8 | (int)IMU_data[11];
    g_z = (int)IMU_data[12] << 8 | (int)IMU_data[13];
}

//FUNCTION GET ACC ANGLE
float getAngleA(float axsum, float azsum) {
    float result;
    //result = atan(axsum  / sqrt(pow(aysum,2) + pow(azsum,2)));
    result = atan2(axsum, azsum) * 180 / PI;
    // pitch value
    return result;
}

//FUNCTION GET GYRO ANGLE
void getAngleG() {
    const double CONST = 131;
    g_ysum = g_ysum / CONST;
}

//FUNCTION COMPLEMENTARY FILTER
float filterCMP(float argv1, float argv2) {
    //argv1 will be aAngle
    //argv2 will be g_ysum
    float A, G;
    A = argv1;
    G = argv2;

    const float ALPHA = 0.96;
    filter_gyro_Result = Result + argv2 * dt;
    Result = ALPHA * filter_gyro_Result + (1 - ALPHA) * A;

    //  Result = ALPHA * (Result + argv2 * dt) + (1-ALPHA) * A;

}

void DT_setup() {
    t_prev = millis();
}
void getDT() {
    t_now = millis();
    dt = (t_now - t_prev) / 1000.00;
    t_prev = t_now;
}

void datafilter() {
    int i;
    a_xsum = 0;
    a_ysum = 0;
    a_zsum = 0;
    g_xsum = 0;
    g_ysum = 0;
    g_zsum = 0;

    for (i = 0; i < 10; i++) {
        getRawdata();
        a_xsum += a_x;
        a_ysum += a_y;
        a_zsum += a_z;

        g_xsum += g_x;
        g_ysum += g_y;
        g_zsum += g_z;
    }
    a_xsum = a_xsum / 10 - inita_x;
    a_ysum = a_ysum / 10 - inita_y;
    a_zsum = a_zsum / 10;
    float zOff = 16384 - inita_z;
    a_zsum = a_zsum + zOff;

    g_xsum = g_xsum / 10 - initg_x;
    g_ysum = g_ysum / 10 - initg_y;
    g_zsum = g_zsum / 10 - initg_z;

    if (a_xsum < -16384) {
        a_xsum = -16384;
    }
    else if (a_xsum > 16384) {
        a_xsum = 16384;
    }
    if (a_ysum < -16384) {
        a_ysum = -16384;
    }
    else if (a_ysum > 16384) {
        a_ysum = 16384;
    }
    if (a_zsum < -16384) {
        a_zsum = -16384;
    }
    else if (a_zsum > 16384) {
        a_zsum = 16384;
    }
}

void IMU_setup() {
    int i;
    for (i = 0; i < 50; i++) {
        getRawdata();
        inita_x += a_x;
        inita_y += a_y;
        inita_z += a_z;
        initg_x += g_x;
        initg_y += g_y;
        initg_z += g_z;
    }
    inita_x = inita_x / 50;
    inita_y = inita_y / 50;
    inita_z = inita_z / 50;
    initg_x = initg_x / 50;
    initg_y = initg_y / 50;
    initg_z = initg_z / 50;
}

//FUNCTION ITOA
void ITOA(unsigned long a) {
    itoa(a, buf, 10);

    //integer aval is convert to string 
    for (int j = 0; j < 10; j++)       //send string
    {
        if (buf[j] == '\0') break;
        sendChar1(buf[j]);
    }
    sendChar1(' ');
}
