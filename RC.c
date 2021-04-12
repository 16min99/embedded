//PORTA 선언
#define PORTAO (*((volatile unsigned char*)0x404)) //output
#define DIRA (*((volatile unsigned char*)0x400)) //DIRA
#define PORTAI (*((volatile unsigned char*)0x408)) //input

//uart 통신 핀 정의 (PD핀)
#define DIRC (*((volatile unsigned char*)0x440))
#define RXD (*((volatile unsigned char*)0x820)) 
#define TXD (*((volatile unsigned char*)0x822))
#define STATUS (*((volatile unsigned char*)0x824))
#define CTRLA (*((volatile unsigned char*)0x825))
#define CTRLB (*((volatile unsigned char*)0x826))
#define CTRLC (*((volatile unsigned char*)0x827)) //0=disabled , 1=Reserveed,2=EVEN,3=ODD    datasheet p302
#define BAUD1 (*((volatile unsigned char*)0x828))
#define BAUD2 (*((volatile unsigned char*)0x829))


//ADC register
#define ADC_CTRLA (*((volatile unsigned char *)0x600))
#define ADC_CTRLC (*((volatile unsigned char *)0x602))
#define ADC_MUXPOS (*((volatile unsigned char *)0x606))
#define ADC_COMMAND (*((volatile unsigned char *)0x608))
#define ADC_INTFLAGS (*((volatile unsigned char *)0x60b))
#define ADC_RESL (*((volatile unsigned char *)0x610))

//RTC 핀
#define RTC_CTRLA (*((volatile unsigned char*)0x140))
#define RTC_CLKSEL (*((volatile unsigned char*)0x147)) 
#define RTC_PITCTRLA (*((volatile unsigned char*)0x150))
#define RTC_PITINTCTRL (*((volatile unsigned char*)0x152))
#define RTC_PITINTFLAGS (*((volatile unsigned char*)0x153))

//함수정의
unsigned char uart_rec(void);
void uart_send(char data);
void init_UART(void);
void RTC_init();
void hzdelay(void);
void pwm();
uint8_t readADC();
void ADC_init();


void setup() {
    RTC_init();
    DIRA = 0b11111111;
 ADC_init();
}
void loop() {
   pwm(readADC());
}


//set rtc
void RTC_init() {
    RTC_CTRLA = 0x01; //no prescale
    RTC_CLKSEL = 0x00; // INT32k 주파수 
    RTC_PITCTRLA = 0b00001001   ;  //
}

void hzdelay(void) {
    while ((RTC_PITINTFLAGS & 0x01) == 0);
    RTC_PITINTFLAGS = 0x01;
}



void pwmH(int pv) {
    int i;
    for (i = 0; i < pv; i++)
    { 
        hzdelay();
    }
}
void pwmL(int pv) {
    int i;
    for (i = 0; i < 255 - pv; i++)
    {
        hzdelay();
    }
}


void pwm(int pv) {
    PORTAO |= 0b00000010;
    pwmH(pv);
    PORTAO &= 0b11111101;
    pwmL(pv);    
}




void init_UART(void) {
    DIRC = 0b00000001;//rx핀을 입력으로 tx핀을 출력으로
    CTRLA = 0b10100000;// 송수신 조건문 RXCIE,DREIE 인터럭트 플래그 사용
    CTRLB = 0b11000000; // rxen,txen 활성화 rxmode:노말모드
    CTRLC = 0b00000011; // 비동기화모드,Parity사용X,stop비트 1, 8bit사용
    BAUD1 = 0b00001010;
    BAUD2 = 0b00011010; //baud 계산시 6666
}


unsigned char uart_rec(void) {
    while ((STATUS & 0x80) == 0); // 데이터 수신확인까지 기다린다.
    return RXD;
}


void uart_send(char data) {
    while ((STATUS & 0x20) == 0); // 데이터 송신확인까지 기다린다.

    if (0x40 < data && data < 0x5B) TXD = data + 32; // 아스키코드로 대문자->소문자
    else if (0x60 < data && data < 0x7B) TXD = data - 32; // 아스키코드로 소문자->대문자
    else TXD = data; //나머지그대로
}

uint8_t readADC()
{
    ADC_COMMAND = B00000001;              //start conversion
    while (!(ADC_INTFLAGS & B00000001)); //if result is empty, wait
    ADC_INTFLAGS = B00000001;
    return ADC_RESL;
}



void ADC_init()
{
    ADC_MUXPOS = B00000110;     //use pd6 pin for adc
    ADC_CTRLA = B00000101;      //8bit resolution, ADC is enabled
    ADC_CTRLC = B00010011;      //Vref = VDD(5V), CLK_PER divided by 8
}