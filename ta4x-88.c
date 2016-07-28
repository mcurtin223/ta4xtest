/*******************************************************
This program was created by the
CodeWizardAVR V3.22 Advanced
Automatic Program Generator
© Copyright 1998-2015 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 8/4/2015
Author  : 
Company : 
Comments: 


Chip type               : ATmega88A
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/


#include <mega88a.h>
#include <io.h>
#include <iobits.h>

#define OFF 0
#define ON 1

// LED drives
#define LEDOFF 1
#define LEDON 0
#define auto_switch PINB.0        // Port B0 Auto/Man Switch Input
#define auto_led PORTB.1          // Port B1 Auto/Man LED
#define take1_led PORTD.7          // Port D7 Take 1 LED
#define take2_led PORTD.6          // Port D7 Take 1 LED
#define take3_led PORTD.5          // Port D7 Take 1 LED
#define take4_led PORTD.4          // Port D7 Take 1 LED
#define present1_led PORTD.0        // PORT D0 Present LED
#define present2_led PORTD.1
#define present3_led PORTD.2
#define present4_led PORTD.3

// Select Input Switch Names, Inputs
#define take1_switch PINB.2     // Take 1 switch
#define take2_switch PINB.3     // Take 2 switch
#define take3_switch PINB.4     // Take 3 switch
#define take4_switch PINB.5     // Take 4 switch

#define rx1_in PINC.0       // RX1 Opto Input
#define rx2_in PINC.1       // RX2 Opto Input
#define rx3_in PINC.2       // RX3 Opto Input
#define rx4_in PINC.3       // RX4 Opto Input


// Multiplexer Control lines, Outputs
#define mux_a PORTB.6    // mux A control
#define mux_b PORTB.7    // mux B control
#define mux_enable PORTC.4   // mux enable control Active low = ON

// Declare your global variables here
volatile unsigned int tic;          // Heartbeat counter
volatile unsigned int tic_last;     // save a copy for RX presence test
volatile unsigned char tic_faster;     // just a counter for the leds blinking
volatile unsigned char tic_medium;



unsigned char switches;     // test for switch inputs
unsigned char rx_status;    // presence status(4 bits) 0=no, 1=yes 

bit go_switch;          //input switch valid flag.  Was low and now high
bit switch_primed;      // the input switch low has been detected and appears valid, must go high now
bit go_auto;            // go_auto and auto_primed are for switch detection only
bit auto_primed;
bit auto_status;

bit rx1_last;       // RX opto input last status. Use for input sensing
bit rx2_last;
bit rx3_last;
bit rx4_last;
bit rx1_primed;
bit rx2_primed;
bit rx3_primed;
bit rx4_primed;

unsigned char switch_status[4];    //array of switch status bits in byte form
unsigned char switch_now[4];
unsigned char switch_last[4];

//************************************************************************
// * Name: auto_poll()
// * Purpose: Poll the Input Switch bit
// * Input:  none
// * Comments:  if an input Low (0) is detected timer0 is started,
// * Comments:  an interupt is generated 50-100 ms later to recheck input
// * Output: The 'ok_to_switch' bit is set if standy is on
// ************************************************************************/
void auto_poll(void)
{
    if(!auto_switch)        // check for a switch closure  */
     {                      // This Control Register value directly effects becounce time.
        TCCR0B=0x05;        // start timer0 counting at 5=ck/1024. 4=ck/256, 3=ck/64 */
        SETBIT(TIMSK0,0);   // sets TIMSK.0 directly.  must include iobits.h              
    }
}


//************************************************************************
// * Name: inputswitch_poll()
// * Purpose: Poll the Input Switch bits
// * Input:  Input switches at: PB2-5
// * Comments:  if an input Low (0) is detected timer0 is started, prescaler debounces
// * Comments:  an interupt is generated 1-33 ms later to recheck input
// * Output: The 'ok_to_switch' bit is set if standy is on
// ************************************************************************/
void inputswitch_poll(void)
{                          
        if(!take1_switch |!take2_switch | !take3_switch | !take4_switch)    // check for any switch closure on Take inputs  */
        {
        switch_now[0] = take1_switch;
        switch_now[1] = take2_switch;
        switch_now[2] = take3_switch;
        switch_now[3] = take4_switch;    
                                // This Control Register value directly effects becounce time.
        TCCR0B=0x05;            // start timer0 counting at 5=ck/1024. 4=ck/256, 3=ck/64 */
        SETBIT(TIMSK0,0);    
            // TIMSK0 |= (1<<TOIE0);   //**** this will also enable bit TIOE0
            // TIMSK0=0x01;        // enable timer0 interrupt, another way to set TIMSK bit position 0 to 1
        }   
}

/************************************************************************
// * Name: rx1_presence_test()
// * Purpose: Poll the Input RX Opto Input
// * Input:  Timer2 prescaler determines tic period. For Clk/8(256us), rx counter average 22
// * Comments:   RX1 (PortC.0) input scanned for ups and downs
// * Comments:   Count the pules on RX1
// * Output: keep track of opto RX activity. Presense LED's set
// ************************************************************************/
void rx1_presence_test(void)
{    
    unsigned char rx1_count_now = 0;     // counts the pulses on RX1 opto input
    tic_last = tic;     //save tic counter   

    while(tic_last == tic)  //stay here for 1 tic period (
    {
        rx1_last = rx1_in;  //capture rx1 input level
        
        if(!rx1_count_now)  // if same, no activity on RX1 opto input
        {
           rx1_primed = OFF;        // appears we have no activity, affirm that.
        }
        else
        {    
            present1_led = LEDON;       // turn led on
            rx1_primed = ON;              
        }
        
        if(rx1_last != rx1_in)
        {
            rx1_count_now++;        // increment the input pulse count
        }     
    }
    
    if(!rx1_primed)  // if no RX1 activity shut the led off.
    {
        present1_led = LEDOFF;       // turn led OFF
    }       
}

/************************************************************************
// * Name: rx2_presence_test()
// * Purpose: Poll the Input RX Opto Input
// * Input:  Polled
// * Comments:  RX2 (PortC.1) input scanned for ups and downs
// * Comments:   Count the pules on RX2
// * Output: keep track of opto RX activity.  Presence LED's set
// ************************************************************************/
void rx2_presence_test(void)
{    
    unsigned char rx2_count_now = 0;       
    tic_last = tic;     //save tic counter        

    while(tic_last == tic)  //stay here for 1 tic period
    {
        rx2_last = rx2_in;  //capture rx1 input level
        
        if(!rx2_count_now)  // if same, no activity on RX1 opto input
        {
           rx2_primed = OFF;
        }
        else
        {    
            present2_led = LEDON;       // turn led on
            rx2_primed = ON;           
        }
        
        if(rx2_last != rx2_in)
            rx2_count_now++;        // increment the input pulse count
    }
    
     if(!rx2_primed)  // if no RX1 activity shut the led off.
    {
        present2_led = LEDOFF;       // turn led OFF
    }          
}                    

/************************************************************************
// * Name: rx3_presence_test()
// * Purpose: Poll the Input RX Opto Input
// * Input:  Polled
// * Comments:  RX3 (PortC.2) input scanned for ups and downs
// * Comments:   Count the pules on RX3
// * Output: keep track of opto RX activity.  Presence LED's set
// ************************************************************************/
void rx3_presence_test(void)
{    
    unsigned char rx3_count_now = 0;        
    tic_last = tic;     //save tic counter        

    while(tic_last == tic)  //stay here for 1 tic period
    {
        rx3_last = rx3_in;  //capture rx1 input level
        
        if(!rx3_count_now)  // if same, no activity on RX1 opto input
        {
           rx3_primed = OFF;
        }
        else
        {    
            present3_led = LEDON;       // turn led on
            rx3_primed = ON;          
        }
        
        if(rx3_last != rx3_in)
            rx3_count_now++;        // increment the input pulse count
    }
    
    if(!rx3_primed)  // if no RX1 activity shut the led off.
    {
        present3_led = LEDOFF;       // turn led OFF
    }            
} 

/************************************************************************
// * Name: rx4_presence_test()
// * Purpose: Poll the Input RX Opto Input
// * Input:  Polled
// * Comments:  RX4 (PortC.3) input scanned for ups and downs
// * Comments:  Count the pules on RX4
// * Output: keep track of opto RX activity.  Presence LED's set
// ************************************************************************/
void rx4_presence_test(void)
{    
    unsigned char rx4_count_now =0;    
    tic_last = tic;     //save tic counter        

    while(tic_last == tic)  //stay here for 1 tic period
    {
        rx4_last = rx4_in;  //capture rx1 input level
        
        if(!rx4_count_now)  // if same, no activity on RX1 opto input
        {
           rx4_primed = OFF;
           //present4_led = LEDOFF;       // turn led off
        }
        else
        {    
            present4_led = LEDON;       // turn led on
            rx4_primed = ON;          
        }
        
        if(rx4_last != rx4_in)
            rx4_count_now++;        // increment the input pulse count
    }
    
    if(!rx4_primed)  // if no RX1 activity shut the led off.
    {
        present4_led = LEDOFF;       // turn led OFF
    }            
} 


/************************************************************************
// * Name: rx_presence_status()
// * Purpose: assemble the 4 rx presence status bits
// * Input:  Polled
// * Comments:  
// * Output: keep track of opto RX activity.  Presence LED's set
// ************************************************************************/
void rx_presence_status(void)
{    
    rx_status = 0;
    if(!present1_led && rx1_primed)   // if led is set (0), there is activity on Receiver
        {
            rx_status |= 0x01;
            rx2_primed = OFF;
            rx3_primed = OFF;
            rx4_primed = OFF;
        }
    if(!present2_led && rx2_primed)
        {
            rx_status |= 0x02;
            rx1_primed = OFF;
            rx3_primed = OFF;
            rx4_primed = OFF;
        }
    if(!present3_led && rx3_primed)
        { 
            rx_status |= 0x04;
            rx1_primed = OFF;
            rx2_primed = OFF;
            rx4_primed = OFF;
        }
    if(!present4_led && rx4_primed)
        {
            rx_status |= 0x08;
            rx1_primed = OFF;
            rx2_primed = OFF;
            rx3_primed = OFF;
        }
}
/************************************************************************
// * Name: cycle_leds()
// * Purpose: super fancy led swirling
// * Input:  no activity on RX inputs.
// * Comments: bypassed if not in auto mode  
// * Output: 
// ************************************************************************/
void cycle_leds(void)
{    
        
    switch(tic_medium)
    {
        case 0x10:
            present1_led = LEDOFF;  // set the LED's      
            present2_led = LEDOFF;
            present3_led = LEDOFF;
            present4_led = LEDON;
            break;
        case 0x18:
            present1_led = LEDOFF;  // set the LED's      
            present2_led = LEDOFF;
            present3_led = LEDON;
            present4_led = LEDOFF;
            break;
        case 0x20:
            present1_led = LEDOFF;  // set the LED's      
            present2_led = LEDON;
            present3_led = LEDOFF;
            present4_led = LEDOFF;
            break;
        case 0x28:
            present1_led = LEDON;  // set the LED's      
            present2_led = LEDOFF;
            present3_led = LEDOFF;
            present4_led = LEDOFF;
            break;
        default:
            if(tic_medium > 0x30)
                tic_medium = 0x00;
            break;
    }
           
        mux_enable = OFF;   //turn on the muxer
} 

// ************************************************************************
// * Name: mux_write(unsigned char)
// * Purpose: writes to mux control lines of 74HC4052 (dual 4X)
// * Input: input number to switch to (0-3)
// * Comments:  only uses the first two bits
// * Comments:   0000 0011
// * Output: writes/updates the mux control lines
// ************************************************************************/
void mux_write(unsigned char mux)
{
	mux_a = mux & 1;    // mask unused bits
	mux_b = mux & 2;
}


// ************** Timer 0 overflow interrupt service routine *********************
// * This is used for Input Switch debouncing
// * Clock of 8MHz / 1024 = .128ms.  Overflow at 0xff = 33ms delay
// * Clock of 8MHz / 64 = 8 us.  Overflow at 0xff = 2ms delay
// * We arrive here 33ms after a input switch is detected (with TCCR0=0x05)
// * As long as a switch remains low it's not ready.
// *******************************************************************************
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{    
    CLRBIT(TIMSK0,0);   // Disable timer0 interrupt (0x01 enables), clears bit TOIE0 
                        //TIMSK |= (0<<TOIE0); 
    TCCR0B=0x00;        // stop timer0 counting, clear prescaler
    
    if(!auto_switch)    // is the AUTO switch pressed
    {
        go_auto = OFF;
        auto_primed = ON;
    }
    else     
    {
        if(auto_primed)      // if auto_primed is set, switch is debounced, no longer pressed, now its ready
        {
            go_auto = ON;
            auto_primed = OFF;            
        }
    }
        
    if(!take1_switch |!take2_switch | !take3_switch | !take4_switch)    // Is any switch still low?
    {
        switch_last[0] = switch_now[0];
        switch_last[1] = switch_now[1];
        switch_last[2] = switch_now[2];
        switch_last[3] = switch_now[3];
        
        go_switch = OFF;
		switch_primed = ON;
	}
	else
	{
		if(switch_primed)	 // if no active switch, then ready to trigger a switch, via go_switch
		{
			switch_status[0] = switch_last[0];
            switch_status[1] = switch_last[1];
            switch_status[2] = switch_last[2];
            switch_status[3] = switch_last[3];
            
            go_switch = ON;
			switch_primed = OFF;
		}
	}    
}

// ************** Timer 2 overflow interrupt service routine *********************
// * This is used for heart beat clock 
// * Clock of 8MHz/256 = 32us(31250 hz) * 0xff = 122 hz . TCCR2B 110
// * Clock of 8MHz/8 = 1us(1000000 hz) * 0xff = 3906 hz . TCCR2B 010
// * Timer2 overflow interrupt service routine 
// *******************************************************************************

interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
    tic++;          // bump 16 bit heartbeat counter
    tic_faster++;   // bump a 8 bit counter
    if(tic_faster == 0)
        tic_medium++;     
}
   

void main(void)
{
// Declare your local variables here
unsigned char mux_counter;        // used by mux_write counter
unsigned char mux_num;

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=In 
DDRB=(1<<DDB7) | (1<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (1<<DDB1) | (0<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=P Bit4=P Bit3=P Bit2=P Bit1=0 Bit0=P 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (1<<PORTB5) | (1<<PORTB4) | (1<<PORTB3) | (1<<PORTB2) | (0<<PORTB1) | (1<<PORTB0);

// Port C initialization
// Function: Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(1<<DDC6) | (1<<DDC5) | (1<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit6=T Bit5=0 Bit4=0 Bit3=P Bit2=P Bit1=P Bit0=P 
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (1<<PORTC3) | (1<<PORTC2) | (1<<PORTC1) | (1<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
// Mode: Normal top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: clk/8
// Mode: Normal top=0xFF
// OC2A output: Disconnected
// OC2B output: Disconnected
// Timer Period: .256ms
ASSR=(0<<EXCLK) | (0<<AS2);
TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
TCCR2B=(0<<WGM22) | (0<<CS22) | (1<<CS21) | (0<<CS20);      //set timer prescaler 010= clk/8 * 256 = 256us
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2);


// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: Off
EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
EIMSK=(0<<INT1) | (0<<INT0);
PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);    // disable any pin change interrupts

// USART initialization
// USART disabled
UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
ADCSRB=(0<<ACME);
// Digital input buffer on AIN0: On
// Digital input buffer on AIN1: On
DIDR1=(0<<AIN0D) | (0<<AIN1D);

// ADC initialization
// ADC disabled
ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);

// Watchdog Timer initialization RESET WDCE & WDE to 0.  Dont need 
// Watchdog Timer Prescaler: OSC/2k
// Watchdog timeout action: Reset
#pragma optsize-
WDTCSR=(0<<WDIF) | (0<<WDIE) | (0<<WDP3) | (0<<WDCE) | (0<<WDE) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
WDTCSR=(0<<WDIF) | (0<<WDIE) | (0<<WDP3) | (0<<WDCE) | (0<<WDE) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

#asm("sei")     // Global enable interrupts   "cli" disable all interrupts

PORTD = 0xff;       // turn off all input and presense LED's

// Initialize the mux IC.  74HC4052
mux_a = 0;
mux_b = 0;
mux_enable = 0; // enable 74HC4052
go_auto = OFF;  // turn off the auto mode
auto_led = ON;  // turn off the auto led
auto_status = OFF;  

switch_status[3] = 1;
switch_status[2] = 1;
switch_status[1] = 1;
switch_status[0] = 0;

go_switch = ON;
switch_primed = OFF;
        
while (1)
{        
    auto_poll();        // check and set the Auto Sense switch
    inputswitch_poll(); // check and set input switches 1-4

    rx1_presence_test();      // test RX-1 thru RX-4 Opto input for active signals.
    rx2_presence_test();
    rx3_presence_test();
    rx4_presence_test();    
    rx_presence_status();   // collect the rx presence bit into a status byte
               
    
    if(go_auto) // toggle the auto switch function
        {   
            auto_led = !auto_led; //toggle the Auto LED, port b1 led.
            if(!auto_led)
                auto_status = ON;   // if led is set(on) were in auto sense mode            
            else
                auto_status = OFF;
                    
            go_auto = OFF;
            auto_primed = OFF;
        }
    
    if(auto_status)
        {
             if(rx_status & 0x01)
             {
                mux_write(0x00);
                take1_led = LEDON;  // set the LED's      
                take2_led = LEDOFF;
                take3_led = LEDOFF;
                take4_led = LEDOFF;
             }   
             
             else if(rx_status & 0x02)
             {   
                mux_write(0x01);
                take1_led = LEDOFF;  // set the LED's      
                take2_led = LEDON;
                take3_led = LEDOFF;
                take4_led = LEDOFF;
             }
                
             else if(rx_status & 0x04)
             { 
                mux_write(0x02);
                take1_led = LEDOFF;  // set the LED's      
                take2_led = LEDOFF;
                take3_led = LEDON;
                take4_led = LEDOFF;
             }
                
             else if(rx_status & 0x08)
             {
                mux_write(0x03);
                take1_led = LEDOFF;  // set the LED's      
                take2_led = LEDOFF;
                take3_led = LEDOFF;
                take4_led = LEDON;
             }
             
             if(rx_status == 0)
                 cycle_leds();  // blink the stupid leds   
        }
            
    if(go_switch & !auto_status)	// do we have a valid switch input
  	    {
            switches = 0;                  
            switches = switch_status[3];   // assemble 4 switch bits by shifting
            switches <<= 1;
            switches |= switch_status[2];
            switches <<= 1;
            switches |= switch_status[1];
            switches <<= 1;
            switches |= switch_status[0];
        
            switches ^= 0x0f;  // expose only the changed bits.  allow only 1 active switch. reset multiples 
            if(switches >= 0x08)
                switches = 0x08;
            else if(switches >= 0x04)
                switches = 0x04;
            else if(switches >=0x02)
                switches = 0x02;
            else if(switches == 1)
                switches = 0x01;
                
            switches = ~switches;  // invert the bits for proper LED display (0 == active/illuminated
           
            for(mux_counter=0;mux_counter<4;mux_counter++)
            {
                switch_status[mux_counter]= switches & 0x01; //shift bits and write to status array
                switches >>= 1;
            }
                      
            // which switch input is active
            take1_led = switch_status[0];  // set the LED's      
            take2_led = switch_status[1];
            take3_led = switch_status[2];
            take4_led = switch_status[3];
            
            go_switch = OFF;
	        switch_primed = OFF;
            // only one switch can be active and valid, now write that one valid switch to the audio mux
            for(mux_counter=0;mux_counter<4;mux_counter++)
            {
                mux_num = switch_status[mux_counter]; // see which bit is low
                if (!mux_num)
                {
                    mux_write(mux_counter);
                }
            }    
	    }
        
}
}