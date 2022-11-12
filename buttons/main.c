#include "msp430g2553.h"


void initI2C();
void initButtons();

#define ADDRESS 0x46
#define false 0
#define true 1

unsigned char bSendButton = false;
unsigned char uRegister;
unsigned char arButtonBuffer[8];
unsigned char uButtonIndex = 0;
unsigned char uButtonSize = 0;

unsigned char arButtonBits[] = { BIT0, BIT1, BIT2, BIT3 };


/****************************************************************************
 * Pinout
 *  p1.0    Output Col 1
 *  p1.1    Output Col 2
 *  p1.2    Output Col 3
 *  p1.3    Output Col 4
 *  p1.6    SCL
 *  p1.7    SDA
 *
 *  p2.0    Input Row 1
 *  p2.1    Input Row 2
 *  p2.2    Input Row 3
 *  p2.3    Input Row 4
 ****************************************************************************/
void main( void )
{
   WDTCTL = (WDTPW | WDTHOLD);                               // Stop watchdog timer

   initI2C();
   initButtons();

   __bis_SR_register( GIE );                                 // Enable global interrupts

   while (true) {
      __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
   }
}

void initI2C()
{
   P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
   P1SEL2 |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

   UCB0CTL1 |= UCSWRST;                      // Enable SW reset
   UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C slave, synchronous mode
   UCB0I2COA = ADDRESS;                      // Set Address
   UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

   IE2 |=  UCB0RXIE;   //
}

void initButtons()
{
   // Output
   P1DIR |= BIT0 + BIT1 + BIT2 + BIT3;     // Set to output
   P1OUT |= BIT0 + BIT1 + BIT2 + BIT3;     // Output high

   // Input
   P2IE |=  BIT0 + BIT1 + BIT2 + BIT3;     // Enable interrupt
   P2IES &= ~(BIT0 + BIT1 + BIT2 + BIT3); // Low to high transition
   P2REN |= BIT0 + BIT1 + BIT2 + BIT3;    // Enable Pull Up/Down Resistors
   P2OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3); // If Pull Up/Down are enabled, this determines which direction those are (0 pull down, 1 pull up)
   // Clear IFG
   P2IFG &= ~BIT0;
   P2IFG &= ~BIT1;
   P2IFG &= ~BIT2;
   P2IFG &= ~BIT3;

   // Timer to start listening again. Default 1mhz, we want 0.1sec
   // 1,000,000hz / 2 = 500,000hz
   // Set timer to 1/10 of that, 50000, we get 0.1 sec
   CCTL0 = CCIE;                             // CCR0 interrupt enabled
   CCR0 = 50000;                             // CCR0 value of 12500
   TACTL = TASSEL_2 + MC_0 + ID_3;//ID_1;                  // SMCLK, off for now, divider of 2
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
   if (bSendButton) {
      // Send byte
      UCB0TXBUF = arButtonBuffer[uButtonIndex++];
      bSendButton = false;
      IE2 &= ~UCB0TXIE;
   } else  {
      uRegister = UCB0RXBUF;

      if (uButtonIndex == uButtonSize) {
         uButtonIndex = 0;
         uButtonSize = 1;
         arButtonBuffer[0] = 255;
      }
      bSendButton = true;
      IE2 |= UCB0TXIE;                          // Enable i2c TX interrupt
   }
}

// Port 2 interrupt service routine
unsigned char button;
unsigned char bit;
unsigned char offset;
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
   P2IE &=  ~(BIT0 + BIT1 + BIT2 + BIT3); // Turn off interrupt
   P1OUT &=  ~(BIT0 + BIT1 + BIT2 + BIT3); // Turn off output

   if (P2IFG & BIT0) {
      bit = BIT0;
      offset = 0;
   } else if (P2IFG & BIT1) {
      bit = BIT1;
      offset = 4;
   } else if (P2IFG & BIT2) {
      bit = BIT2;
      offset = 8;
   } else {//if (P2IFG & BIT3) {
      bit = BIT3;
      offset = 12;
   }

   P2IFG &=  ~(BIT0 + BIT1 + BIT2 + BIT3);
   for (button = 0; button < 4; button++) {
      P1OUT |= arButtonBits[button];
      __delay_cycles(1);
      if (P2IN & bit) {
         break;
      }
      P1OUT &= ~arButtonBits[button];
   }
   // If we couldn't find a button, turn everything back on and return
   if (button == 4) {
      P1OUT |= BIT0 + BIT1 + BIT2 + BIT3; // Turn on output
      __delay_cycles(1);
      P2IE |=  BIT0 + BIT1 + BIT2 + BIT3; // Turn on interrupt
      return;
   }

   P1OUT &= ~arButtonBits[button];
   if (uButtonIndex == uButtonSize) {
      uButtonIndex = 0;
      uButtonSize = 1;
      arButtonBuffer[0] = button + offset;
   } else {
      arButtonBuffer[uButtonSize++] = button + offset;
   }

   TACTL |= MC_2; // Start the timer
}

// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
   TACTL &= ~MC_3; // Turns it off
   P1OUT |= BIT0 + BIT1 + BIT2 + BIT3; // Turn on output
   __delay_cycles(1);
   P2IE |=  BIT0 + BIT1 + BIT2 + BIT3; // Turn on interrupt
}

