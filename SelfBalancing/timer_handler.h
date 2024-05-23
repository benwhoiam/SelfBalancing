namespace timer {
  void setup() {
    TWBR = 12;  // scr1.jpg -> SCL freq = CPUspeed(==16) / (16 +2*TWBR*PrescalerValue(==1)) --> clck spd = 400kHz
    // To create a variable pulse for controlling the stepper motors a timer is created
    // that will execute a piece of code (subroutine) every 20us
    TCCR2A = 0;  // scr2
    TCCR2B = 0;
    TIMSK2 |= (1 << OCIE2A);  //scr3 --> Set the interupt enable bit OCIE2A in the TIMSK2 register
    TCCR2B |= (1 << CS21);    //scr4 --> Set the CS21 bit in the TCCRB register to set the prescaler to 8 (scr 5)
    // hence now clck pulse (freq) is 2MHz (16/8)
    OCR2A = 39;              // donc T=1/freq = 0.5us, but want 20us, donc multiply but 40-1 (CR -> compare register)
    TCCR2A |= (1 << WGM21);  // scr6 --> Set counter 2 to CTC (clear timer on compare), bima3na, if match, rdtimer 0, call inturrept routine
  }
}