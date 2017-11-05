#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include "MPU9250.h"
#include "MPU9250_reg.h"


#define UART_PRINTF

#ifdef UART_PRINTF
int fputc(int _c, register FILE *_fp);
int fputs(const char *_ptr, register FILE *_fp);
#endif

//memory readout
int mem_read = 0;
//PMT monitoring
int pmt1_stat,pmt2_stat,pmt3_stat,pmt_stat;


//Telecommand
int comm_read = 0;
//MEMORY
int add1,add2;
int address = 0x00000000;
int data,manid,devid,mem_id;
int status;//status reg

int success;//chip erase
int read_buf[1] = {0};
int read_buf_max[256] = {0};//putting max num = 256 for reading from memory
int hua; //Writing to memory

//Timeout
int counter = 0;


//IMU
int imu_id;
int success_imu;

//accel and gyro word declaration
int accel_lx,accel_hx,accel_ly,accel_hy,accel_lz,accel_hz;
int gyro_lx,gyro_hx,gyro_ly,gyro_hy,gyro_lz,gyro_hz;
int accel_off_lx,accel_off_hx,accel_off_ly,accel_off_hy,accel_off_lz,accel_off_hz;
long accel_off_x,accel_off_y,accel_off_z;

int gyro_off_lx,gyro_off_hx,gyro_off_ly,gyro_off_hy,gyro_off_lz,gyro_off_hz;
long gyro_off_x,gyro_off_y,gyro_off_z;
long accel_x,accel_y,accel_z;
long gyro_x,gyro_y,gyro_z;

//For GPS
char gps_buf[82] = {"0"};
int time_hh, time_mm,time_ss;
int lat_min,lat_sec1,lat_sec2,lat_sec3;
char lat_hem,long_hem;
int long_min,long_sec1,long_sec2,long_sec3;
float anten_height,geo_height;

//For BMP
int success_bmp;
unsigned AC1a,AC1b,AC2a,AC2b,AC3a,AC3b,AC4a,AC4b,AC5a,AC5b,AC6a,AC6b;//8 bits
short AC1,AC2,AC3;
unsigned short  AC4,AC5,AC6;
unsigned B1a,B1b,B2a,B2b,MBa,MBb,MCa,MCb,MDa,MDb;
short B1,B2,MB,MC,MD;
unsigned bmp_id;
unsigned temp_lower, temp_upper, pres_lower, pres_upper,pres_third;
long  temp_final, pres_final; //uncompensated

//MEMORY FUNCTIONS
void Init_mem()
{
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    P3SEL |= 0x31;                            // P3.5,4,0 option select


    //GENERATE CHIP SEL 5.1 chip sel
    P5SEL &= 0xFD;
    P5DIR |= 0x02;
    P5OUT |= 0x02;

    UCA0CTL0 |= UCSYNC + UCMSB + UCMST + UCCKPL;     // 3-pin, 8-bit SPI master, Clock polarity high, MSB config is for master MSP430

    UCA0CTL1 |= UCSSEL_2;
    UCA0BR0 = 0x00;                         //1MHz clock
    UCA0BR1 = 0x00;
    UCA0MCTL = 0;                             // No modulation

    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

}

int Read_ID ()      //Returns device ID in decimal memory
{
    //reading chip ID

    while (!(UCA0IFG&UCTXIFG));
    P5OUT &= 0xFD; //cs# active
    UCA0TXBUF = 0x90;                     // Transmit first character
    __delay_cycles(10);

    while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
    data = UCA0RXBUF;
    UCA0TXBUF = 0x00;
    __delay_cycles(10);

    while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
    data = UCA0RXBUF;
    UCA0TXBUF = 0x00;
    __delay_cycles(10);

    while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
    data = UCA0RXBUF;
    UCA0TXBUF = 0x00;
    __delay_cycles(10);

    while (!(UCA0IFG&UCTXIFG));
    data = UCA0RXBUF;
    UCA0TXBUF = 0x00;

    while (!(UCA0IFG & UCRXIFG));               // USCI_A0 RX buffer ready?
    manid = UCA0RXBUF;
    UCA0TXBUF = 0xFF; // dummy value to read manu id
    __delay_cycles(10);

    while (!(UCA0IFG&UCRXIFG));               // USCI_A0 TX buffer ready?
    devid = UCA0RXBUF;
    P5OUT|= 0x02; //chip sel high
    __delay_cycles(10);

    return devid;

}

int Read_status()   //Returns status reg value in decimal
{
    while (!(UCA0IFG&UCTXIFG));
    P5OUT &= 0xFD;                       //cs# active
    UCA0TXBUF = 0x05;                //read status reg
    __delay_cycles(10);

    while (!(UCA0IFG&UCTXIFG));
    data = UCA0RXBUF;
    UCA0TXBUF = 0x00;

    while (!(UCA0IFG & UCRXIFG));               // USCI_A0 RX buffer ready?
    status = UCA0RXBUF;
    P5OUT|= 0x02;                       //chip sel high

    return status;

}

int Write_enable()  //Returns 1 if successful
{
    while (!(UCA0IFG&UCTXIFG));
    P5OUT &= 0xFD;              //cs# active
    UCA0TXBUF = 0x06;                //write enable
    __delay_cycles(10);

    while (!(UCA0IFG&UCTXIFG));
    data = UCA0RXBUF;                       //to avoid overrun flag
    P5OUT|= 0x02;                    //chip sel high

    //read status reg to check
    int stat;
    stat = Read_status();

    if(stat&02) return 1;       //Write Latch enable
    else return 0;

}

int Chip_erase()        //Returns 1 if successful
{

    int allow = Write_enable();

    if(allow == 1)
    {
        while (!(UCA0IFG & UCTXIFG));
        P5OUT &= 0xFD;              //cs# active
        UCA0TXBUF = 0xC7;                //chip erase
        //__delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));
        data = UCA0RXBUF;                       //to avoid overrun flag
        P5OUT|= 0x02;                    //chip sel high
        __delay_cycles(100);

        while((Read_status()) & 0x02); //waits for chip erase
        return 1;
    }
    else return 0;
}

int Sector_erase(int byte1,int byte2, int byte3)
{

    int allow = Write_enable();

    if(allow == 1)
    {
        while (!(UCA0IFG & UCTXIFG));
        P5OUT &= 0xFD;              //cs# active
        UCA0TXBUF = 0x20;                //chip erase
        __delay_cycles(10);

        //24 bit address = 3 bytes
        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = byte1;//MSB
        __delay_cycles(10);

        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = byte2;//byte 2
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = byte3;//byte 3
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));
        data = UCA0RXBUF;                       //to avoid overrun flag
        P5OUT|= 0x02;                    //chip sel high
        __delay_cycles(100);

        while((Read_status()) & 0x02); //waits for sector erase
        return 1;
    }
    else return 0;
}

int Read_data (int address) //reads only one byte
{
    int stat_reg = Read_status();

    if(stat_reg & 0x01)  //WIP is high
    {
        return 0;
    }
    else
    {
        while (!(UCA0IFG & UCTXIFG));
        P5OUT &= 0xFD;              //cs# active
        UCA0TXBUF = 0x03;                //read data
        __delay_cycles(10);

        //24 bit address = 3 bytes
        int addtemp1,addtemp2,addtemp3;
        addtemp1 = address & 0x000000FF;
        addtemp2 = address & 0x0000FF00;
        addtemp3 = address & 0x00FF0000;
        addtemp2 = addtemp2/4;//2^2
        addtemp3 = addtemp3/16;//2^4

        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp3;//MSB
        __delay_cycles(10);

        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp2;//byte 2
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp1;//byte 3
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));
        data = UCA0RXBUF;//to avoid overrun
        UCA0TXBUF = 0x00;//dummy value to keep clock going

        P5OUT|= 0x02; //chip sel high
        return read_buf[0];
    }
}


int Read_data_num (int address,int num) //reads num bytes
{
    int stat_reg = Read_status();

    if(stat_reg & 0x01)  //WIP is high
    {
        return 0;
    }
    else
    {
        while (!(UCA0IFG & UCTXIFG));
        P5OUT &= 0xFD;              //cs# active
        UCA0TXBUF = 0x03;                //read data
        __delay_cycles(10);

        //24 bit address = 3 bytes
        int addtemp1,addtemp2,addtemp3;
        addtemp1 = address & 0x000000FF;
        addtemp2 = address & 0x0000FF00;
        addtemp3 = address & 0x00FF0000;
        addtemp2 = addtemp2/4;//2^2
        addtemp3 = addtemp3/16;//2^4

        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp3;//MSB
        __delay_cycles(10);

        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp2;//byte 2
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp1;//byte 3
        __delay_cycles(10);
        int i = 0;
        if(num>256) return 0;
        for(i=0;i<num;i++)
        {

        while (!(UCA0IFG&UCTXIFG));
        read_buf_max[i] = UCA0RXBUF;//to avoid overrun
        UCA0TXBUF = 0x00;//dummy value to keep clock going

        }

    }

        P5OUT|= 0x02; //chip sel high
        return 1;
}



int Page_program(int address, int byte)
{
    int allow = Write_enable();

    if(allow == 1)
    {
        while (!(UCA0IFG & UCTXIFG));
        P5OUT &= 0xFD;              //cs# active
        UCA0TXBUF = 0x02;           //page program
        __delay_cycles(10);


        int addtemp1,addtemp2,addtemp3;
        addtemp1 = address & 0x000000FF;
        addtemp2 = address & 0x0000FF00;
        addtemp3 = address & 0x00FF0000;
        addtemp2 = addtemp2/4;//2^2
        addtemp3 = addtemp3/16;//2^4
        //24 bit address = 3 bytes
        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp3;//MSB
        __delay_cycles(10);

        while (!(UCA0IFG & UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp2;//byte 2
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = addtemp1;//byte 3
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
        data = UCA0RXBUF;
        UCA0TXBUF = byte;//data
        __delay_cycles(10);

        while (!(UCA0IFG&UCTXIFG));
        data = UCA0RXBUF;                       //to avoid overrun flag
        P5OUT|= 0x02;                    //chip sel high
        __delay_cycles(10);

        while((Read_status()) & 0x02); //waits for sector erase
        return 1;
    }
    else return 0;
}


//IMU FUNCTIONS

void imu_init()
{
    //set up

    UCB3CTL1 |= UCSWRST;
    P10SEL |= BIT1 | BIT2;                        // Assign I2C pins to USCI_B3 10.1 Data and 10.2 clock
    UCB3CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB3CTL1 = UCSSEL_2 + UCSWRST;           // Enable SW reset, SMCLK
    UCB3BR0 = 3;                             // fSCL = SMCLK/12 = ~100kHz
    UCB3BR1 = 0;
    UCB3CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
}

int read_imu_id()
{
    while(UCB3CTL1 & UCTXSTP);  //stop
    UCB3CTL1 |= UCSWRST;        //reset
    UCB3I2CSA = 0x68;         //slave address 0110 1000
    UCB3CTL1 |= UCTR;           // I2C transmitter
    UCB3CTL1 |= UCTXSTT;        // I2C start condition

    UCB3CTL1 &= ~UCSWRST;
    __delay_cycles(100);


    UCB3TXBUF = 0x75;
    while(UCB3CTL1 & UCTXSTT);        // Wait for START to finish
    __delay_cycles(100);


    while(!(UCB3IFG & UCTXIFG) );    // Wait for transmit to finish
    __delay_cycles(100);
    UCB3CTL1 &= ~UCTR;                // I2C receiver

    UCB3CTL1 |= UCTXSTT; //Repeated start
    while(UCB3CTL1 & UCTXSTT);        // Wait for START to finish
    __delay_cycles(100);
    while((  UCB3IFG & UCRXIFG)==0 );    // Wait for XMIT to finish
    __delay_cycles(100);
   // int data;
    data = UCB3RXBUF;


    UCB3CTL1 |= UCTXSTP;            // I2C stop condition
    while(UCB3CTL1 & UCTXSTP);

    return(data);
}

int read_imu_i2c(unsigned char reg_add)
{
    counter = 0;
    data = 0;
    while(UCB3CTL1 & UCTXSTP)  //stop
    {
        counter++;
        __delay_cycles(5);
        if(counter == 100)
        goto RET_IMU;
    }

    UCB3CTL1 |= UCSWRST;        //reset
    UCB3I2CSA = 0x68;         //slave address 0110 1000
    UCB3CTL1 |= UCTR;           // I2C transmitter
    UCB3CTL1 |= UCTXSTT;        // I2C start condition

    UCB3CTL1 &= ~UCSWRST;
    __delay_cycles(100);


    UCB3TXBUF = reg_add;
    counter = 0;
    while(UCB3CTL1 & UCTXSTT)      // Wait for START to finish
    {
        counter++;
        __delay_cycles(5);
        if(counter == 100)
        goto RET_IMU;

    }

    __delay_cycles(100);


    counter = 0;
    while(!(UCB3IFG & UCTXIFG) )  // Wait for transmit to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_IMU;

     }


    __delay_cycles(100);
    UCB3CTL1 &= ~UCTR;                // I2C receiver

    UCB3CTL1 |= UCTXSTT; //Repeated start

    counter = 0;
    while(UCB3CTL1 & UCTXSTT)       // Wait for START to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_IMU;

     }


    __delay_cycles(100);

    counter = 0;
    while((  UCB3IFG & UCRXIFG)==0 )   // Wait for XMIT to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_IMU;

     }

    __delay_cycles(100);

    data = UCB3RXBUF;


    UCB3CTL1 |= UCTXSTP;            // I2C stop condition

    counter = 0;
    while(UCB3CTL1 & UCTXSTP)
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_IMU;

     }

RET_IMU:    return(data);

}


int write_imu_i2c(unsigned char reg_add,unsigned data)
{
    while(UCB3CTL1 & UCTXSTP);  //stop
    UCB3CTL1 |= UCSWRST;        //reset
    UCB3I2CSA = (0x68);     //slave address
    UCB3CTL1 |= UCTR;           // I2C transmitter
    UCB3CTL1 |= UCTXSTT;        // I2C start condition
    UCB3CTL1 &= ~UCSWRST;
    UCB3TXBUF = reg_add;        // Move reg to TX

    while(UCB3CTL1 & UCTXSTT);        // Wait for START to finish



    while(!(  UCB3IFG & UCTXIFG) );    // Wait for XMIT to finish
    UCB3TXBUF = data ;                  // Move reg to TX
    __delay_cycles(100);
    if (UCB3IFG & UCTXIFG) { UCB3CTL1 |= UCTXSTP;}            // I2C stop condition
    __delay_cycles(100);
    while (UCB3STAT & UCBBUSY);
    while((UCB3IFG & UCTXIFG)==0);    // Wait for XMIT to finish

    //UCB3CTL1 |= UCTXSTP;            // I2C stop condition
    while(UCB3CTL1 & UCTXSTP); //if code stuck here then stop was not sent/data was not transmitted
    return(1);

}

//GPS FUNCTIONS

void set_GPS()
{
    P9SEL = 0x30;                             // P9.4,5 = USCI_A0 TXD/RXD 00110000
    UCA2CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA2CTL1 |= UCSSEL_2;                     // SMCLK
    UCA2BR0 = 6;                             // 1MHz 9600 (see User's Guide)
    UCA2BR1 = 0;                              // 1MHz
    UCA2MCTL |= UCBRS_0 + UCBRF_13 + UCOS16;     //UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=0xEE, UCBRFx=0
    UCA2CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

//BMP180 FUNCTIONS

void i2c_init(void)
{
    UCB0CTL1 &= ~UCSWRST;           // reset = 0
    P3SEL |= BIT1 | BIT2;                        // Assign I2C pins to USCI_B0
    UCB0CTL1 |= UCSWRST;
    UCB0CTL1 = UCSSEL_2 + UCSWRST;           // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
    UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
    UCB0BR1 = 0;
    UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
}

int write_i2c(unsigned char reg_add,unsigned data)
{

    success_bmp = 0;

    counter = 0;
    while(UCB0CTL1 & UCTXSTP)  //stop
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP;
     }

    UCB0CTL1 |= UCSWRST;        //reset
    UCB0I2CSA = (0x77);          //slave address
    UCB0CTL1 |= UCTR;           // I2C transmitter
    UCB0CTL1 |= UCTXSTT;        // I2C start condition
    UCB0CTL1 &= ~UCSWRST;
    UCB0TXBUF = reg_add;        // Move reg to TX

    counter = 0;
    while(UCB0CTL1 & UCTXSTT)    // Wait for START to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP;
     }
    counter = 0;
    while(!(  UCB0IFG & UCTXIFG) )    // Wait for XMIT to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP;
     }
    UCB0TXBUF = data ;                  // Move reg to TX
    __delay_cycles(100);
    if (UCB0IFG & UCTXIFG) { UCB0CTL1 |= UCTXSTP;}            // I2C stop condition
    __delay_cycles(100);

    counter = 0;
    while (UCB0STAT & UCBBUSY)
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP;
     }

    counter = 0;
    while((UCB0IFG & UCTXIFG)==0)   // Wait for XMIT to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP;
     }
    //UCB0CTL1 |= UCTXSTP;            // I2C stop condition
    counter = 0;
    while(UCB0CTL1 & UCTXSTP) //if code stuck here then stop was not sent/data was not transmitted
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP;
     }
    return(0);

RET_BMP: return(1);

}


int read_i2c(unsigned char reg_add)
{
    unsigned data = 0;
    counter = 0;
    while(UCB0CTL1 & UCTXSTP)
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP_R;
     }

    UCB0CTL1 |= UCSWRST;
    UCB0I2CSA = (0x77);
    UCB0CTL1 |= UCTR;                // I2C transmitter
    UCB0CTL1 |= UCTXSTT;            // I2C start condition
    UCB0CTL1 &= ~UCSWRST;
    UCB0TXBUF = reg_add;          // Move reg to TX
    counter = 0;
    while(UCB0CTL1 & UCTXSTT)     // Wait for START to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP_R;
     }

    counter = 0;
    while(!(  UCB0IFG & UCTXIFG) )   // Wait for XMIT to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP_R;
     }

    UCB0CTL1 &= ~UCTR;                // I2C receiver
    UCB0CTL1 |= UCTXSTT; //Repeated start
    counter = 0;
    while(UCB0CTL1 & UCTXSTT)       // Wait for START to finish
    {
         counter++;
         __delay_cycles(5); //0.5 ms
         if(counter == 100)
         goto RET_BMP_R;
     }

    counter = 0;
    while((  UCB0IFG & UCRXIFG)==0 )    // Wait for XMIT to finish
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP_R;
     }

    data = UCB0RXBUF;
    UCB0CTL1 |= UCTXSTP;            // I2C stop condition
    counter = 0;
    while(UCB0CTL1 & UCTXSTP)
    {
         counter++;
         __delay_cycles(5);
         if(counter == 100)
         goto RET_BMP_R;
     }
    return(data);
RET_BMP_R: return(0);

}

//MAIN FUNCTION

void main (void)
{
    WDTCTL = WDTPW + WDTHOLD;

    #ifdef UART_PRINTF
    // initialize USCI module
    P5SEL |= BIT6 + BIT7;                     // P5.6,7 = USCI_A1 TXD/RXD

    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL__ACLK;                 // AMCLK
    UCA1BR0 = 3;                             // 32,768kHz 1200 (see User's Guide)
    UCA1BR1 = 0;                              // 32,768kHz 1200
    UCA1MCTL = UCBRS_3;                     // 32,768kHz 1200
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    //now its 9600 baud, for telemetry
    #endif
    //LED Blink config
    P1DIR |= 0x01;                            // P1.0 output

    //Timer stuff config
/*  TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
    TA1CCR0 = 0x0001;                          //match value
    TA1CTL = TASSEL_1 + MC_1 + TACLR ;         //    ACLK, upmode, clear TAR
    */

    //Configuring counter 1 o/p pins(i/p to MCU)
    P3SEL &= 0x37; //GPIO 00110111 LSB 3.3,3.6,3.7
    P3REN &= 0x37;
    P7SEL &= 0x07; //GPIO 00000111 MSB
    P7DIR &= 0x07; //Input pins
    P7REN &= 0x07;


    //Configuring counter 2 o/p pins(i/p to MCU)
    P10SEL &= 0x07; //GPIO 00000111 MSB
    P10DIR &= 0x07; //Input pins
    P10REN &= 0x07;
    P1SEL &= 0x1F; //GPIO  00011111 LSB
    P1DIR &= 0x1F; //Input pins
    P1REN &= 0x1F;

    //MUX1 and MUX2 select pins //P4.0(A1) P4.1(B1) P4.2(C1)//P4.5(A2) P4.6(B2) P4.7(C2)
    P4SEL &= 0x18; //GPIO bit0 - A1 00011000
    P4DIR |= 0xE7; //Output pins
    P4OUT |= 0xE7;
    //Reset counter1 and 2
    P4OUT = 0xA5; //HOLD FOR 20ns
    __delay_cycles(0.02);

    //GPS Initialization
      set_GPS();


    //Memory Initialization
    Init_mem();
    mem_id = Read_ID();
    printf("@");
    printf("%d,",mem_id);
    //success = Chip_erase();

    //BMP180 Initialization
    i2c_init();
    bmp_id = read_i2c(0xD0);
    printf("%d,",bmp_id);

    //IMU Initialization
    imu_init();
    //Reading IMU ID

    imu_id = read_imu_id(); //115 in hex is 0x73
    printf("%d,",imu_id);
    success_imu = write_imu_i2c(0x1C,0x08); //ACCEL setting +-4g
    success_imu = write_imu_i2c(0x23,0x78); //Accel and Gyro FIFO enble


    //BMP and IMU Calibration registers
    //Temperature and Pressure

    AC1a = read_i2c(0xAA);
    AC1b = read_i2c(0xAB);

    AC1 = AC1a * 256 + AC1b;

    AC2a = read_i2c(0xAC);
    AC2b = read_i2c(0xAD);

    AC2 = AC2a * 256 + AC2b;

    AC3a = read_i2c(0xAE);
    AC3b = read_i2c(0xAF);
    AC3 = AC3a * 256 + AC3b;

    AC4a = read_i2c(0xB0);
    AC4b = read_i2c(0xB1);
    AC4 = AC4a * 256 + AC4b;

    AC5a = read_i2c(0xB2);
    AC5b = read_i2c(0xB3);
    AC5 = AC5a * 256 + AC5b;

    AC6a = read_i2c(0xB4);
    AC6b = read_i2c(0xB5);
    AC6 = AC6a * 256 + AC6b;

    B1a = read_i2c(0xB6);
    B1b = read_i2c(0xB7);
    B1 = B1a * 256 + B1b;

    B2a = read_i2c(0xB8);
    B2b = read_i2c(0xB9);
    B2 = B2a * 256 + B2b;

    MBa = read_i2c(0xBA);
    MBb = read_i2c(0xBB);
    MB = MBa * 256 + MBb;

    MCa = read_i2c(0xBC);
    MCb = read_i2c(0xBD);
    MC = MCa * 256 + MCb;


    MDa = read_i2c(0xBE);
    MDb = read_i2c(0xBF);
    MD = MDa * 256 + MDb;


    //printf("BMP calibration registers:");
    printf("%d,",AC1);
    printf("%d,",AC2);
    printf("%d,",AC3);
    printf("%ud,",AC4);
    printf("%ud,",AC5);
    printf("%ud,",AC6);
    printf("%d,",B1);
    printf("%d,",B2);
    printf("%d,",MB);
    printf("%d,",MC);
    printf("%d,",MD);



    //accel and gyro offset registers read and transmit

    accel_off_hx = read_imu_i2c(0x77);
    accel_off_lx = read_imu_i2c(0x78);
    accel_off_lx = accel_off_lx / 2;
    accel_off_hy = read_imu_i2c(0x7A);
    accel_off_ly = read_imu_i2c(0x7B);
    accel_off_ly = accel_off_ly / 2;
    accel_off_hz = read_imu_i2c(0x7D);
    accel_off_lz = read_imu_i2c(0x7E);
    accel_off_lz = accel_off_lz / 2;

    accel_off_x = accel_off_hx * 256 + accel_off_lx;
    accel_off_y = accel_off_hy * 256 + accel_off_ly;
    accel_off_z = accel_off_hz * 256 + accel_off_lz;


    printf("%d,",accel_off_x);
    printf("%d,",accel_off_y);
    printf("%d,",accel_off_z);


    gyro_off_hx = read_imu_i2c(0x13);
    gyro_off_lx = read_imu_i2c(0x14);
    gyro_off_hy = read_imu_i2c(0x15);
    gyro_off_ly = read_imu_i2c(0x16);
    gyro_off_hz = read_imu_i2c(0x17);
    gyro_off_lz = read_imu_i2c(0x18);

    printf("%d,",gyro_off_x);
    printf("%d,",gyro_off_y);
    printf("%d,",gyro_off_z);


    //PMT monitoring pins initialization
    //4.3,5.4,8.6
    //Input pins
    P4SEL &= 0xF7; //GPIO 1111 0111
    P4DIR &= 0xF7; //GPIO 1111 0111
    P4REN &= 0xF7; // disable pull up or pull down

    P5SEL &= 0xEF; //GPIO 1110 1111
    P5DIR &= 0xEF;
    P5REN &= 0xEF;

    P8SEL &= 0xBF; //GPIO 1011 1111
    P8DIR &= 0xBF;
    P8DIR &= 0xBF;

    //Memory readout config, pin 5.0
    //Input pin
    P5SEL &= 0xFE; //GPIO 1111 1110
    P5DIR &= 0xFE;
    P5REN &= 0xFE;

    while(1)
    {
        P1OUT ^= 0x01;                            // Toggle P1.0

        //Telecommand //Counter reset : P8.5 input
        //Check if '1': reset counter ICs
        P8SEL &= 0xDF; //GPIO 1101 1111
        P8DIR &= 0xDF; //1101 1111
        comm_read = P8IN;
        comm_read = comm_read & 0x20; //0010 0000
        if(comm_read)
        {
            //reset counter
            //COUNTER CLEAR
            P4OUT = 0xA5;//10100101
            __delay_cycles(1);
        }


        //Give pulse for telemetry P10.0 Output pin
        P10SEL &= 0xFE; //GPIO 1111 1110
        P10DIR |= 0x01; //0000 0001
        P10OUT |= 0x01;
        //give delay
        __delay_cycles(10); //10us = 0.01 ms
        //make pin 0
        P10OUT &= 0xFE ; //1111 1110

        //PMT monitoring
        pmt1_stat = P4IN;
        pmt2_stat = P5IN;
        pmt3_stat = P8IN;
        pmt1_stat = pmt1_stat & 0x08; //0000 1000
        pmt1_stat = pmt1_stat/8.0;
        pmt2_stat = pmt2_stat & 0x10; //0001 0000
        pmt2_stat = pmt2_stat/8.0;
        pmt3_stat = pmt3_stat & 0x40; //0100 0000
        pmt3_stat = pmt3_stat/16.0;
        pmt_stat = pmt1_stat + pmt2_stat + pmt3_stat;
        pmt_stat = pmt_stat + 0x30; //0011 0000


        //read GPS
        int i = 0;
        counter = 0;

        printf("$");
        printf("G");
        printf("P");
        printf("G");
        printf("G");
        printf("A ");

        printf("%d,",pmt_stat);

        while(!(UCA2IFG&UCRXIFG))
        {
           counter++;
           __delay_cycles(20);
           if(counter == 100000) //wait for atleast 2s = 2000ms
               goto IMU;
        }                                   // Byte is received

        int dummy;
        int gga_length = 0;
        while(UCA2IFG&UCRXIFG)              // Byte is received,start from $
        {
           if(UCA2RXBUF == '$')
           {
               //read further
               for(i=0;i<82;i++)
               {
                   gps_buf[i] = UCA2RXBUF;
                   if(gps_buf[i] == 0x0A)
                      {
                       gga_length = i;
                       break;
                      }
                   while(!(UCA2IFG&UCRXIFG)); // Byte is received

               }
               break;
           }
           else
          {
              //read RX buffer
              dummy = UCA2RXBUF;
          }

        }

        //Printing in one line all gps data
        for(i=7;i<gga_length-2;i++)
        {
          printf("%c",gps_buf[i]);
        }
        printf(",");



IMU:
        //read accel and gyro data

         accel_hx = read_imu_i2c(0x3B);
         accel_lx = read_imu_i2c(0x3C);
         accel_hy = read_imu_i2c(0x3D);
         accel_ly = read_imu_i2c(0x3E);
         accel_hz = read_imu_i2c(0x3F);
         accel_lz = read_imu_i2c(0x40);


         accel_x = accel_hx*256 + accel_lx;
         accel_y = accel_hy*256 + accel_ly;
         accel_z = accel_hz*256 + accel_lz;
         accel_x = accel_x * (4.0/32768.0);
         accel_y = accel_y * (4.0/32768.0);
         accel_z = accel_z * (4.0/32768.0);

         printf("%d,",accel_x);
         printf("%d,",accel_y);
         printf("%d,",accel_z);

         gyro_hx = read_imu_i2c(0x43);
         gyro_lx = read_imu_i2c(0x44);
         gyro_hy = read_imu_i2c(0x45);
         gyro_ly = read_imu_i2c(0x46);
         gyro_hz = read_imu_i2c(0x47);
         gyro_lz = read_imu_i2c(0x48);

         gyro_x = gyro_hx*256 + gyro_lx;
         gyro_y = gyro_hy*256 + gyro_ly;
         gyro_z = gyro_hz*256 + gyro_lz;

         gyro_x = gyro_x * (250.0/32768.0);
         gyro_y = gyro_y * (250.0/32768.0);
         gyro_z = gyro_z * (250.0/32768.0);

         //Gyro
         printf("%d,",gyro_x);
         printf("%d,",gyro_y);
         printf("%d,",gyro_z);

         //Reading BMP
        //Temperature data
         success_bmp = write_i2c(0xF4, 0x2E);
         //delay 4.5 ms
         __delay_cycles(32768);
         temp_lower = read_i2c(0xF7);
         temp_upper = read_i2c(0xF6);

         //pressure uncompensated

         success_bmp = write_i2c(0xF4, 0x34);
         __delay_cycles(32768);
         pres_lower = read_i2c(0xF7);
         pres_upper = read_i2c(0xF6);

         pres_third = read_i2c(0xF8);

         temp_final = temp_upper * 256 + temp_lower;
         pres_final = pres_upper * 256 + pres_lower;

         //TRUE TEMPERATURE
         long X1 = 0.0, X2 = 0.0, B5 = 0.0;
         long temperature = 0.0;
         long pressure = 0.0;
         X1 = (temp_final - AC6)*AC5/32768.0;
         X2 = MC * 2048.0/(X1 + MD);
         B5 = X1 + X2;
         temperature = (B5 + 8.0)/160.0;

         //TRUE PRESSURE
         long B6 = 40.0, X3 = 0.0, B3 = 0.0;
         //int oss = 0;

         long B4 = 0.0,B7 = 0.0;
         //formulae
         B6 = B5 - 4000.0;
         X1 = (B2 * (B6 * B6/4096.0))/2048.0;
         X2 = AC2 * B6/ 2048.0;
         X3 = X1 + X2;
         B3 = ((AC1 * 4.0 + X3) +2)/4.0;
         X1 = AC3 * B6 / 8192.0;
         X2 = (B1 * (B6 * B6/4096.0))/65536.0;
         X3 = ((X1 + X2) + 2.0)/4.0;
         B4 = AC4 * (unsigned long)(X3 + 32768.0)/32768.0;
         B7 = ((unsigned long)pres_final - B3) * (50000.0);

         if (B7 < 0x80000000) {pressure = (B7 * 2.0)/B4;}
         else { pressure = (B7/B4) * 2.0;}

         X1 = (pressure/256.0)*(pressure/256.0);
         X1 = (X1 * 3038.0)/65536.0;
         X2 = (-7357.0 * pressure) / 65536.0;
         pressure = pressure + (X1+X2+3791.0) / 16.0;


         printf("%lu,",pressure);
         printf("%d,",temperature);

         //Reading counters

        unsigned char count1,count2,count3,count4,countPMT1,countPMT2,countPMT3,countC;
        unsigned char temp1,temp2,temp3,temp4;
        unsigned char temp5,temp6;

        __delay_cycles(1);
        //COUNTER DATA
        P4OUT = 0x84;//rclk 10000100
        __delay_cycles(1);//50ns

        //COUNTER CLEAR
        P4OUT = 0xA5;//10100101
        __delay_cycles(1);

        P4OUT = 0x00; //A lower
        __delay_cycles(5);

        count1 = P3IN;
        count2 = P10IN;
        temp5 = P7IN;
        temp6 = P1IN;

        temp1 = count1 & 0xC0; //11000000
        temp3 = temp1/32.0; //2^5
        temp2 = count1 & 0x08; //00001000
        temp4 = temp2/8.0; //2^3
        //count1 = P7IN;
        count1 = temp5 & 0xF8;//11111000
        count1 = count1+temp3+temp4;


        temp1 = count2 & 0xF8; //11111000
        //temp2 = P1IN;
        temp2 = temp6 & 0xE0; //11100000
        temp2 = temp2/32.0; //2^5
        count2 = temp1+temp2;

        P4OUT = 0x21;//A upper 00100001
        __delay_cycles(5);
        count3 = P3IN;
        count4 = P10IN;
        temp5 = P7IN;
        temp6 = P1IN;

        temp1 = count3 & 0xC0; //11000000
        temp3 = temp1/32.0; //2^5
        temp2 = count3 & 0x08; //00001000
        temp4 = temp2/8.0; //2^3

        count3 = temp5 & 0xF8;//11111000
        count3 = count3+temp3+temp4;

        temp1 = count4 & 0xF8; //11111000

        temp2 = temp6 & 0xE0; //11100000
        temp2 = temp2/32.0; //2^5
        count4 = temp1+temp2;

        countPMT1 = count3*256 + count1;
        countPMT3 = count4*256 + count2;

        P4OUT = 0x42;//B lower 01000010
        __delay_cycles(5);
        count1 = P3IN;
        count2 = P10IN;

        temp1 = count1 & 0xC0; //11000000
        temp3 = temp1/32; //2^5
        temp2 = count1 & 0x08; //00001000
        temp4 = temp2/8; //2^3
        count1 = P7IN;
        count1 = count1 & 0xF8;//11111000
        count1 = count1+temp3+temp4;

        temp1 = count2 & 0xF8; //11111000
        temp2 = P1IN;
        temp2 = temp2 & 0xE0; //11100000
        temp2 = temp2/32; //2^5
        count2 = temp1+temp2;

        P4OUT = 0x63;//B upper 01100011
        __delay_cycles(5);
        count3 = P3IN;
        count4 = P10IN;

        temp1 = count3 & 0xC0; //11000000
        temp3 = temp1/32; //2^5
        temp2 = count3 & 0x08; //00001000
        temp4 = temp2/8; //2^3

        count3 = P7IN;

        count3 = count3 & 0xF8;//11111000
        count3 = count3+temp3+temp4;


        temp1 = count4 & 0xF8; //11111000
        temp2 = P1IN;
        temp2 = temp2 & 0xE0; //11100000
        count4 = temp1+temp2;

        countPMT2 = count3*256 + count1;
        countC = count4*256 + count2;

        printf("%d,",countPMT1);
        printf("%d,",countPMT2);
        printf("%d,",countPMT3);
        printf("%d",countC);
        printf("\r\n");

       // __delay_cycles(500000);


        //P5.0 for memory readout
        mem_read = P5IN;
        mem_read = mem_read & 0x01; //0000 0001
        if (mem_read)
        {
            //read out memory and do something
            //Think of that something soon
            //and code here.
            //Thanks


        }

    }


}

#ifdef UART_PRINTF
int fputc(int _c, register FILE *_fp)
{
    while(!(UCA1IFG&UCTXIFG));
    UCA1TXBUF = (unsigned char) _c;

    return((unsigned char)_c);
}

int fputs(const char *_ptr, register FILE *_fp)
{
    unsigned int i, len;

    len = strlen(_ptr);

    for(i=0 ; i<len ; i++)
    {
         while(!(UCA1IFG&UCTXIFG));
         UCA1TXBUF = (unsigned char) _ptr[i];
    }

    return len;
}
#endif
