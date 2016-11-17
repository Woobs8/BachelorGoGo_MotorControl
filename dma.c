
/*******************************************************************************
  DMA Generated Driver File

Company:
Microchip Technology Inc.

File Name:
    dma.c

Summary:
This is the generated driver implementation file for the DMA driver using MPLAB(c) Code Configurator

Description:
This source file provides implementations for driver APIs for DMA.
Generation Information :
Product Revision  :  MPLAB(c) Code Configurator - 3.16
Device            :  dsPIC33EP128MC504
Version           :  1.0
The generated drivers are tested against the following:
Compiler          :  XC16 1.26
MPLAB             :  MPLAB X 3.20
*******************************************************************************/

#include "dma.h"

tADC_DMA_BUF BufferAdc;

void DMA_Initialize(void) 
{ 
    // Initialize channels which are enabled 
    // AMODE Register Indirect with Post-Increment mode; CHEN enabled; DIR Reads from peripheral address, writes to RAM address; HALF Initiates interrupt when all of the data has been moved; SIZE 16 bit; NULLW disabled; MODE Continuous, Ping-Pong modes are disabled; 
    DMA0CON= 0x8000 & 0xFFFE; //Enable DMA Channel later;
    // FORCE disabled; IRQSEL ADC1; 
    DMA0REQ= 13;
    // CNT 4; 
    DMA0CNT= 3;
    // STA 4096; 
    DMA0PAD = ((volatile unsigned int) &ADC1BUF0); // Point DMA to ADC1BUF0
    DMA0STAL= (volatile unsigned int) &BufferAdc;
    // STA 0; 
    DMA0STAH= 0x0;
    
    // Clearing Channel 0 Interrupt Flag;
    IFS0bits.DMA0IF = 0;
    // Enabling Channel 0 Interrupt
    IEC0bits.DMA0IE = 1;

    //Enable DMA Channel 0
    DMA0CONbits.CHEN = 1;
}



/**
  End of File
*/
