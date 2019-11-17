
#include "pn532.h"
#include "Registrs.h"


uint8_t Reg_610X[0x10];
uint8_t Reg_620X[0x07];
uint8_t Reg_63XX[0x40];

uint16_t Adr[128];

void ReadRegs(SPI_HandleTypeDef *hspi){
//  uint16_t Adr;
  uint16_t i;
  
  Reg_610X[0] = 0;
  Reg_610X[1] = 0;
  Reg_610X[2] = 0;
  for(i = 0; i < 0x0F; i++){
    Adr[i] = 0x6100 + i;
  }
  readRegisters(hspi, Adr, Reg_610X, 0x10);
  
  
  
   for(i = 0; i < 0x06; i++){
     Adr[ i ] = 0x6200 + i;
  }
  readRegisters(hspi, Adr, Reg_620X, 0x07);
  
   for(i = 0; i < 0x3F; i++){
     Adr[ i ] = 0x6300 + i;
  }
  readRegisters(hspi, Adr, Reg_63XX, 0x20);
  readRegisters(hspi, (Adr + 0x20), (Reg_63XX + 0x20), 0x20);  
  
} 
