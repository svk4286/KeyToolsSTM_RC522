/*
 * RC522.c
 *
 *  Created on: 23 ‡‚„. 2019 „.
 *      Author: Sergey
 */

#include <string.h>
#include "RC522.h"

#define true 1
#define false 0

#define CS2_LOW		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
#define CS2_HIGHT		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

//extern SPI_HandleTypeDef hspi2;
Uid uid;

// Firmware data for self-test
// Reference values based on firmware version
// Hint: if needed, you can remove unused self-test data to save flash memory
//
// Version 0.0 (0x90)
// Philips Semiconductors; Preliminary Specification Revision 2.0 - 01 August 2005; 16.1 self-test
const uint8_t MFRC522_firmware_referenceV0_0[] = {
	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
// Version 1.0 (0x91)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV1_0[]  = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
// Version 2.0 (0x92)
// NXP Semiconductors; Rev. 3.8 - 17 September 2014; 16.1.1 self-test
const uint8_t MFRC522_firmware_referenceV2_0[]  = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};
// Clone
// Fudan Semiconductor FM17522 (0x88)
const uint8_t FM17522_firmware_reference[]  = {
	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
};


/////////////////////////////////////////////////////////////////////////////////////
// Basic interface functions for communicating with the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegister(	SPI_HandleTypeDef *  hspi,
						PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
						uint8_t value,			///< The value to write.
						uint16_t timeout) {
	uint8_t d[2];
	d[0] = reg;
	d[1] = value;

	CS2_LOW;		// Select slave
	HAL_SPI_Transmit( hspi, d, 2, timeout  );//SPI.transfer(reg);						// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	CS2_HIGHT;		// Release slave again

} // End PCD_WriteRegister()


/*** Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegisters(SPI_HandleTypeDef *  hspi,
						PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
						uint8_t count,			///< The number of bytes to write to the register
						uint8_t *values,		///< The values to write. Byte array.
						uint16_t timeout) {
	uint8_t r = reg;
	CS2_LOW;		// Select slave
	HAL_SPI_Transmit( hspi, &r, 1, 1000  );// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	HAL_SPI_Transmit( hspi, values, count, timeout  );
	CS2_HIGHT;
} // End PCD_WriteRegister()


/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t PCD_ReadRegister(SPI_HandleTypeDef *  hspi,
							PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
							uint16_t timeout) {
	uint8_t value[2];
	uint8_t r[2] = {0x80 | reg, 0x00};;
	CS2_LOW;		// Select slave
	HAL_SPI_TransmitReceive(hspi, r, value, 2, timeout);
	CS2_HIGHT;
	return value[1];
} // End PCD_ReadRegister()


/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadRegisters(
								SPI_HandleTypeDef *  hspi,
								PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
								uint8_t count,			///< The number of bytes to read
								uint8_t *values,		///< Byte array to store the values in.
								uint8_t rxAlign,		///< Only bit positions rxAlign..7 in values[0] are updated.
								uint16_t timeout) {
	if (count == 0) {
		return;
	}
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));
	uint8_t address = 0x80 | reg;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
	uint8_t value;
	CS2_LOW; 									// Select slave
	count--;								// One read is performed outside of the loop
	HAL_SPI_Transmit( hspi, &address, 1, timeout  );					// Tell MFRC522 which address we want to read
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		uint8_t mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		HAL_SPI_TransmitReceive(hspi, &address, &value, 1, timeout);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		HAL_SPI_TransmitReceive(hspi, &address, (values + index), 1, timeout);
		index++;
	}
	address = 0;
	HAL_SPI_TransmitReceive(hspi, &address, (values + index), 1, timeout); // Read the final byte. Send 0 to stop reading.
	CS2_HIGHT;		// Release slave again
} // End PCD_ReadRegister()


/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(SPI_HandleTypeDef *  hspi,
										PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask,			///< The bits to set.
										uint16_t timeout) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(hspi, reg, timeout);
	PCD_WriteRegister(hspi, reg, tmp | mask, timeout);			// set bit mask
} // End PCD_SetRegisterBitMask()


/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(SPI_HandleTypeDef *  hspi,
								PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
										uint8_t mask,			///< The bits to clear.
										uint16_t timeout	  ) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(hspi, reg, timeout);
	PCD_WriteRegister(hspi, reg, tmp & (~mask), timeout);		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CalculateCRC(SPI_HandleTypeDef *  hspi,
							uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
							uint8_t length,	///< In: The number of bytes to transfer.
							uint8_t *result,	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
							uint16_t timeout ) {
	PCD_WriteRegister(hspi,CommandReg, PCD_Idle, timeout);		// Stop any active command.
	PCD_WriteRegister(hspi,DivIrqReg, 0x04, timeout);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegister(hspi,FIFOLevelReg, 0x80, timeout);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisters(hspi,FIFODataReg, length, data, timeout);	// Write data to the FIFO
	PCD_WriteRegister(hspi,CommandReg, PCD_CalcCRC, timeout);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73Œºs.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (uint16_t i = 10000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		uint8_t n = PCD_ReadRegister(hspi,DivIrqReg, timeout);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(hspi,CommandReg, PCD_Idle, timeout);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = PCD_ReadRegister(hspi,CRCResultRegL, timeout);
			result[1] = PCD_ReadRegister(hspi,CRCResultRegH, timeout);
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	uint8_t hardReset = false;
	  GPIO_InitTypeDef GPIO_InitStruct;


	// If a valid pin number has been set, pull device out of power down / reset state.
		// First set the resetPowerDownPin as digital input, to check the MFRC522 power down mode.
		 /*Configure GPIO pin : PB12 */
		  GPIO_InitStruct.Pin = GPIO_PIN_12;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {	// The MFRC522 chip is in power down mode.
			  /*Configure GPIO pin : PB12 */
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);		// Now set the resetPowerDownPin as digital output.

			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);		// Make shure we have a clean LOW state.
			  HAL_Delay(1);				// 8.8.1 Reset timing requirements says about 100ns. Let us be generous: 2Œºsl
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		// Exit power down mode. This triggers a hard reset.
			// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74Œºs. Let us be generous: 50ms.
			 HAL_Delay(50);
			 hardReset = true;
		}

	if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		PCD_Reset(hspi, timeout);
	}

	// Reset baud rates
	PCD_WriteRegister(hspi, TxModeReg, 0x00, timeout);
	PCD_WriteRegister(hspi, RxModeReg, 0x00, timeout);
	// Reset ModWidthReg
	PCD_WriteRegister(hspi, ModWidthReg, 0x26, timeout);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PCD_WriteRegister(hspi, TModeReg, 0x80, timeout);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(hspi, TPrescalerReg, 0xA9, timeout);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25Œºs.
	PCD_WriteRegister(hspi, TReloadRegH, 0x03, timeout);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(hspi, TReloadRegL, 0xE8, timeout);

	PCD_WriteRegister(hspi, TxASKReg, 0x40, timeout);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(hspi, ModeReg, 0x3D, timeout);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn(hspi, timeout);						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()


/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	PCD_WriteRegister(hspi, CommandReg, PCD_SoftReset, timeout);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74Œºs. Let us be generous: 50ms.
	uint8_t count = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		HAL_Delay(50);
	} while ((PCD_ReadRegister(hspi,CommandReg, timeout) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()


/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	uint8_t value = PCD_ReadRegister(hspi, TxControlReg, timeout);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(hspi, TxControlReg, value | 0x03, timeout);
	}
} // End PCD_AntennaOn()


/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	PCD_ClearRegisterBitMask(hspi, TxControlReg, 0x03, timeout);
} // End PCD_AntennaOff()


/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t PCD_GetAntennaGain(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	return PCD_ReadRegister(hspi, RFCfgReg, timeout) & (0x07<<4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void PCD_SetAntennaGain(SPI_HandleTypeDef *  hspi, uint8_t  mask, uint16_t timeout) {
	if (PCD_GetAntennaGain(hspi, timeout) != mask) {						// only bother if there is a change
		PCD_ClearRegisterBitMask(hspi, RFCfgReg, (0x07<<4), timeout);		// clear needed to allow 000 pattern
		PCD_SetRegisterBitMask(hspi, RFCfgReg, mask & (0x07<<4), timeout);	// only set RxGain[2:0] bits
	}
} // End PCD_SetAntennaGain()

/**
 * Performs a self-test of the MFRC522
 * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 *
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */
uint8_t PCD_PerformSelfTest(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	// This follows directly the steps outlined in 16.1.1
	// 1. Perform a soft reset.
	PCD_Reset(hspi, timeout);

	// 2. Clear the internal buffer by writing 25 bytes of 00h
	uint8_t ZEROES[25] = {0x00};
	PCD_WriteRegister(hspi, FIFOLevelReg, 0x80, timeout);		// flush the FIFO buffer
	PCD_WriteRegisters(hspi, FIFODataReg, 25, ZEROES, timeout);	// write 25 bytes of 00h to FIFO
	PCD_WriteRegister(hspi, CommandReg, PCD_Mem, timeout);		// transfer to internal buffer

	// 3. Enable self-test
	PCD_WriteRegister(hspi, AutoTestReg, 0x09, timeout);

	// 4. Write 00h to FIFO buffer
	PCD_WriteRegister(hspi, FIFODataReg, 0x00, timeout);

	// 5. Start self-test by issuing the CalcCRC command
	PCD_WriteRegister(hspi, CommandReg, PCD_CalcCRC, timeout);

	// 6. Wait for self-test to complete
	uint8_t n;
	for (uint8_t i = 0; i < 0xFF; i++) {
		// The datasheet does not specify exact completion condition except
		// that FIFO buffer should contain 64 bytes.
		// While selftest is initiated by CalcCRC command
		// it behaves differently from normal CRC computation,
		// so one can't reliably use DivIrqReg to check for completion.
		// It is reported that some devices does not trigger CRCIRq flag
		// during selftest.
		n = PCD_ReadRegister(hspi, FIFOLevelReg, timeout);
		if (n >= 64) {
			break;
		}
	}
	PCD_WriteRegister(hspi, CommandReg, PCD_Idle, timeout);		// Stop calculating CRC for new content in the FIFO.

	// 7. Read out resulting 64 bytes from the FIFO buffer.
	uint8_t result[64];
	PCD_ReadRegisters(hspi, FIFODataReg, 64, result, 0, timeout);

	// Auto self-test done
	// Reset AutoTestReg register to be 0 again. Required for normal operation.
	PCD_WriteRegister(hspi, AutoTestReg, 0x00, timeout);

	// Determine firmware version (see section 9.3.4.8 in spec)
	uint8_t version = PCD_ReadRegister(hspi, VersionReg, timeout);

	// Pick the appropriate reference values
	const uint8_t *reference;
	switch (version) {
		case 0x88:	// Fudan Semiconductor FM17522 clone
			reference = FM17522_firmware_reference;
			break;
		case 0x90:	// Version 0.0
			reference = MFRC522_firmware_referenceV0_0;
			break;
		case 0x91:	// Version 1.0
			reference = MFRC522_firmware_referenceV1_0;
			break;
		case 0x92:	// Version 2.0
			reference = MFRC522_firmware_referenceV2_0;
			break;
		default:	// Unknown version
			return false; // abort test
	}

	// Verify that the results match up to our expectations
	for (uint8_t i = 0; i < 64; i++) {
		if (result[i] != reference[i]) {
			return false;
		}
	}

	// Test passed; all is good.
	return true;
} // End PCD_PerformSelfTest()


/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

//IMPORTANT NOTE!!!!
//Calling any other function that uses CommandReg will disable soft power down mode !!!
//For more details about power control, refer to the datasheet - page 33 (8.6)

void PCD_SoftPowerDown(SPI_HandleTypeDef *  hspi,uint16_t timeout){//Note : Only soft power down mode is available throught software
	uint8_t val = PCD_ReadRegister(hspi, CommandReg, timeout); // Read state of the command register
	val |= (1<<4);// set PowerDown bit ( bit 4 ) to 1
	PCD_WriteRegister(hspi,CommandReg, val, timeout);//write new value to the command register
}

void PCD_SoftPowerUp(SPI_HandleTypeDef *  hspi,uint16_t timeout){
	uint8_t val = PCD_ReadRegister(hspi, CommandReg, timeout); // Read state of the command register
	val &= ~(1<<4);// set PowerDown bit ( bit 4 ) to 0
	PCD_WriteRegister(hspi, CommandReg, val, timeout);//write new value to the command register
	// wait until PowerDown bit is cleared (this indicates end of wake up procedure)
	const uint32_t pwr_timeout = HAL_GetTick() + 500;// create timer for timeout (just in case)

	while(HAL_GetTick() <= pwr_timeout){ // set timeout to 500 ms
		val = PCD_ReadRegister(hspi, CommandReg, timeout);// Read state of the command register
		if(!(val & (1<<4))){ // if powerdown bit is 0
			break;// wake up procedure is finished
		}
	}
}


/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_TransceiveData(SPI_HandleTypeDef *  hspi,
								uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
								uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
								uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
								uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
								uint8_t *validBits,	///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default nullptr.
								uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
								uint8_t checkCRC,		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
								uint16_t timeout) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(hspi, PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC,timeout);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CommunicateWithPICC(	SPI_HandleTypeDef *  hspi,
									uint8_t command,		///< The command to execute. One of the PCD_Command enums.
									uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
									uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
									uint8_t sendLen,		///< Number of bytes to transfer to the FIFO.
									uint8_t *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
									uint8_t *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
									uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									uint8_t checkCRC,		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									uint16_t timeout) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteRegister(hspi,CommandReg, PCD_Idle, timeout);			// Stop any active command.
	PCD_WriteRegister(hspi,ComIrqReg, 0x7F, timeout);					// Clear all seven interrupt request bits
	PCD_WriteRegister(hspi,FIFOLevelReg, 0x80, timeout);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisters(hspi,FIFODataReg, sendLen, sendData, timeout);	// Write sendData to the FIFO
	PCD_WriteRegister(hspi,BitFramingReg, bitFraming, timeout);		// Bit adjustments
	PCD_WriteRegister(hspi,CommandReg, command, timeout);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(hspi,BitFramingReg, 0x80, timeout);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86Œºs.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	uint16_t i;
	for (i = 2000; i > 0; i--) {
		uint8_t n = PCD_ReadRegister(hspi,ComIrqReg, timeout);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		return STATUS_TIMEOUT;
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(hspi, ErrorReg, timeout); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	uint8_t _validBits = 0;

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n = PCD_ReadRegister(hspi, FIFOLevelReg, timeout);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegisters(hspi, FIFODataReg, n, backData, rxAlign, timeout);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(hspi, ControlReg, timeout) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		StatusCode status = PCD_CalculateCRC(hspi, &backData[0], *backLen - 2, &controlBuffer[0], timeout);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_RequestA(SPI_HandleTypeDef *  hspi,
						uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
						uint8_t *bufferSize,	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
						uint16_t timeout) {
	return PICC_REQA_or_WUPA(hspi, PICC_CMD_REQA, bufferATQA, bufferSize, timeout);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_WakeupA(SPI_HandleTypeDef *  hspi,
						uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
						uint8_t *bufferSize,	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
						uint16_t timeout) {
	return PICC_REQA_or_WUPA(hspi, PICC_CMD_WUPA, bufferATQA, bufferSize, timeout);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_REQA_or_WUPA(SPI_HandleTypeDef *  hspi,
							uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
							uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize,	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
							uint16_t timeout) {
	uint8_t validBits;
	StatusCode status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 uint8_ts long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(hspi, CollReg, 0x80, timeout);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(hspi, &command, 1, bufferATQA, bufferSize, &validBits, 0, false, timeout);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_Select(	SPI_HandleTypeDef *  hspi,
						Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
						uint8_t validBits,		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
						uint16_t timeout) {
	uint8_t uidComplete;
	uint8_t selectDone;
	uint8_t useCascadeTag;
	uint8_t cascadeLevel = 1;
	StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	uint8_t *responseBuffer;
	uint8_t responseLength;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	PCD_ClearRegisterBitMask(hspi, CollReg, 0x80, timeout);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(hspi, buffer, 7, &buffer[7], timeout);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(hspi, BitFramingReg, (rxAlign << 4) + txLastBits, timeout);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(hspi, buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false, timeout);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = PCD_ReadRegister(hspi, CollReg, timeout); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(hspi, responseBuffer, 1, &buffer[2], timeout);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_HaltA(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	StatusCode result;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(hspi, buffer, 2, &buffer[2], timeout);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(hspi, buffer, sizeof(buffer), NULL, 0, NULL, 0, false, timeout);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
StatusCode PCD_Authenticate(SPI_HandleTypeDef *  hspi,
							uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
							uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
							MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
							Uid *uid,			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
							uint16_t timeout) {
	uint8_t waitIRq = 0x10;		// IdleIRq

	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyByte[i];
	}
	// Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid.
	for (uint8_t i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}

	// Start the authentication.
	return PCD_CommunicateWithPICC(hspi, PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData), NULL,  NULL,  NULL, 0, false, timeout);
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void PCD_StopCrypto1(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(hspi, Status2Reg, 0x08, timeout); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Read(SPI_HandleTypeDef *  hspi,
						uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
						uint8_t *buffer,		///< The buffer to store the data in
						uint8_t *bufferSize,	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
						uint16_t timeout) {
	StatusCode result;

	// Sanity check
	if (buffer == NULL || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(hspi, buffer, 2, &buffer[2], timeout);
	if (result != STATUS_OK) {
		return result;
	}

	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(hspi, buffer, 4, buffer, bufferSize, NULL, 0, true, timeout);
} // End MIFARE_Read()

/**
 * Writes 16 bytes to the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 bytes are transferred to the Ultralight PICC, only the least significant 4 bytes (bytes 0 to 3)
 * are written to the specified address. It is recommended to set the remaining bytes 04h to 0Fh to all logic 0.
 * *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Write(SPI_HandleTypeDef *  hspi,
						uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
						uint8_t *buffer,	///< The 16 bytes to write to the PICC
						uint8_t bufferSize,	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
						uint16_t timeout) {
	StatusCode result;

	// Sanity check
	if (buffer == NULL || bufferSize < 16) {
		return STATUS_INVALID;
	}

	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(hspi, cmdBuffer, 2, false, timeout); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(hspi, buffer, bufferSize, false, timeout); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK) {
		return result;
	}

	return STATUS_OK;
} // End MIFARE_Write()


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_MIFARE_Transceive(SPI_HandleTypeDef *  hspi,
								uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
								uint8_t sendLen,		///< Number of bytes in sendData.
								uint8_t acceptTimeout,	///< True => A timeout is also success
								uint16_t timeout) {
	StatusCode result;
	uint8_t cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

	// Sanity check
	if (sendData == NULL || sendLen > 16) {
		return STATUS_INVALID;
	}

	// Copy sendData[] to cmdBuffer[] and add CRC_A
	memcpy(cmdBuffer, sendData, sendLen);
	result = PCD_CalculateCRC(hspi,cmdBuffer, sendLen, &cmdBuffer[sendLen],timeout);
	if (result != STATUS_OK) {
		return result;
	}
	sendLen += 2;

	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	uint8_t cmdBufferSize = sizeof(cmdBuffer);
	uint8_t validBits = 0;
	result = PCD_CommunicateWithPICC(hspi, PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, false, timeout);
	if (acceptTimeout && result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result != STATUS_OK) {
		return result;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()

/**
 * Returns a __FlashStringHelper pointer to a status code name.
 *
 * @return const __FlashStringHelper *
 */
//String GetStatusCodeName(StatusCode code	///< One of the StatusCode enums.
//										) {
//	switch (code) {
//		case STATUS_OK:				return F("Success.");
//		case STATUS_ERROR:			return F("Error in communication.");
//		case STATUS_COLLISION:		return F("Collission detected.");
//		case STATUS_TIMEOUT:		return F("Timeout in communication.");
//		case STATUS_NO_ROOM:		return F("A buffer is not big enough.");
//		case STATUS_INTERNAL_ERROR:	return F("Internal error in the code. Should not happen.");
//		case STATUS_INVALID:		return F("Invalid argument.");
//		case STATUS_CRC_WRONG:		return F("The CRC_A does not match.");
//		case STATUS_MIFARE_NACK:	return F("A MIFARE PICC responded with NAK.");
//		default:					return F("Unknown error");
//	}
//} // End GetStatusCodeName()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
PICC_Type PICC_GetType(SPI_HandleTypeDef *  hspi,
						uint8_t sak		///< The SAK byte returned from PICC_Select().
										) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()


/**
 * Performs the "magic sequence" needed to get Chinese UID changeable
 * Mifare cards to allow writing to sector 0, where the card UID is stored.
 *
 * Note that you do not need to have selected the card through REQA or WUPA,
 * this sequence works immediately when the card is in the reader vicinity.
 * This means you can use this method even on "bricked" cards that your reader does
 * not recognise anymore (see MFRC522::MIFARE_UnbrickUidSector).
 *
 * Of course with non-bricked devices, you're free to select them before calling this function.
 */
uint8_t MIFARE_OpenUidBackdoor(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	// Magic sequence:
	// > 50 00 57 CD (HALT + CRC)
	// > 40 (7 bits only)
	// < A (4 bits only)
	// > 43
	// < A (4 bits only)
	// Then you can write to sector 0 without authenticating

	PICC_HaltA(hspi, timeout); // 50 00 57 CD

	uint8_t cmd = 0x40;
	uint8_t validBits = 7; /* Our command is only 7 bits. After receiving card response,
						  this will contain amount of valid response bits. */
	uint8_t response[32]; // Card's response is written here
	uint8_t received;
	StatusCode status = PCD_TransceiveData(hspi, &cmd, (uint8_t)1, response, &received, &validBits, (uint8_t)0, false, timeout); // 40
	if(status != STATUS_OK) {
		return false;
	}
	if (received != 1 || response[0] != 0x0A) {
		return false;
	}

	cmd = 0x43;
	validBits = 8;
	status = PCD_TransceiveData(hspi, &cmd, (uint8_t)1, response, &received, &validBits, (uint8_t)0, false, timeout); // 43
	if(status != STATUS_OK) {
		return false;
	}
	if (received != 1 || response[0] != 0x0A) {
		return false;
	}

	// You can now write to sector 0 without authenticating!
	return true;
} // End MIFARE_OpenUidBackdoor()



/**
 * Reads entire block 0, including all manufacturer data, and overwrites
 * that block with the new UID, a freshly calculated BCC, and the original
 * manufacturer data.
 *
 * It assumes a default KEY A of 0xFFFFFFFFFFFF.
 * Make sure to have selected the card before this function is called.
 */
uint8_t MIFARE_SetUid(SPI_HandleTypeDef *  hspi,uint8_t *newUid, uint8_t uidSize, uint8_t logErrors, uint16_t timeout) {

	// UID + BCC byte can not be larger than 16 together
	if (!newUid || !uidSize || uidSize > 15) {
		if (logErrors) {
//			Serial.println(F("New UID buffer empty, size 0, or size > 15 given"));
		}
		return false;
	}

	// Authenticate for reading
	MIFARE_Key key = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	StatusCode status = PCD_Authenticate(hspi,PICC_CMD_MF_AUTH_KEY_A, (uint8_t)1, &key, &uid, timeout);
	if (status != STATUS_OK) {

		if (status == STATUS_TIMEOUT) {
			// We get a read timeout if no card is selected yet, so let's select one

			// Wake the card up again if sleeping
//			  uint8_t atqa_answer[2];
//			  uint8_t atqa_size = 2;
//			  PICC_WakeupA(atqa_answer, &atqa_size);

			if (!PICC_IsNewCardPresent(hspi, timeout) || !PICC_ReadCardSerial(hspi, timeout)) {
//				Serial.println(F("No card was previously selected, and none are available. Failed to set UID."));
				return false;
			}

			status = PCD_Authenticate(hspi, PICC_CMD_MF_AUTH_KEY_A, (uint8_t)1, &key, &uid, timeout);
			if (status != STATUS_OK) {
				// We tried, time to give up
				if (logErrors) {
//					Serial.println(F("Failed to authenticate to card for reading, could not set UID: "));
//					Serial.println(GetStatusCodeName(status));
				}
				return false;
			}
		}
		else {
			if (logErrors) {
//				Serial.print(F("PCD_Authenticate() failed: "));
//				Serial.println(GetStatusCodeName(status));
			}
			return false;
		}
	}

	// Read block 0
	uint8_t block0_buffer[18];
	uint8_t byteCount = sizeof(block0_buffer);
	status = MIFARE_Read(hspi, (uint8_t)0, block0_buffer, &byteCount, timeout);
	if (status != STATUS_OK) {
		if (logErrors) {
//			Serial.print(F("MIFARE_Read() failed: "));
//			Serial.println(GetStatusCodeName(status));
//			Serial.println(F("Are you sure your KEY A for sector 0 is 0xFFFFFFFFFFFF?"));
		}
		return false;
	}

	// Write new UID to the data we just read, and calculate BCC byte
	uint8_t bcc = 0;
	for (uint8_t i = 0; i < uidSize; i++) {
		block0_buffer[i] = newUid[i];
		bcc ^= newUid[i];
	}

	// Write BCC byte to buffer
	block0_buffer[uidSize] = bcc;

	// Stop encrypted traffic so we can send raw bytes
	PCD_StopCrypto1(hspi, timeout);

	// Activate UID backdoor
	if (!MIFARE_OpenUidBackdoor(hspi, timeout)) {
		if (logErrors) {
//			Serial.println(F("Activating the UID backdoor failed."));
		}
		return false;
	}

	// Write modified block 0 back to card
	status = MIFARE_Write(hspi, (uint8_t)0, block0_buffer, (uint8_t)16, timeout);
	if (status != STATUS_OK) {
		if (logErrors) {
//			Serial.print(F("MIFARE_Write() failed: "));
//			Serial.println(GetStatusCodeName(status));
		}
		return false;
	}

	// Wake the card up again
	uint8_t atqa_answer[2];
	uint8_t atqa_size = 2;
	PICC_WakeupA(hspi, atqa_answer, &atqa_size, timeout);

	return true;
}

/**
 * Resets entire sector 0 to zeroes, so the card can be read again by readers.
 */
uint8_t MIFARE_UnbrickUidSector(SPI_HandleTypeDef *  hspi,uint8_t logErrors, uint16_t timeout) {
	MIFARE_OpenUidBackdoor(hspi, timeout);

	uint8_t block0_buffer[] = {0x01, 0x02, 0x03, 0x04, 0x04, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	// Write modified block 0 back to card
	StatusCode status = MIFARE_Write(hspi, (uint8_t)0, block0_buffer, (uint8_t)16, timeout);
	if (status != STATUS_OK) {
		if (logErrors) {
//			Serial.print(F("MIFARE_Write() failed: "));
//			Serial.println(GetStatusCodeName(status));
		}
		return false;
	}
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return bool
 */
uint8_t PICC_IsNewCardPresent(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	PCD_WriteRegister(hspi, TxModeReg, 0x00, timeout);
	PCD_WriteRegister(hspi, RxModeReg, 0x00, timeout);
	// Reset ModWidthReg
	PCD_WriteRegister(hspi, ModWidthReg, 0x26, timeout);

	StatusCode result = PICC_RequestA(hspi, bufferATQA, &bufferSize, timeout);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return bool
 */
uint8_t PICC_ReadCardSerial(SPI_HandleTypeDef *  hspi, uint16_t timeout) {
	StatusCode result = PICC_Select(hspi, &uid, 0, timeout);
	return (result == STATUS_OK);
} // End


//void rc(void){
//
//	int scs = PCD_PerformSelfTest(&hspi2,100);
//	if ( ! PICC_IsNewCardPresent(&hspi2,100) || ! PICC_ReadCardSerial(&hspi2,100) ) {
//	    HAL_Delay(50);
//	    return;
//	  }
//	HAL_Delay(1000);
//}





























