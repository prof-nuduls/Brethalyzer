/*
 * RFIDlib.c
 *
 *  Created on: Feb 19, 2021
 *      Author: kirchhtj
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
/* Standard Defines */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <RFIDlib.h>



extern void RFID_RegWrite(uint8_t reg, uint8_t data) {
    GPIO_setOutputLowOnPin(SSport, SSpin);  //enable communication
    SPI_transmitData(EUSCI_B0_BASE, reg);   //send reg
    SPI_transmitData(EUSCI_B0_BASE, data);  //send data
    GPIO_setOutputHighOnPin(SSport, SSpin); //disable communication
}

void RFID_RegWriteMult(uint8_t reg, uint8_t length, uint8_t *values) {
    GPIO_setOutputLowOnPin(SSport, SSpin);  //enable communication
    SPI_transmitData(EUSCI_B0_BASE, reg);   //send reg
    uint8_t i;
    for (i = 0; i < length; i++) { //send data
        SPI_transmitData(EUSCI_B0_BASE, values[i]);
    }
    GPIO_setOutputHighOnPin(SSport, SSpin); //disable communication
}

extern uint8_t RFID_RegRead(uint8_t reg) {
    GPIO_setOutputLowOnPin(SSport, SSpin);          //enable communication
    SPI_transmitData(EUSCI_B0_BASE, reg);           //send reg
    uint8_t val = SPI_receiveData(EUSCI_B0_BASE);   //gets value from reg
    GPIO_setOutputHighOnPin(SSport, SSpin);         //disable communication
    return val;
}

void RFID_RegReadMult(uint8_t reg, uint8_t count, uint8_t *values) {
    //Serial.print(F("Reading "));  Serial.print(count); Serial.println(F(" uint8_ts from register."));
    uint8_t address = 0x80 | reg;              // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t index = 0;                         // Index in values array.
    GPIO_setOutputLowOnPin(SSport, SSpin);          //enable communication
    count--;                                // One read is performed outside of the loop
    SPI_transmitData(EUSCI_B0_BASE, address);                  // Tell MFRC522 which address we want to read
    while (index < count) {
        values[index] = SPI_receiveData(EUSCI_B0_BASE);  // Read value and tell that we want to read the same address again.
        index++;
        SPI_transmitData(EUSCI_B0_BASE, address);
    }
    values[index] = SPI_receiveData(EUSCI_B0_BASE);            // Read the final uint8_t. Send 0 to stop reading.
    GPIO_setOutputHighOnPin(SSport, SSpin);         //disable communication

}

//this code is all adapted from miguelbalboa/rfid github
extern void RFID_Config() {
    // Reset baud rates
    RFID_RegWrite(TxModeReg, 0x00);
    RFID_RegWrite(RxModeReg, 0x00);
    // Reset ModWidthReg
    RFID_RegWrite(ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    RFID_RegWrite(TModeReg, 0x80);       // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    RFID_RegWrite(TPrescalerReg, 0xA9);  // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    RFID_RegWrite(TReloadRegH, 0x03);    // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    RFID_RegWrite(TReloadRegL, 0xE8);

    RFID_RegWrite(TxASKReg, 0x40);       // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    RFID_RegWrite(ModeReg, 0x3D);        // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)

    //deal with antenaaa
    //enable communication
    GPIO_setOutputLowOnPin(SSport, SSpin);
    //send reg
    SPI_transmitData(EUSCI_B0_BASE, (0x80 | TxControlReg));
    uint8_t maskval = SPI_receiveData(EUSCI_B0_BASE);
    GPIO_setOutputHighOnPin(SSport, SSpin);
    RFID_RegWrite(TxControlReg, (maskval | 0x03));
}

extern bool PICC_IsNewCardPresent() {
    //clears parts of a bit in CollReg
    uint8_t tmp;
    tmp = RFID_RegRead(CollReg);
    RFID_RegWrite(CollReg, tmp & (~0x80));      // clear bit mask

    uint8_t validbits = 7;
    //start communicating
    uint8_t senddata = PICC_CMD_REQA;
    uint8_t bufferATQA[2];
    uint8_t buffersize = sizeof(bufferATQA);
    enum StatusCode result = CommunicateWithPICC(PCD_Transceive, 0x30, &senddata, 1, bufferATQA, &buffersize, &validbits, 0, false);
    return (result == STATUS_OK || result == STATUS_COLLISION);
}

extern enum StatusCode CommunicateWithPICC( uint8_t command,       ///< The command to execute. One of the PCD_Command enums.
                                    uint8_t waitIRq,       ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                    uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                    uint8_t sendLen,       ///< Number of uint8_ts to transfer to the FIFO.
                                    uint8_t *backData,     ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                    uint8_t *backLen,      ///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
                                    uint8_t *validBits,    ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
                                    uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                    bool checkCRC       ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
                                   ) {
    RFID_RegWrite(CommandReg, PCD_Idle);            // Stop any active command.
    RFID_RegWrite(ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
    RFID_RegWrite(FIFOLevelReg, 0x80);              // FlushBuffer = 1, FIFO initialization
    RFID_RegWriteMult(FIFODataReg, sendLen, sendData);  // Write sendData to the FIFO
    RFID_RegWrite(BitFramingReg, 0);                // Bit adjustments
    RFID_RegWrite(CommandReg, command);             // Execute the command

    if (command == PCD_Transceive) {
        //enable communication
        GPIO_setOutputLowOnPin(SSport, SSpin);
        //send reg
        SPI_transmitData(EUSCI_B0_BASE, (0x80 | TxControlReg));
        uint8_t maskval = SPI_receiveData(EUSCI_B0_BASE);
        GPIO_setOutputHighOnPin(SSport, SSpin);
        RFID_RegWrite(TxControlReg, (maskval | 0x03));
    }

    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86μs.
    // TODO check/modify for other architectures than Arduino Uno 16bit
    uint16_t i;
    for (i = 2000; i > 0; i--) {
        uint8_t n = RFID_RegRead(ComIrqReg);   // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq) {                  // One of the interrupts that signal success has been set.
            break;
        }
        if (n & 0x01) {                     // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }
    // 35.7ms and nothing happend. Communication with the MFRC522 might be down.
    if (i == 0) {
        return STATUS_TIMEOUT;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = RFID_RegRead(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13) {  // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    uint8_t _validBits = 0;

    if (backData && backLen) {
        uint8_t n = RFID_RegRead(FIFOLevelReg);    // Number of uint8_ts in the FIFO
        if (n > *backLen) {
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                           // Number of uint8_ts returned
        RFID_RegReadMult(FIFODataReg, n, backData);    // Get received data from FIFO
        _validBits = RFID_RegRead(ControlReg) & 0x07;       // RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
        if (validBits) {
            *validBits = _validBits;
        }
    }

    // Tell about collisions
    if (errorRegValue & 0x08) {     // CollErr
        return STATUS_COLLISION;
    }

    return STATUS_OK;
}


extern bool PICC_CardRead(uint8_t *tagID) {
    return (STATUS_OK == PICC_Select(tagID));
}

extern enum StatusCode PICC_Select(uint8_t *tagID) {
    // Prepare MFRC522
    uint8_t tmp;
    tmp = RFID_RegRead(CollReg);
    RFID_RegWrite(CollReg, tmp & (~0x80));      // clear bit mask

    //pre varibles
    bool uidComplete;
    bool selectDone;
    uint8_t count;
    uint8_t checkBit;
    uint8_t index;
    uint8_t uidIndex;                  // The first index in uid->uiduint8_t[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits;       // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];                 // The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
    uint8_t bufferUsed;                // The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
    uint8_t rxAlign;                   // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;                // Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
    uint8_t *responseBuffer;
    uint8_t responseLength;
    enum StatusCode result;

    buffer[0] = PICC_CMD_SEL_CL1;
    uidIndex = 0;
    currentLevelKnownBits = 0;

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while(!uidComplete) {
        selectDone = false;
        while (!selectDone) {
            // Find out how many bits and uint8_ts to send and receive.
            if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
                //Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = PCD_CalculateCRC(buffer, 7, &(buffer[7]));
                if (result != STATUS_OK) {
                    return result;
                }
                txLastBits      = 0; // 0 => All 8 bits are valid.
                bufferUsed      = 9;
                // Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer  = &buffer[6];
                responseLength  = 3;
            }
            else { // This is an ANTICOLLISION.
                //Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
                txLastBits      = currentLevelKnownBits % 8;
                count           = currentLevelKnownBits / 8;    // Number of whole uint8_ts in the UID part.
                index           = 2 + count;                    // Number of whole uint8_ts: SEL + NVB + UIDs
                buffer[1]       = (index << 4) + txLastBits;    // NVB - Number of Valid Bits
                bufferUsed      = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer  = &buffer[index];
                responseLength  = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                           // Having a separate variable is overkill. But it makes the next line easier to read.
            RFID_RegWrite(BitFramingReg, (rxAlign << 4) + txLastBits);  // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = CommunicateWithPICC(PCD_Transceive, 0x30, buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);

            if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
                uint8_t valueOfCollReg = RFID_RegRead(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
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
                currentLevelKnownBits   = collisionPos;
                count           = currentLevelKnownBits % 8; // The bit to modify
                checkBit        = (currentLevelKnownBits - 1) % 8;
                index           = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index]   |= (1 << checkBit);
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
        index           = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        uint8_t bytesToCopy     = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++) {
            tagID[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK) {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
            return STATUS_CRC_WRONG;
        }
        else {
            uidComplete = true;
        }
    } // End of while (!uidComplete)

    return STATUS_OK;
}

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
extern enum StatusCode PCD_CalculateCRC(  uint8_t *data,     ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                                   uint8_t length,    ///< In: The number of uint8_ts to transfer.
                                   uint8_t *result    ///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
                                 ) {
    RFID_RegWrite(CommandReg, PCD_Idle);        // Stop any active command.
    RFID_RegWrite(DivIrqReg, 0x04);             // Clear the CRCIRq interrupt request bit
    RFID_RegWrite(FIFOLevelReg, 0x80);          // FlushBuffer = 1, FIFO initialization
    RFID_RegWriteMult(FIFODataReg, length, data);   // Write data to the FIFO
    RFID_RegWrite(CommandReg, PCD_CalcCRC);     // Start the calculation

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
    // TODO check/modify for other architectures than Arduino Uno 16bit

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
    uint16_t i;
    for (i = 5000; i > 0; i--) {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n = RFID_RegRead(DivIrqReg);
        if (n & 0x04) {                                 // CRCIRq bit set - calculation done
            RFID_RegWrite(CommandReg, PCD_Idle);    // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = RFID_RegRead(CRCResultRegL);
            result[1] = RFID_RegRead(CRCResultRegH);
            return STATUS_OK;
        }
    }
    // 89ms passed and nothing happend. Communication with the MFRC522 might be down.
    return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()







