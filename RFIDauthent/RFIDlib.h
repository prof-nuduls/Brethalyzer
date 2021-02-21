/*
 * RFIDlib.h
 *
 *  Created on: Feb 19, 2021
 *      Author: kirchhtj
 */

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#ifndef RFIDLIB_H_
#define RFIDLIB_H_


#define SSport                  (0b100000)
#define SSpin                   (0b100)
    //this code is all taken from miguelbalboa/rfid github
    // MFRC522 registers. Described in chapter 9 of the datasheet.
    // When using SPI all addresses are shifted one bit left in the "SPI address byte" (section 8.1.2.3)
// Page 0: Command and status
//                        0x00          // reserved for future use
#define CommandReg               0x01 << 1    // starts and stops command execution
#define ComIEnReg                0x02 << 1    // enable and disable interrupt request control bits
#define DivIEnReg                0x03 << 1    // enable and disable interrupt request control bits
#define ComIrqReg                0x04 << 1    // interrupt request bits
#define DivIrqReg                0x05 << 1    // interrupt request bits
#define ErrorReg                 0x06 << 1    // error bits showing the error status of the last command executed
#define Status1Reg               0x07 << 1    // communication status bits
#define Status2Reg               0x08 << 1    // receiver and transmitter status bits
#define FIFODataReg              0x09 << 1    // input and output of 64 byte FIFO buffer
#define FIFOLevelReg             0x0A << 1    // number of bytes stored in the FIFO buffer
#define WaterLevelReg            0x0B << 1    // level for FIFO underflow and overflow warning
#define ControlReg               0x0C << 1    // miscellaneous control registers
#define BitFramingReg            0x0D << 1    // adjustments for bit-oriented frames
#define CollReg                  0x0E << 1    // bit position of the first bit-collision detected on the RF interface
        //                       0x0F          // reserved for future use

        // Page 1: Command
        //                       0x10          // reserved for future use
#define ModeReg                  0x11 << 1    // defines general modes for transmitting and receiving
#define TxModeReg                0x12 << 1    // defines transmission data rate and framing
#define RxModeReg                0x13 << 1    // defines reception data rate and framing
#define TxControlReg             0x14 << 1    // controls the logical behavior of the antenna driver pins TX1 and TX2
#define TxASKReg                 0x15 << 1    // controls the setting of the transmission modulation
#define TxSelReg                 0x16 << 1    // selects the internal sources for the antenna driver
#define RxSelReg                 0x17 << 1    // selects internal receiver settings
#define RxThresholdReg           0x18 << 1    // selects thresholds for the bit decoder
#define DemodReg                 0x19 << 1    // defines demodulator settings
        //                       0x1A          // reserved for future use
        //                       0x1B          // reserved for future use
#define MfTxReg                  0x1C << 1    // controls some MIFARE communication transmit parameters
#define MfRxReg                  0x1D << 1    // controls some MIFARE communication receive parameters
        //                       0x1E         // reserved for future use
#define SerialSpeedReg           0x1F << 1    // selects the speed of the serial UART interface
        // Page 2: Configuration
        //                       0x20          // reserved for future use
#define CRCResultRegH            0x21 << 1    // shows the MSB and LSB values of the CRC calculation
#define CRCResultRegL            0x22 << 1
        //                       0x23          // reserved for future use
#define ModWidthReg              0x24 << 1    // controls the ModWidth setting?
        //                       0x25          // reserved for future use
#define RFCfgReg                 0x26 << 1    // configures the receiver gain
#define GsNReg                   0x27 << 1    // selects the conductance of the antenna driver pins TX1 and TX2 for modulation
#define CWGsPReg                 0x28 << 1    // defines the conductance of the p-driver output during periods of no modulation
#define ModGsPReg                0x29 << 1    // defines the conductance of the p-driver output during periods of modulation
#define TModeReg                 0x2A << 1    // defines settings for the internal timer
#define TPrescalerReg            0x2B << 1    // the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
#define TReloadRegH              0x2C << 1    // defines the 16-bit timer reload value
#define TReloadRegL              0x2D << 1
#define TCounterValueRegH        0x2E << 1    // shows the 16-bit timer value
#define TCounterValueRegL        0x2F << 1

#define PCD_Idle                0x00     // no action, cancels current command execution
#define PCD_Mem                 0x01     // stores 25 bytes into the internal buffer
#define PCD_GenerateRandomID    0x02     // generates a 10-byte random ID number
#define PCD_CalcCRC             0x03     // activates the CRC coprocessor or performs a self-test
#define PCD_Transmit            0x04     // transmits data from the FIFO buffer
#define PCD_NoCmdChange         0x07     // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define PCD_Receive             0x08     // activates the receiver circuits
#define PCD_Transceive          0x0C     // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
#define PCD_MFAuthent           0x0E     // performs the MIFARE standard authentication as a reader
#define PCD_SoftReset           0x0F      // resets the MFRC522

#define PICC_CMD_WUPA           0x52
#define PICC_CMD_REQA           0x26
#define PICC_CMD_SEL_CL1        0x93
#define PICC_CMD_CT             0x88

    // Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
    // last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
    enum StatusCode {
        STATUS_OK               ,   // Success
        STATUS_ERROR            ,   // Error in communication
        STATUS_COLLISION        ,   // Collission detected
        STATUS_TIMEOUT          ,   // Timeout in communication.
        STATUS_NO_ROOM          ,   // A buffer is not big enough.
        STATUS_INTERNAL_ERROR   ,   // Internal error in the code. Should not happen ;-)
        STATUS_INVALID          ,   // Invalid argument.
        STATUS_CRC_WRONG        ,   // The CRC_A does not match
        STATUS_MIFARE_NACK      = 0xff  // A MIFARE PICC responded with NAK.
    };


//below here is my code
/*!
 *  \brief This function prints acceleration values to terminal
 *
 *  This function will write a value to the regerister on the RFID with address reg
 *
 *  \param reg is the address we are writing to
 *  \param data is the data being written at the address
 *
 *  \return None
 */
extern void RFID_RegWrite(uint8_t reg, uint8_t data);

/*!
 *  \brief This function prints acceleration values to terminal
 *
 *  This function will read the value stored at register address reg on the RFID
 *
 *  \param reg is the address we are reading from
 *
 *  \return the value stored in the register on RFID
 */
extern uint8_t RFID_RegRead(uint8_t reg);

/*!
 *  \brief This function prints acceleration values to terminal
 *
 *  This function configures our RFID
 *
 *  \return None
 */
extern void RFID_Config();

/*!
 *  \brief This function prints acceleration values to terminal
 *
 *  This function tells whether there is a card present
 *
 *  \return None
 */
extern bool PICC_IsNewCardPresent();

/*!
 *  \brief This function prints acceleration values to terminal
 *
 *  This function configures our RFID
 *
 *  \param SSport the enable port
 *  \param SSpin the enable pin
 *
 *  \return None
 */
extern enum StatusCode CommunicateWithPICC( uint8_t command,       ///< The command to execute. One of the PCD_Command enums.
                                    uint8_t waitIRq,       ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                    uint8_t *sendData,     ///< Pointer to the data to transfer to the FIFO.
                                    uint8_t sendLen,       ///< Number of uint8_ts to transfer to the FIFO.
                                    uint8_t *backData,     ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                    uint8_t *backLen,      ///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
                                    uint8_t *validBits,    ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
                                    uint8_t rxAlign,       ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                    bool checkCRC       ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
                                   );

/*!
 *  \brief This function prints acceleration values to terminal
 *
 *  This function tells when the card has been read and fills in the tagID
 *
 *  \return None
 */
extern bool PICC_CardRead(uint8_t *tagID);


/*!
 *  \brief This function prints acceleration values to terminal
 *
 *  This function fills in the tagID
 *
 *  \return None
 */
extern enum StatusCode PICC_Select(uint8_t *tagID);


extern enum StatusCode PCD_CalculateCRC(  uint8_t *data, uint8_t length, uint8_t *result);

#endif /* RFIDLIB_H_ */
