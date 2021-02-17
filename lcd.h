/*!
 * lcd.h
 *
 *      Description: Header file for LCD library. For Hitachi HD44780 parallel LCD
 *               in 8-bit mode.
 *
 *      Author: ece230
 */

#ifndef LCD_H_
#define LCD_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#define CTRL_MODE       0
#define DATA_MODE       1
#define LINE1_OFFSET    0x0
#define LINE2_OFFSET    0x40

/* Instruction masks */
#define CLEAR_DISPLAY_MASK  0x01
#define RETURN_HOME_MASK    0x02
#define ENTRY_MODE_MASK     0x04
#define DISPLAY_CTRL_MASK   0x08
#define CURSOR_SHIFT_MASK   0x10
#define FUNCTION_SET_MASK   0x20
#define SET_CGRAM_MASK      0x40
#define SET_CURSOR_MASK     0x80

/* Field masks for instructions:
 * DL   = 1: 8 bits, DL = 0: 4 bits
 * N    = 1: 2 lines, N = 0: 1 line
 * S/C  = 1: Display shift
 * S/C  = 0: Cursor move
 * F    = 1: 5 x 10 dots, F = 0: 5 x 8 dots
 * R/L  = 1: Shift to the right
 * R/L  = 0: Shift to the left
 * D    = 1: Display On, D = 0: Display Off
 * C    = 1: Cursor On, D = 0: Cursor Off
 * I/D  = 1: Increment
 * I/D  = 0: Decrement
 * B    = 1: Cursor blink On, D = 0: Cursor blink Off
 * S    = 1: Accompanies display shift
 * BF   = 1: Internally operating
 * BF   = 0: Instructions acceptable
 */
#define DL_FLAG_MASK        0x10
#define N_FLAG_MASK         0x08
#define SC_FLAG_MASK        0x08
#define F_FLAG_MASK         0x04
#define RL_FLAG_MASK        0x04
#define D_FLAG_MASK         0x04
#define C_FLAG_MASK         0x02
#define ID_FLAG_MASK        0x02
#define B_FLAG_MASK         0x01
#define S_FLAG_MASK         0x01

/*!
 *
 *  \brief This function configures the selected pins for an LCD
 *
 *  This function configures the selected pins as output pins to interface
 *      with a Hitachi HD44780 LCD in 8-bit mode.
 *         Valid values for ports are:
 *         - \b GPIO_PORT_P1
 *         - \b GPIO_PORT_P2
 *         - \b GPIO_PORT_P3
 *         - \b GPIO_PORT_P4
 *         - \b GPIO_PORT_P5
 *         - \b GPIO_PORT_P6
 *         - \b GPIO_PORT_P7
 *         - \b GPIO_PORT_P8
 *         - \b GPIO_PORT_P9
 *         - \b GPIO_PORT_P10
 *         - \b GPIO_PORT_P11
 *
 *         Valid values for pins are:
 *         - \b GPIO_PIN0
 *         - \b GPIO_PIN1
 *         - \b GPIO_PIN2
 *         - \b GPIO_PIN3
 *         - \b GPIO_PIN4
 *         - \b GPIO_PIN5
 *         - \b GPIO_PIN6
 *         - \b GPIO_PIN7
 *
 *  \param rsPort is the port for the RS signal
 *  \param rsPin is the pin in the selected port for the RS signal
 *  \param enPort is the port for the Enable signal
 *  \param enPin is the pin in the selected port for the Enable signal
 *  \param dbPort is the port for DB0-7
 *
 *  Modified bits of \b PxDIR register and bits of \b PxSEL register.
 *
 *  \return None
 */
extern void configLCD(uint_fast8_t rsPort, uint_fast16_t rsPin,
                    uint_fast8_t enPort, uint_fast16_t enPin,
                    uint_fast8_t dbPort);
/*!
 *  \brief This function updates the display according to the arrays
 *
 *  This function prints ASCII characters to the first 16 character in DDRAM on each line of LCD.
 *
 *  \param line1 is the first line to display on LCD
 *  \param line1 is the second character to display on LCD
 *
 *  \return None
 */
extern void updateScreen(char line1[], char line2[]);

/*!
 *  \brief This function initializes LCD
 *
 *  This function generates initialization sequence for LCD for 8-bit mode.
 *      Delays set by worst-case 2.7 V
 *
 *  \return None
 */

extern void initLCD(void);

/*!
 *  \brief This function prints character to current cursor position
 *
 *  This function prints ASCII character to current cursor position on LCD.
 *
 *  \param character is the character to display on LCD
 *
 *  \return None
 */
extern void commandInstruction(uint8_t command);
extern void printChar(char character);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* LCD_H_ */
