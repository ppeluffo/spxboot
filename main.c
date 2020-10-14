/*
 *  R01 @ 2020-10-13:
 *  El comando que manda 'A' para fijar la direccion de la página donde escribir, lleva
 *  2 bytes AH, AL.
 *  Esto hace que la dirección de página sea de 16 bits lo que limita el tamaño de los programas
 *  ya que como el XMEGA256 tiene 128Kwords, necesito 17 bits y no 16.
 *  Para solucionarlo uso el autoincrement de modo de calcular aqui la próxima dirección y no
 *  darle bola al avrdude.
 *  Cambios:
 *  - En el comando 'A' ignoro los valores y no seteo el address.
 *  - En el comando 'B', luego de grabar la página incremento el address ( en words !!! )
 *
 *
 *  Dado que no tengo forma de usar los pines de control para determinar si entro o no en
 *  el bootload, implemento el siguiente protocolo.
 *  Cuando arranca, envia el string "HELLO\n".
 *  Espera hasta 10s por recibir a 9600N81 la palabra 'boot'.
 *  Si la recibo, respondo con OK\n y ? y entro en modo bootloader.
 *  Al expirar los 10s, si no la recibi salto al programa principal.
 *
 *  De este modo, cuando quiera que entre en modo bootloader, conecto una terminal a 9600
 *  y reseteo el datalogger.
 *  Cuando recibo 'HELLO' respondo con 'boot' y salgo del terminal y bajo el programa con
 *  el avrdude.
 *
 *  Si quiero que arranque normalmente, en los 10s no trasmito nada y solo salta al programa.
 *
 *  Debo inicializar un timer para que cuente hasta 10s.
 *
 *   *  El watchdog debe estar siempre prendido por fuses.
 *  FF 0A FD __ F5 D4
 *
 *  PROGRAMACION FUSES:
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Uflash:w:spx.hex:a -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -u -Ufuse0:w:0xff:m -Ufuse1:w:0x0:m -Ufuse2:w:0xff:m -Ufuse4:w:0xff:m -Ufuse5:w:0xff:m
 *
 *  /usr/bin/avrdude -px256a3b -cavrispmkII -Pusb -Uflash:w:/home/pablo/Spymovil/workspace/spxR5/Release/spxR5.hex:a
 *  /usr/bin/avrdude -p x256a3b -P /dev/ttyUSB0 -b 9600 -c avr109 -v -v -e -V -Uflash:w:/home/pablo/Spymovil/workspace/spxR5/Release/spxR5.hex
 *
 *  Para ver el uso de memoria usamos
 *  avr-nm -n spxR5.elf | more
 *  avr-nm -Crtd --size-sort spxR5.elf | grep -i ' [dbv] '
 *
 *
 *
 */
#include "defines.h"
#include "serial.h"
#include <avr/io.h>
#include "nvm.h"
#include "stdbool.h"

#define ADDR_T unsigned long
#define PAGESIZE_IN_WORDS 256

unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T address);
void BlockRead(unsigned int size, unsigned char mem, ADDR_T address);

#ifdef __ICCAVR__
#  define C_TASK __C_task
#else /* ! __ICCAVR__ */
#  define C_TASK /**/
#endif /* __ICCAVR__ */

uint8_t WriteBuffer[FLASH_PAGE_SIZE];
uint8_t ReadBuffer[FLASH_PAGE_SIZE];

//-----------------------------------------------------------------------------
/* Set interrupt vector location to boot section of flash */
void PMIC_SetVectorLocationToBoot( void )
{
   uint8_t temp = PMIC.CTRL | PMIC_IVSEL_bm;
   CCP = CCP_IOREG_gc;
   PMIC.CTRL = temp;
}
//-----------------------------------------------------------------------------
/*Set interrupt vector location to application section of flash */
void PMIC_SetVectorLocationToApplication( void )
{
   uint8_t temp = PMIC.CTRL & ~PMIC_IVSEL_bm;
   CCP = CCP_IOREG_gc;
   PMIC.CTRL = temp;
}
//-----------------------------------------------------------------------------
/* New function declarations used for the NVM-workaround */
void EraseApplicationPage(uint32_t address)
{
   /*Set the correct settings and store critical registers before NVM-workaround*/
   /*Assembly "function" to preform page erase*/
    nvm_flash_erase_app_page(address);
}
//-----------------------------------------------------------------------------
void EraseWriteApplicationPage(uint32_t address)
{
   /*Set the correct settings and store critical registers before NVM-workaround*/
   /*Assembly "function" to preform page erase-write*/
   nvm_flash_atomic_write_app_page(address); 
}
//-----------------------------------------------------------------------------
void WriteApplicationPage(uint32_t address)
{
   /*Set the correct settings and store critical registers before NVM-workaround*/
   /*Assembly "function" to preform page write*/
   nvm_flash_split_write_app_page(address);
}
//-----------------------------------------------------------------------------
void LoadFlashWord(uint32_t address, uint16_t word )
{
   /*Set the correct settings and store critical registers before NVM-workaround*/
   /*Assembly "function" to load word into flash buffer*/
  nvm_flash_load_word_to_buffer(address, word);
}
//-----------------------------------------------------------------------------
void EraseFlashBuffer(void)
{
   /*Set the correct settings and store critical registers before NVM-workaround*/
   /*Assembly "function" to erase flash buffer*/
    nvm_flash_flush_buffer();
}
//-----------------------------------------------------------------------------
uint8_t boot(void)
{

unsigned char rxChar;
unsigned char rxStr[5];
int8_t i = -1;

	// Inicializo la UART y mando el mensaje de fish
	initbootuart(); // Initialize UART a 9600 N81
	sendchar('H'); 	// Fishing line
	sendchar('E');
	sendchar('L');
	sendchar('L');
	sendchar('O');
	sendchar('r');
	sendchar('0');
	sendchar('2');
	sendchar('\r');

	// Inicializo y arranco el timer.

	// Configuro el timer0 para contar hasta 10s.
	// El micro arranca con un clock de 2Mhz.
	// Pongo un prescaler de 1024.
	// Cuento desde 0.
	// Cada tick es de 2Mhz/1024 o sea 0.5ms.
	// Para llegar a 10s debo contar hasta 20000.

	TCC0.CNT = 0x00;		// Arranco en 0.
	TCC0.CTRLA = 0x07;	// Prescaler a 1024 y arranco el timer.

	// Loop de espera
	while ( TCC0.CNT < 20000 ) {

		// Si recibi un dato lo leo
		if ( UART_STATUS_REG & (1 << RECEIVE_COMPLETE_BIT)) {

			// Proceso el byte recibido
			rxChar = UART_DATA_REG;
			switch ( rxChar ) {
			case 'b':	// Inicio
				i = 0;
				rxStr[i++] = rxChar;
				break;
			case '\r':	// Fin
				if ( ( rxStr[0] == 'b') & ( rxStr[1] == 'o') & ( rxStr[2] == 'o') & ( rxStr[3] == 't') ) {
					TCC0.CTRLA = 0x00;	// Apago el timer
					return(1);
				}
				break;
			default:
				if ( i > 0 ) {
					rxStr[i++] = rxChar;
				}
			}

			// Solo espero 5 bytes.
			if ( i > 5 ) {
				// Ya llego un string con errores. Salgo
				break;
			}
		}

	}

	sendchar('B');
	sendchar('Y');
	sendchar('E');
	sendchar('\r');
	TCC0.CTRLA = 0x00;
	return(0);

}
//-----------------------------------------------------------------------------
int main(void)
{

ADDR_T address = 0;
unsigned int temp_int=0;
unsigned char val;
bool autoincrement = false;

   /* Initialization */    
void (*funcptr)( void ) = 0x0000; // Set up function pointer to RESET vector.

	PMIC_SetVectorLocationToBoot();
	eeprom_disable_mapping();

	if ( boot() == 1 ) {
      initbootuart(); // Initialize UART.

      /* Main loop */
      for(;;) {
	 
    	  val = recchar(); // Wait for command character.

    	  if(val=='P') {
    		  // Enter and leave programming mode.
    		  sendchar('\r'); // Nothing special to do, just answer OK.

    	  } else if(val=='a') {
    		  // Check autoincrement status.
    		  sendchar('Y'); // Yes, we do autoincrement.
    		  autoincrement = true;

    	  } else if(val == 'A') {
    		  // Set address (2 bytes).
    		  // NOTE: Flash addresses are given in words, not bytes.
    		  if ( autoincrement == true ) {
    			  // Descarto los datos ya que uso autoincrement
    			  recchar();
    			  recchar();
    		  } else {
    			  address = recchar();
    			  address <<=  8;
    			  address |= recchar(); // Read address high and low byte.
    		  }
    		  sendchar('\r'); // Send OK back.
	 
    	  } else if(val=='c') {
    		  // Write program memory, low byte.
    		  // NOTE: Always use this command before sending high byte.
    		  temp_int=recchar(); // Get low byte for later LoadFlashWord
    		  sendchar('\r'); // Send OK back.

    	  } else if(val=='C') {
    		  // Write program memory, high byte.
    		  temp_int |= (uint16_t)recchar()<<8; // Get and insert high byte.
    		  nvm_wait_until_ready();
    		  LoadFlashWord( (address << 1), temp_int );
    		  address++; // Auto-advance to next Flash word.
    		  sendchar('\r'); // Send OK back.

    	  } else if(val== 'm') {
    		  // Write page.
    		  if( address >= (APP_END>>1) )  {
    			  // Protect bootloader area.
    			  sendchar('?');
    		  } else {
    			  nvm_wait_until_ready();
    			  // Convert word-address to byte-address and write.
    			  EraseWriteApplicationPage( address << 1);
    			  sendchar('\r'); // Send OK back.
    		  }

    	  } else if(val=='R') {
    		  // Read program memory.
    		  // Send high byte, then low byte of flash word.
    		  nvm_wait_until_ready();
    		  sendchar(nvm_flash_read_byte( (address << 1)+1) );
    		  sendchar( nvm_flash_read_byte( (address << 1)+0) );
    		  address++; // Auto-advance to next Flash word.

    	  } else if (val == 'd') {
    		  // Read EEPROM memory.
    		  sendchar(nvm_eeprom_read_byte(address));
    		  // Select next EEPROM byte
    		  address++;

    	 } else if (val == 'D') {
    		 // Write EEPROM memory.
    		 nvm_eeprom_write_byte(address,recchar());
    		 // Select next EEPROM byte
    		 address++;

    	 } else if(val=='e') {
    		 // Chip erase.
    		 for(address = 0; address < APP_END; address += PAGESIZE) {
    			 // NOTE: Here we use address as a byte-address, not word-address, for convenience.
    			 nvm_wait_until_ready();
    			 EraseApplicationPage( address );
    		 }
    		 nvm_eeprom_erase_all();
    		 //	    sendchar('o');
    		 //	    sendchar('k');
    		 sendchar('\r'); // Send OK back.

    	 } else if(val=='L') {
    		 // Enter and leave programming mode.
    		 sendchar('\r'); // Nothing special to do, just answer OK.

    	 } else if (val=='T') {
    		 // Select device type
    		 recchar(); // Ignore the command and it's parameter.
    		 sendchar('\r'); // Send OK back.

    	 }  else if(val=='s') {
    		 // Return signature bytes.
    		 sendchar( SIGNATURE_BYTE_3 );
    		 sendchar( SIGNATURE_BYTE_2 );
    		 sendchar( SIGNATURE_BYTE_1 );

    	 } else if(val=='t') {
    		 // Return supported device codes.
    		 sendchar( PARTCODE ); // Supports only this device, of course.
    		 sendchar( 0 ); // Send list terminator.

    	 } else if(val=='S') {
    		 // Return programmer identifier.
    		 sendchar('A'); // Return 'AVRBOOT'.
    		 sendchar('V'); // Software identifier (aka programmer signature) is always 7 characters.
    		 sendchar('R');
    		 sendchar('B');
    		 sendchar('O');
    		 sendchar('O');
    		 sendchar('T');

    	 } else if(val=='V') {
    		 // Return software version.
    		 sendchar('0');
    		 sendchar('2');

    	 }  else if (val=='p') {
    		 // Get programmer type.
    		 sendchar('S'); // Answer 'SERIAL'.

    	 } else if (val=='x') {
    		 // Set LED
    		 recchar(); // Ignore the command and it's parameter.
    		 sendchar('\r'); // Send OK back.

    	 } else if(val=='y') {
    		 // clear LED
    		 recchar(); // Ignore the command and it's parameter.
    		 sendchar('\r'); // Send OK back.

    	 } else if(val=='E') {
    		 // Exit bootloader.
    		 nvm_wait_until_ready();
    		 sendchar('\r');
    		 PMIC_SetVectorLocationToApplication();
    		 EIND = 0x00;
    		 funcptr(); // Jump to Reset vector 0x0000 in Application Section.

    	 } else if(val=='b') {
    		 // Check block load support.
    		 sendchar('Y'); // Report block load supported.
    		 sendchar((BLOCKSIZE>>8) & 0xFF); // MSB first.
    		 sendchar(BLOCKSIZE&0xFF); // Report BLOCKSIZE (bytes).

    	 } else if(val=='B') {
    		 // Start block load.
    		 // DEBUG: Uso autoincrement
    		 temp_int = ((uint16_t)recchar()<<8) | recchar(); 	// Get block size.
    		 val = recchar(); // Get memtype.
    		 sendchar( BlockLoad(temp_int, val, address) ); 	// Block load.
    		 if ( autoincrement == true ) {
    			 address += PAGESIZE_IN_WORDS;
    		 }

    	 }	else if(val=='g') {
    		 // Start block read.
    		 temp_int = ((uint16_t)recchar()<<8) | recchar(); // Get block size.
    		 val = recchar(); // Get memtype
    		 BlockRead(temp_int, val, address); // Block read
	 
    	 } else if(val!=0x1b) {
		 // The last command to accept is ESC (synchronization).
		 // If not ESC, then it is unrecognized...
    		 sendchar('?');
    	 }

      } // end: for(;;)

   }  else {
	   nvm_wait_until_ready();
	   PMIC_SetVectorLocationToApplication();
	   EIND = 0x00;
	   funcptr(); // Jump to Reset vector 0x0000 in Application Section.
   }

} // end: main
//------------------------------------------------------------------------------------
unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T address)
{

unsigned int data;
ADDR_T tempaddress;
unsigned char value;
unsigned char buffer[BLOCKSIZE];
   
   if(mem=='E')   {
	   // EEPROM memory type.
	  nvm_eeprom_flush_buffer();
      // disable mapping of EEPROM into data space (enable IO mapped access)
	  eeprom_disable_mapping();
      // Fill buffer first, as EEPROM is too slow to copy with UART speed 
      for(tempaddress=0;tempaddress<size;tempaddress++) {
    	  buffer[tempaddress] = recchar();
      }
      
      // Then program the EEPROM 
      for( tempaddress=0; tempaddress < size; tempaddress++) {
	 	 value = buffer[tempaddress];
	 	 nvm_eeprom_write_byte(address,value);
	 	 (address)++; // Select next EEPROM byte
      }
      
      return '\r'; // Report programming OK
   
   }  else if(mem=='F')  {
	   // Flash memory type
	   // NOTE: For flash programming, 'address' is given in words.
      address <<= 1; // Convert address to bytes temporarily.
      EraseFlashBuffer(); //Erase the flash buffer to avoid data corruption
      for (int i = 0; i < size; i +=2) {
    	  data = recchar();
    	  data |= (uint16_t)recchar() << 8;
    	  nvm_wait_until_ready();
    	  LoadFlashWord(i, data);
      }
      EraseWriteApplicationPage(address);
      nvm_wait_until_ready();
      return '\r'; // Report programming OK
   
   } else {
	   // Invalid memory type?
	   return '?';
   }
}
//------------------------------------------------------------------------------------
void BlockRead(unsigned int size, unsigned char mem, ADDR_T address)
{
   // EEPROM memory type.
   
   if (mem=='E') {
	   // Read EEPROM
      eeprom_disable_mapping();
	  nvm_eeprom_flush_buffer();
      do  {
    	  sendchar(nvm_eeprom_read_byte(address));
    	  // Select next EEPROM byte
    	  (address)++;
    	  size--; // Decrease number of bytes to read
      } while (size); // Repeat until all block has been read
   
   } else if(mem=='F') {
	   // Flash memory type.
      (address) <<= 1; // Convert address to bytes temporarily.
      do {
    	  sendchar( nvm_flash_read_byte( address) );
    	  sendchar( nvm_flash_read_byte( (address)+1) );
    	  (address) += 2;    // Select next word in memory.
    	  size -= 2;          // Subtract two bytes from number of bytes to read
      } while (size);         // Repeat until all block has been read

      (address) >>= 1;       // Convert address back to Flash words again.
   }
}
//------------------------------------------------------------------------------------
/* end of file */
