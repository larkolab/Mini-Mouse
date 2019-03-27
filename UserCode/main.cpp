/*!
 * \file      main.c
 *
 * \brief     Description : utility to detect sx1272/sx1261 radio
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 *
  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 

*/

#include "SX126x.h"
#include "sx1272.h"
#include "ApiMcu.h"
#include "utilities.h"

#define FileId 4

/*!
 * \brief   Radio Interrupt Pin declarations
 */
#ifdef MURATA_BOARD
    McuXX<McuSTM32L072> mcu ( LORA_SPI_MOSI, LORA_SPI_MISO, LORA_SPI_SCLK ) ;
#endif
#ifdef BOARD_L4
    McuXX<McuSTM32L4> mcu ( LORA_SPI_MOSI, LORA_SPI_MISO, LORA_SPI_SCLK ) ;
#endif

int main( void ) {
    uint8_t uid[8];
    uint8_t version = 0;
    SX1272 * Radio_SX1272;
    SX126x * Radio_SX1261;
    uint8_t buff[8];

    mcu.InitMcu( );
    mcu.GetUniqueId( uid ); 
    DEBUG_PRINTF( "MM is starting ...{ %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x } \n",uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7] );

    /* Try to detect an SX1272 radio */
    Radio_SX1272 = new SX1272( D10, A0, D2, D3 ); /* LORA_CS, LORA_RESET, TX_RX_IT, RX_TIMEOUT_IT */
    if( Radio_SX1272 != NULL )
    {
        Radio_SX1272->Reset( );
        version = Radio_SX1272->Read( 0x42 ); /* Get Version */
        if( version == 0x22 )
        {
            while( 1 )
            {
                DEBUG_MSG( "SX1272 radio detected\n" );
                mcu.mwait_ms( 1000 );
            }
        }
        else
        {
            DEBUG_PRINTF( "SX1272: ==> 0x%02X\n", version );
        }

        delete Radio_SX1272;
        Radio_SX1272 = NULL;
    }
    else
    {
        DEBUG_MSG( "ERROR: failed to instantiate SX1272 radio object\n" );
    }

    /* If no SX1272 radio has ben found, try to detect an SX1261 radio */
    Radio_SX1261 = new SX126x( D3, D7, A0, D5 ); /* LORA_BUSY, LORA_CS, LORA_RESET,TX_RX_IT */
    if( Radio_SX1261 != NULL )
    {
        Radio_SX1261->Reset( );
        mcu.mwait_ms( 1000 );
        Radio_SX1261->ReadCommand( SX126x::OpCode_t::GET_STATUS, buff, 1 );
        if( buff[0] == 0x22 )
        {
            while( 1 )
            {
                DEBUG_MSG( "SX1261 radio detected\n" );
                mcu.mwait_ms( 1000 );
            }
        }
        else {
            DEBUG_PRINTF( "SX1261: ==> 0x%02X\n", buff[0] );
        }

        delete Radio_SX1261;
        Radio_SX1261 = NULL;
    }
    else
    {
        DEBUG_MSG( "ERROR: failed to instantiate SX1261 radio object\n" );
    }

    /* Nothing found */
    while( 1 )
    {
        DEBUG_MSG( "ERROR: no SX1272 or SX1261 radio found\n" );
        mcu.mwait_ms( 1000 );
    }

    return 0;
}
