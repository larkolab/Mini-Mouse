/*!
 * \file      main.c
 *
 * \brief     Description : LoRaWAN Stack example
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
  __  __ _       _
 |  \/  (_)     (_)
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___|


 * \endcode

Maintainer        : Fabien Holin (SEMTECH)
*/

#include "LoraMacDataStoreInFlash.h"
#include "LoraWanProcess.h"
#include "Define.h"
#include "utilities.h"
#include "UserDefine.h"
#include "appli.h"
#include "SX126x.h"
#include "ApiMcu.h"
#include "utilities.h"
#include "main.h"
#include "UserDefine.h"
#include "ApiMcu.h"


#define FileId 4

/*!
 * \brief   BackUpFlash The LoraWan structure parameters save into the flash memory for failsafe restauration.
 */

struct sBackUpFlash BackUpFlash;

/*!
 * \brief   Instantiate MCU object to be used by applications
 */

#ifdef MURATA_BOARD
    McuXX<McuSTM32L072> mcu( LORA_SPI_MOSI, LORA_SPI_MISO, LORA_SPI_SCLK ) ;
#elif BOARD_L4
    McuXX<McuSTM32L4> mcu( LORA_SPI_MOSI, LORA_SPI_MISO, LORA_SPI_SCLK ) ;
#endif

/*!
 * \brief   Declare the radio object to be instanciated and used by applications
 */

#ifdef SX126x_BOARD
    #define FW_VERSION 0x18
    SX126x * RadioUser;
#elif SX1276_BOARD
    #define FW_VERSION 0x17
    SX1276 * RadioUser;
#elif SX1272_BOARD
    #define FW_VERSION 0x13
    SX1272 * RadioUser;
#endif

/*!
 * \brief   Parameters of the LoraWanKeys structure.
 * \remark  For APB Devices only NwkSkey, AppSKey and devaddr are mandatory
 * \remark  For OTA Devices only DevEUI, AppEUI and AppKey are mandatory
 */

uint8_t LoRaMacNwkSKeyInit[]      = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
uint8_t LoRaMacAppSKeyInit[]      = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
uint8_t LoRaMacAppKeyInit[]       = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0xBB };
uint8_t AppEuiInit[]              = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xff, 0x50 };
uint8_t DevEuiInit[]              = { 0x38, 0x35, 0x31, 0x31, 0x18, 0x47, 0x37, 0x31 };
uint32_t LoRaDevAddrInit          = 0xB0CAD001;
sLoRaWanKeys LoraWanKeys = { LoRaMacNwkSKeyInit, LoRaMacAppSKeyInit, LoRaMacAppKeyInit, AppEuiInit, DevEuiInit, LoRaDevAddrInit, APB_DEVICE };

/*!
 * \brief   Tets application enum
 */
typedef enum {
    TEST_APP_LORAWAN,
    TEST_APP_TX_SHOTGUN,
    TEST_APP_RX
} eTestApp;

/*!
 * \brief   Function prototypes.
 */

void LoRaWAN_app( void );
void TxShotgun_app( void );
void Rx_app( void );
uint32_t TimeOnAir( uint8_t pktLen, eBandWidth bandwidth, uint8_t Datarate, uint16_t PreambleLen, bool CrcOn, bool HeaderOn, uint8_t Coderate );
void UserIsr( void );

/*!
 * \brief   Main function
 */

int main( void ) {
    uint8_t uid[8];
    eTestApp TestApp = TEST_APP_LORAWAN;

    /* RtcInit , WakeUpInit, LowPowerTimerLoRaInit() are Mcu dependant. */
    mcu.InitMcu( );
    mcu.WatchDogStart( );
    mcu.GetUniqueId( uid );

    /* Restore the LoraWan Context */
    DEBUG_PRINTF( "MM is starting ...{ %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x }\n", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7] );
    mcu.mwait( 2 );

    /* Instantiate the radio object to be used by applications */
#ifdef SX126x_BOARD
    #define FW_VERSION     0x18
    RadioUser = new SX126x( LORA_BUSY, LORA_CS, LORA_RESET, TX_RX_IT );
#elif SX1276_BOARD
    #define FW_VERSION     0x17
    RadioUser = new SX1276( LORA_CS, LORA_RESET, TX_RX_IT, RX_TIMEOUT_IT );
#elif SX1272_BOARD
    #define FW_VERSION     0x13
    RadioUser = new SX1272( LORA_CS, LORA_RESET, TX_RX_IT, RX_TIMEOUT_IT );
#endif

    /* Launch main application */
    switch( TestApp ) {
        case TEST_APP_TX_SHOTGUN:
            TxShotgun_app( );
            break;
        case TEST_APP_RX:
            Rx_app( );
            break;
        default:
            LoRaWAN_app( );
            break;
    }

    /* Never come here anyway... */
    delete RadioUser;
}

/*!
 * \brief   LoRaWAN application
 */

void LoRaWAN_app( void ) {
    uint8_t UserPayloadSize;
    uint8_t UserPayload[255];
    uint8_t UserRxPayloadSize;
    uint8_t UserRxPayload[125];
    uint8_t UserFport;
    uint8_t UserRxFport;
    uint8_t MsgType;
    uint8_t AppTimeSleeping = 5;
    uint8_t AvailableRxPacket = NO_LORA_RXPACKET_AVAILABLE;
    eLoraWan_Process_States LpState = LWPSTATE_IDLE;

    /* Lp<LoraRegionsEU>: A LoRaWan Object with Eu region's rules. */
#ifdef SX126x_BOARD
    LoraWanObject<LoraRegionsEU, SX126x> Lp( LoraWanKeys, RadioUser, USERFLASHADRESS );
#elif SX1276_BOARD
    LoraWanObject<LoraRegionsEU, SX1276> Lp( LoraWanKeys, RadioUser, USERFLASHADRESS );
#elif SX1272_BOARD
    LoraWanObject<LoraRegionsEU, SX1272> Lp( LoraWanKeys, RadioUser, USERFLASHADRESS );
#endif

    /* Prepare UserPayload and User parameters */
    UserFport       = 3;
    UserPayloadSize = 14;
    for ( int i = 0; i < UserPayloadSize; i++ ) {
        UserPayload[i] = i;
    }
    UserPayload[0] = FW_VERSION;

    /* Handle LoRaWanProcess */
#if 0
    Lp.RestoreContext  ( );
#endif
    Lp.SetDataRateStrategy( USER_DR_DISTRIBUTION );
    Lp.NewJoin();

    DEBUG_MSG("\n---------------------> Starting LoRaWAN application <-------------------------------\n");

    while ( 1 ) {
        /* For this example : send an (un)confirmed message on port 3. The user payload is a ramp from 0 to 13 (14 bytes) + FW version. */
        if ( ( Lp.IsJoined ( ) == NOT_JOINED ) && ( Lp.GetIsOtaDevice ( ) == OTA_DEVICE) ) {
            LpState = Lp.Join( );
        } else {
            DEBUG_MSG("\n---------------------> Sending a NEW PACKET <-------------------------------\n");
            MsgType = CONF_DATA_UP;
            LpState = Lp.SendPayload( UserFport, UserPayload, UserPayloadSize, MsgType );
        }

        /*
        * This function manages the state of the MAC and performs all the computation intensive (crypto) tasks of the MAC.
        * This function is called periodically by the user's application whenever the state of the MAC is not idle.
        * The function is not timing critical, can be called at any time and can be interrupted by any IRQ (including the user's IRQ).
        * The only requirement is that this function must be called at least once between the end of the transmission and the beginning of the RX1 slot.
        * Therefore when the stack is active a call periodicity of roughly 300mSec is recommended.
        */
        while ( ( LpState != LWPSTATE_IDLE ) && ( LpState != LWPSTATE_ERROR ) && ( LpState != LWPSTATE_INVALID) ) {
            LpState = Lp.LoraWanProcess( &AvailableRxPacket );
            mcu.GotoSleepMSecond( 300 );
        }

        /* Reset if an error occured */
        if ( LpState == LWPSTATE_ERROR ) {
            InsertTrace ( __COUNTER__, FileId );
            NVIC_SystemReset( );
        }

        /* Everything is OK so far... */
        mcu.WatchDogRelease( );

        /* Is there any downlink to process ? */
        if ( AvailableRxPacket != NO_LORA_RXPACKET_AVAILABLE ) {
            InsertTrace ( __COUNTER__, FileId );
            Lp.ReceivePayload( &UserRxFport, UserRxPayload, &UserRxPayloadSize );
            DEBUG_PRINTF( "Receive on port %d an Applicative Downlink\n DATA[%d] = [ ", UserRxFport, UserRxPayloadSize );
            for ( int i = 0; i < UserRxPayloadSize; i++ ) {
                DEBUG_PRINTF( "0x%.2x ", UserRxPayload[i] );
            }
            DEBUG_MSG( "]\n\n\n" );
        }

        /*
        * Send a Packet every 5 seconds in case of join
        * Send a packet every AppTimeSleeping seconds in normal mode
        */
        if ( ( Lp.IsJoined( ) == NOT_JOINED ) && ( Lp.GetIsOtaDevice( ) == OTA_DEVICE) && ( LpState != LWPSTATE_INVALID) ) {
            InsertTrace( __COUNTER__, FileId );
            mcu.GotoSleepSecond( 5 );
        } else {
            InsertTrace( __COUNTER__, FileId );
            mcu.GotoSleepSecond( AppTimeSleeping );
            InsertTrace( __COUNTER__, FileId );
        }
    }
}

/*!
 * \brief   TX shotgun application
 */

void TxShotgun_app( void ) {
    uint8_t UserPayloadSize;
    uint8_t UserPayload[255];
    uint32_t ToA;
    uint8_t CurrentSF;

    /* Prepare UserPayload and User parameters */
    UserPayloadSize = 14;
    for ( int i = 0; i < UserPayloadSize; i++ ) {
        UserPayload[i] = i;
    }
    UserPayload[0] = FW_VERSION;

    DEBUG_MSG("\n---------------------> Starting TxShotgun application <-------------------------------\n");

    mcu.AttachInterruptIn( &UserIsr );

    RadioUser->Reset();

    while ( 1 ) {
        /* Send packet */
        DEBUG_MSG("\n---------------------> Sending a NEW PACKET <-------------------------------\n");

        CurrentSF = 7;

        RadioUser->SendLora( &UserPayload[0], UserPayloadSize, CurrentSF, BW125, 866100000, 0 );
        ToA = TimeOnAir( UserPayloadSize, BW125, CurrentSF, 8, true, true, 1 );

        /* Wait for packet to be sent before sending the next one */
        mcu.mwait_ms( ToA );
        if ( CurrentSF == 12 ) {
            mcu.mwait_ms( 100 ); /* add some time before sending next packet, to avoid the GW to receive the next SF5 before this SF12 (processing time) */
        }

        /* Everything is OK so far... */
        mcu.WatchDogRelease( );
    }
}

/*!
 * \brief   TX shotgun application
 */

void Rx_app( void ) {
}

/*!
 * \brief   Utility functions
 */

uint32_t TimeOnAir( uint8_t pktLen, eBandWidth bandwidth, uint8_t Datarate, uint16_t PreambleLen, bool CrcOn, bool HeaderOn, uint8_t Coderate ) {
    uint32_t airTime = 0;
    double bw = 0.0;

    if (Datarate < 7) {
        Datarate = 7; /* TODO */
    }

    switch (bandwidth) {
        case BW125:
            bw = 125 * 1e3;
            break;
        case BW250:
            bw = 250 * 1e3;
            break;
        case BW500:
            bw = 500 * 1e3;
            break;
        default:
            break;
    }

    // Symbol rate : time for one symbol (secs)
    double rs = bw / ( 1 << Datarate );
    double ts = 1 / rs;
    // time of preamble
    double tPreamble = ( PreambleLen + 4.25 ) * ts;
    // Symbol length of payload and time
    double tmp = ceil( ( 8 * pktLen - 4 * Datarate +
                            28 + 16 * CrcOn -
                            ( ( HeaderOn == false ) ? 20 : 0 ) ) /
                            ( double )( 4 * ( Datarate -
                            ( ( Datarate > 10 ) ? 2 : 0 ) ) ) ) *
                            ( Coderate + 4 );
    double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return ms secs
    airTime = floor( tOnAir * 1e3 + 0.999 );

    return airTime;
}

void UserIsr( void ) {
    IrqFlags_t RegIrqFlag;

    RegIrqFlag = RadioUser->GetIrqFlagsLora( );
    RadioUser->ClearIrqFlagsLora( );

    switch ( RegIrqFlag ) {
        case SENT_PACKET_IRQ_FLAG:
            DEBUG_MSG("packet sent\n");
            break;

        case RECEIVE_PACKET_IRQ_FLAG:
            DEBUG_MSG("packet received\n");
            break;

        case RXTIMEOUT_IRQ_FLAG:
            DEBUG_MSG("RX timeout\n");
            break;

        case BAD_PACKET_IRQ_FLAG:
            DEBUG_MSG("Bad packet\n");
            break;

        default :
            DEBUG_PRINTF( "ERROR: radio ISR %x\n", RegIrqFlag );
            break;
    }
}
