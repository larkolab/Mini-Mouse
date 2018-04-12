/*

  __  __ _       _                                 
 |  \/  (_)     (_)                                
 | \  / |_ _ __  _ _ __ ___   ___  _   _ ___  ___  
 | |\/| | | '_ \| | '_ ` _ \ / _ \| | | / __|/ _ \
 | |  | | | | | | | | | | | | (_) | |_| \__ \  __/ 
 |_|  |_|_|_| |_|_|_| |_| |_|\___/ \__,_|___/\___| 
                                                   
                                                   
Description       : Mcu Api.  


License           : Revised BSD License, see LICENSE.TXT file include in the project

Maintainer        : Fabien Holin (SEMTECH)
*/
#ifndef MCUXX_H
#define MCUXX_H
#include "mbed.h"
#include "Define.h"
#include "ClassSTM32L4.h"


class McuXX {
public :    
     McuXX ( );
    ~McuXX ( );
     /** Int Mcu .  
     * 
     *  This method configure the mcu such as clock, gpio etc ...
     * 
     *  @param        void
     *  @return       void
     */ 
    void InitMcu ( void );

/******************************************************************************/
/*                                Mcu Flash Api                               */
/******************************************************************************/
     /** RestoreContext data from a flash device.  
     * 
     *  This method invokes memcpy - reads number of bytes from the address 
     * 
     *  @param buffer Buffer to write to 
     *  @param addr   Flash address to begin reading from 
     *  @param size   Size to read in bytes 
     *  @return       0 on success, negative error code on failure 
     */ 
    int RestoreContext(uint8_t *buffer, uint32_t addr, uint32_t size);

 
    /** StoreContext data to flash
     *  To be safer this function have to implement a read/check data sequence after programation 
     *  
     * 
     *  @param buffer Buffer of data to be written 
     *  @param addr   Flash Address to begin writing to,
     *  @param size   Size to write in bytes,
     *  @return       0 on success, negative error code on failure 
     */ 
    int StoreContext(const void *buffer, uint32_t addr, uint32_t size); 
    
/******************************************************************************/
/*                                Mcu RTC Api                                 */
/******************************************************************************/
    /*!
    * RtcInit Function
    * \remark must be called before any call to initiliaze the timers
    * \param [IN]   void
    * \param [OUT]  void       
    */
    void     RtcInit            ( void ) ;
    
    /*!
    * RtcGetTimeSecond : return the Current Rtc time in Second 
    * \remark is used for :
    * \remark scheduling autonomous retransmissions (for exemple NbTrans) , transmitting MAC answers , basically any delay without accurate time constraints
    * \remark also used to measure the time spent inside the LoRaWAN process for the integrated failsafe
    * \param [IN]   void
    * \param [OUT]  uint32_t RTC time in Second       
    */
    uint32_t RtcGetTimeSecond       ( void ) ;
   /*!
    * RtcGetTimeMs : return the Current Rtc time in Ms 
    * \remark is used to timestamp radio events (end of TX), will also be used for future classB
    * \remark this function may be used by the application.
    * \param [IN]   void
    * \param [OUT]  uint32_t Current RTC time in ms wraps every 49 days       
    */
    uint32_t RtcGetTimeMs  ( void ) ;
    
/******************************************************************************/
/*                                Mcu Sleep Api                               */
/******************************************************************************/
    /*!
    * A function to set the mcu in low power mode  for duration seconds
    * \remark inside this function watchdog has to be manage to not reset the mcu
    * \param [IN]   int delay 
    * \param [OUT]  void       
    */
    void GotoSleepSecond (int duration ) ;
    
        /*!
    * A function to set the mcu in low power mode  for duration in milliseconds
    * \remark 
    * \param [IN]   int delay 
    * \param [OUT]  void       
    */
    void     GotoSleepMSecond   ( int delay );
    
/******************************************************************************/
/*                             Mcu WatchDog Api                               */
/******************************************************************************/
    /* A function to init and start the Watchdog 
    * \remark The expired period = WATCH_DOG_PERIOD_RELEASE seconds
    * \param [IN]   void  
    * \param [OUT]  void       
    */
    void WatchDogStart ( void ) ;

    /* A function to release the Watchdog 
    * \remark Application have to call this function periodically (with a period <WATCH_DOG_PERIOD_RELEASE)
    *         If not , the mcu will reset.
    * \param [IN]   void  
    * \param [OUT]  void       
    */
    void WatchDogRelease ( void ) ;
    
    
/******************************************************************************/
/*                             Mcu WatchDog Api                               */
/******************************************************************************/
/*!
    * LowPowerTimerLoRa Init
    *\remark initializes the dedicated LoRaWAN low power timer object. MCU specific.
    * \param [IN]  void
    * \param [OUT] void         
    */
    void LowPowerTimerLoRaInit ( void );
    /*!
    * LowPowerTimerLoRa AttachMsecond
    *
    * \param void (* _Func) (void *) a static method member of the current Obj
    * \param *_obj a pointer to the current objet
    * \param int delay in ms delay should be between 1ms and 16s.
    * \param [OUT] void         
    * \remark the code  Func =  _Func ; and obj  = _obj; isn't mcu dependent , and could be keep as already implemented
    * \remark starts the LoRaWAN dedicated timer and attaches the IRQ to the handling Interupt Service Routine in the LoRaWAN object.
    */
    void StartTimerMsecond     ( void (* _Func) (void *) , void * _obj, int delay) ;
        
    /*!
    *  timerISR
    * \remark    Do Not Modify 
    */
    void timerISR              ( void ) { Func(obj); };
private :
    /*!
    *  Low power timer
    * \remark    Do Not Modify 
    */
    static void DoNothing (void *) { };
    void (* Func) (void *);
    void * obj;
};
extern McuSTM32L4 mcu;
#endif