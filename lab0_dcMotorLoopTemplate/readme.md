FreeRTOS modifications
======================

---

1. Download FreeRTOS. Current version: 10.5.0-0

2. Unzip into lib folder

3. Add flags to platformio.ini file

        build_flags =
            -Ilib/Arduino_FreeRTOS_Library/src


4. Let's change Watchdog by Timer1 to increase tick precision

    4.1. Modify  `FreeRTOSVariant.h` as follows

        #ifndef portUSE_TIMER1
        #define portUSE_TIMER1 TIMER1_10ms //TODO:check this and use it
        #endif

    4.2. Mofify `port.c` as follows

    In line 67:

        #elif defined( portUSE_TIMER1 )
        ...
        #define portSCHEDULER_ISR TIMER1_COMPA_vect
        ...

    In line 663:

        void vPortEndScheduler( void )
        ...
        TIMSK1 &= ~(1 << OCIE1A); // turn off the timer interrupt
        ...

    In line 810:

        #elif defined( portUSE_TIMER1 )
        ...
        static void prvSetupTimerInterrupt( void ) 
        {
          OCR1A = 19999;//the Tick time based on clock frequency, prescaler, etc.
          ...
        }

5. Note that tick is defined in `portmacro.h`
        
        #define portTICK_PERIOD_MS ( (TickType_t) 1000 / configTICK_RATE_HZ )
   
   And  `configTICK_RATE_HZ` comes from line 59  in `FreeRTOSConfig.h`
        
        #define configTICK_RATE_HZ 100//STR 1000

   This must agree with the configuration in `prvSetupTimerInterrupt`

   TODO: Based on `#define portUSE_TIMER1 TIMER1_10ms` develop all the configurations

6. Add `str_compute` to waste some computing time without the need for implementing custom `delays`

        void str_compute(unsigned long milliseconds)
        {
                unsigned int i = 0;
                unsigned int imax = 0;
                imax = milliseconds * 92;
                volatile float dummy = 1;
                for (i = 0; i < imax; i++)
                {
                        dummy = dummy * dummy;
                }
        }

   Note that faster/slower microcontrollers will require a different implementation

7. Add `str_getTime` to get the current time based on timer1 registers
   
   For example:

        float str_getTime(void)
        {
          float t=(float)(0.5e-3*((float)OCR1A*xTaskGetTickCount()+TCNT1));
          return t;
        }

   Note that the 0.5e-3 converts from cpu clocks to milliseconds

8. Add `str_trace` to get info and task states into circular buffers
   
   8.1 Add trace into the available hook provided in `Arduino_FreeRTOS.h` line 348

        extern void str_trace(void);
        #ifndef traceTASK_SWITCHED_IN
          #define traceTASK_SWITCHED_IN() str_trace()
        #endif

   8.2 Modify `eTaskGetState` in `Arduino_FreeRTOS.h` to allow tracing task states
    
        #ifndef INCLUDE_eTaskGetState
            #define INCLUDE_eTaskGetState   1    
        #endif

   8.3. Modify `FreeRTOSConfig.h` to allow more than 4 different priorities

        #define configMAX_PRIORITIES 10 //STR 4

   8.4. Configure 4 tasks `Task1` ... `Task4` appropriately

        xTaskCreate(Task1," Task1", configMINIMAL_STACK_SIZE, NULL, 5, &Task1Handle);
        xTaskCreate(Task2, "Task2", configMINIMAL_STACK_SIZE, NULL, 4, &Task2Handle);
        xTaskCreate(Task3, "Task3", configMINIMAL_STACK_SIZE, NULL, 3, &Task3Handle);
        xTaskCreate(Task4, "Task4", configMINIMAL_STACK_SIZE, NULL, 2, &Task4Handle);
        
   Note that the third parameter is the stack size which must be sized properly

   8.5. Tracing requires taksHandler in each task. Added at the beginning of main and used in the last parameter of the `xTaskCreate`

        TaskHandle_t Task1Handle;

   8.6. Tracing is used inside a circular buffer to keep track of whatever happens

   At the beginning:

        #define BUFF_SIZE 300
        float t[BUFF_SIZE] = {};
        char circ_buffer1[BUFF_SIZE] = {};
        char circ_buffer2[BUFF_SIZE] = {};
        ...
        
   At run-time:

        t[circ_buffer_counter ] = str_getTime();
        circ_buffer1[circ_buffer_counter] = eTaskGetState (Task1Handle);
        circ_buffer2[circ_buffer_counter] = eTaskGetState (Task2Handle);
        ...

      

9. Implement a software timer from the freeRTOS library to stop the kernel and send data to the host PC

        TimerHandle_t  xOneShotTimer;
        BaseType_t xOneShotStarted;
        xOneShotTimer=xTimerCreate("OneShotTimer",pdMS_TO_TICKS(TSTOP),pdFALSE,0,OneShotTimerCallback);
        xOneShotStarted = xTimerStart( xOneShotTimer, 0 );
        ...
        void OneShotTimerCallback ( TimerHandle_t xTimer)
        {
                vTaskSuspendAll();
                ...            
        }

   Note that the software timer is used to stop the kernel and send data from the circular buffer to serial

        for (i = 0; i < BUFF_SIZE; i++)
        {
	        Serial.println("DAT");
	        Serial.print((float)t[i]);
	        Serial.print(",");
	        Serial.write((uint8_t)circ_buffer1[i]);
	        Serial.print(",");
	        ...
        }




