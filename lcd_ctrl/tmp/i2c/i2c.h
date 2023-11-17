/*
    2  * Copyright (c) 2015-2017, Texas Instruments Incorporated
    3  * All rights reserved.
    4  *
    5  * Redistribution and use in source and binary forms, with or without
    6  * modification, are permitted provided that the following conditions
    7  * are met:
    8  *
    9  * *  Redistributions of source code must retain the above copyright
   10  *    notice, this list of conditions and the following disclaimer.
   11  *
   12  * *  Redistributions in binary form must reproduce the above copyright
   13  *    notice, this list of conditions and the following disclaimer in the
   14  *    documentation and/or other materials provided with the distribution.
   15  *
   16  * *  Neither the name of Texas Instruments Incorporated nor the names of
   17  *    its contributors may be used to endorse or promote products derived
   18  *    from this software without specific prior written permission.
   19  *
   20  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   21  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
   22  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   23  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   24  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   25  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   26  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   27  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   28  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   29  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   30  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   31  */
   32 /*!*****************************************************************************
   33  *  @file       I2C.h
   34  *
   35  *  @brief      I2C driver interface
   36  *
   37  *  The I2C driver interface provides device independent APIs, data types,
   38  *  and macros. The I2C header file should be included in an application as
   39  *  follows:
   40  *  @code
   41  *  #include <ti/drivers/I2C.h>
   42  *  @endcode
   43  *
   44  *  # Overview #
   45  *  This section assumes that you have background knowledge and understanding
   46  *  about how the I2C protocol operates. For the full I2C specifications and
   47  *  user manual (UM10204), see the NXP Semiconductors website.
   48  *
   49  *  The I2C driver has been designed to operate as a single I2C master by
   50  *  performing I2C transactions between the target and I2C slave peripherals.
   51  *  The I2C driver does not support I2C slave mode.
   52  *  I2C is a communication protocol - the specifications define how data
   53  *  transactions are to occur via the I2C bus. The specifications do not
   54  *  define how data is to be formatted or handled, allowing for flexible
   55  *  implementations across different peripheral vendors. As a result, the
   56  *  I2C handles only the exchange of data (or transactions) between master
   57  *  and slaves. It is the left to the application to interpret and
   58  *  manipulate the contents of each specific I2C peripheral.
   59  *
   60  *  The I2C driver has been designed to operate in an RTOS environment.  It
   61  *  protects its transactions with OS primitives supplied by the underlying
   62  *  RTOS.
   63  *
   64  *  # Usage #
   65  *
   66  *  The I2C driver includes the following APIs:
   67  *    - I2C_init(): Initialize the I2C driver.
   68  *    - I2C_Params_init():  Initialize an #I2C_Params structure with default
   69  *      vaules.
   70  *    - I2C_open():  Open an instance of the I2C driver.
   71  *    - I2C_control():  Performs implemenation-specific features on a given
   72  *      I2C peripheral.
   73  *    - I2C_transfer():  Transfer the data.
   74  *    - I2C_close():  De-initialize the I2C instance.
   75  *
   76  *
   77  *  ### I2C Driver Configuration #
   78  *
   79  *  In order to use the I2C APIs, the application is required
   80  *  to provide device-specific I2C configuration in the Board.c file.
   81  *  The I2C driver interface defines a configuration data structure:
   82  *
   83  *  @code
   84  *  typedef struct I2C_Config_ {
   85  *      I2C_FxnTable  const    *fxnTablePtr;
   86  *      void                   *object;
   87  *      void          const    *hwAttrs;
   88  *  } I2C_Config;
   89  *  @endcode
   90  *
   91  *  The application must declare an array of I2C_Config elements, named
   92  *  I2C_config[].  Each element of I2C_config[] must be populated with
   93  *  pointers to a device specific I2C driver implementation's function
   94  *  table, driver object, and hardware attributes.  The hardware attributes
   95  *  define properties such as the I2C peripheral's base address and
   96  *  pins.  Each element in I2C_config[] corresponds to
   97  *  an I2C instance, and none of the elements should have NULL pointers.
   98  *  There is no correlation between the index and the
   99  *  peripheral designation (such as I2C0 or I2C1).  For example, it is
  100  *  possible to use I2C_config[0] for I2C1.
  101  *
  102  *  Because the I2C configuration is very device dependent, you will need to
  103  *  check the doxygen for the device specific I2C implementation.  There you
  104  *  will find a description of the I2C hardware attributes.  Please also
  105  *  refer to the Board.c file of any of your examples to see the I2C
  106  *  configuration.
  107  *
  108  *  ### Initializing the I2C Driver #
  109  *
  110  *  I2C_init() must be called before any other I2C APIs.  This function
  111  *  iterates through the elements of the I2C_config[] array, calling
  112  *  the element's device implementation I2C initialization function.
  113  *
  114  *  ### I2C Parameters
  115  *
  116  *  The #I2C_Params structure is passed to the I2C_open() call.  If NULL
  117  *  is passed for the parameters, I2C_open() uses default parameters.
  118  *  An #I2C_Params structure is initialized with default values by passing
  119  *  it to I2C_Params_init().
  120  *  Some of the I2C parameters are described below.  To see brief descriptions
  121  *  of all the parameters, see #I2C_Params.
  122  *
  123  *  #### I2C Transfer Mode
  124  *  The I2C driver supports two transfer modes of operation: blocking and
  125  *  callback:
  126  *  - #I2C_MODE_BLOCKING: The call to I2C_transfer() blocks until the
  127  *    transfer completes.
  128  *  - #I2C_MODE_CALLBACK: The call to I2C_transfer() returns immediately.
  129  *    When the transfer completes, the I2C driver will call a user-
  130  *    specified callback function.
  131  *
  132  *  The transfer mode is determined by the #I2C_Params.transferMode parameter
  133  *  passed to I2C_open().  The I2C driver defaults to blocking mode, if the
  134  *  application does not set it.
  135  *
  136  *  In blocking mode, a task calling I2C_transfer() is blocked until the
  137  *  transaction completes.  Other tasks requesting I2C transactions while
  138  *  a transaction is currently taking place, are also placed into a
  139  *  blocked state.
  140  *
  141  *  In callback mode, an I2C_transfer() functions asynchronously, which
  142  *  means that it does not block a calling task's execution.  In this
  143  *  mode, the user must set #I2C_Params.transferCallbackFxn to a user-
  144  *  provided callback function. After an I2C transaction has completed,
  145  *  the I2C driver calls the user- provided callback function.
  146  *  If another I2C transaction is requested, the transaction is queued up.
  147  *  As each transfer completes, the I2C driver will call the user-specified
  148  *  callback function.  The user callback will be called from either hardware
  149  *  or software interrupt context, depending upon the device implementation.
  150  *
  151  *  Once an I2C driver instance is opened, the
  152  *  only way to change the transfer mode is to close and re-open the I2C
  153  *  instance with the new transfer mode.
  154  *
  155  *  #### Specifying an I2C Bus Frequency
  156  *  The I2C controller's bus frequency is determined by #I2C_Params.bitRate
  157  *  passed to I2C_open().  The standard I2C bus frequencies are 100 kHz and
  158  *  400 kHz, with 100 kHz being the default.
  159  *
  160  *  ### Opening the I2C Driver #
  161  *  After initializing the I2C driver by calling I2C_init(), the application
  162  *  can open an I2C instance by calling I2C_open().  This function
  163  *  takes an index into the I2C_config[] array and an I2C parameters data
  164  *  structure.   The I2C instance is specified by the index of the I2C in
  165  *  I2C_config[].  Only one I2C index can be used at a time;
  166  *  calling I2C_open() a second time with the same index previosly
  167  *  passed to I2C_open() will result in an error.  You can,
  168  *  though, re-use the index if the instance is closed via I2C_close().
  169  *
  170  *  If no I2C_Params structure is passed to I2C_open(), default values are
  171  *  used. If the open call is successful, it returns a non-NULL value.
  172  *
  173  *  Example opening an I2C driver instance in blocking mode:
  174  *  @code
  175  *  I2C_Handle i2c;
  176  *
  177  *  // NULL params are used, so default to blocking mode, 100 KHz
  178  *  i2c = I2C_open(Board_I2C0, NULL);
  179  *
  180  *  if (!i2c) {
  181  *      // Error opening the I2C
  182  *  }
  183  *  @endcode
  184  *
  185  *  Example opening an I2C driver instance in callback mode and 400KHz bit rate:
  186  *
  187  *  @code
  188  *  I2C_Handle i2c;
  189  *  I2C_Params params;
  190  *
  191  *  I2C_Params_init(&params);
  192  *  params.transferMode  = I2C_MODE_CALLBACK;
  193  *  params.transferCallbackFxn = myCallbackFunction;
  194  *  params.bitRate  = I2C_400kHz;
  195  *
  196  *  handle = I2C_open(Board_I2C0, &params);
  197  *  if (!i2c) {
  198  *      // Error opening I2C
  199  *  }
  200  *  @endcode
  201  *
  202  *  ### Transferring data #
  203  *  An I2C transaction with an I2C peripheral is started by calling
  204  *  I2C_transfer().  Three types of transactions are supported: Write, Read,
  205  *  or Write/Read. Each transfer is completed before another transfer is
  206  *  initiated.
  207  *
  208  *  For Write/Read transactions, the specified data is first written to the
  209  *  peripheral, then a repeated start is sent by the driver, which initiates
  210  *  the read operation.  This type of transfer is useful if an I2C peripheral
  211  *  has a pointer register that needs to be adjusted prior to reading from
  212  *  the referenced data register.
  213  *
  214  *  The details of each transaction are specified with an #I2C_Transaction data
  215  *  structure. This structure defines the slave I2C address, pointers
  216  *  to write and read buffers, and their associated byte counts. If
  217  *  no data needs to be written or read, the corresponding byte counts should
  218  *  be set to zero.
  219  *
  220  *  If an I2C transaction is requested while a transaction is currently
  221  *  taking place, the new transaction is placed onto a queue to be processed
  222  *  in the order in which it was received.
  223  *
  224  *  The below example shows sending three bytes of data to a slave peripheral
  225  *  at address 0x50, in blocking mode:
  226  *
  227  *  @code
  228  *  unsigned char writeBuffer[3];
  229  *  I2C_Transaction i2cTransaction;
  230  *
  231  *  i2cTransaction.slaveAddress = 0x50;
  232  *  i2cTransaction.writeBuf = writeBuffer;
  233  *  i2cTransaction.writeCount = 3;
  234  *  i2cTransaction.readBuf = NULL;
  235  *  i2cTransaction.readCount = 0;
  236  *
  237  *  status = I2C_transfer(i2c, &i2cTransaction);
  238  *  if (!status) {
  239  *      // Unsuccessful I2C transfer
  240  *  }
  241  *  @endcode
  242  *
  243  *  The next example shows reading of five bytes of data from the I2C
  244  *  peripheral, also in blocking mode:
  245  *
  246  *  @code
  247  *  unsigned char readBuffer[5];
  248  *  I2C_Transaction i2cTransaction;
  249  *
  250  *  i2cTransaction.slaveAddress = 0x50;
  251  *  i2cTransaction.writeBuf = NULL;
  252  *  i2cTransaction.writeCount = 0;
  253  *  i2cTransaction.readBuf = readBuffer;
  254  *  i2cTransaction.readCount = 5;
  255  *
  256  *  status = I2C_transfer(i2c, &i2cTransaction);
  257  *  if (!status) {
  258  *      // Unsuccessful I2C transfer
  259  *  }
  260  *  @endcode
  261  *
  262  *  This example shows writing of two bytes and reading of four bytes in a
  263  *  single transaction.
  264  *
  265  *  @code
  266  *  unsigned char readBuffer[4];
  267  *  unsigned char writeBuffer[2];
  268  *  I2C_Transaction i2cTransaction;
  269  *
  270  *  i2cTransaction.slaveAddress = 0x50;
  271  *  i2cTransaction.writeBuf = writeBuffer;
  272  *  i2cTransaction.writeCount = 2;
  273  *  i2cTransaction.readBuf = readBuffer;
  274  *  i2cTransaction.readCount = 4;
  275  *
  276  *  status = I2C_transfer(i2c, &i2cTransaction);
  277  *  if (!status) {
  278  *      // Unsuccessful I2C transfer
  279  *  }
  280  *  @endcode
  281  *
  282  *  This final example shows usage of asynchronous callback mode, with queuing
  283  *  of multiple transactions.  Because multiple transactions are simultaneously
  284  *  queued, separate I2C_Transaction structures must be used.  (This is a
  285  *  general rule, that I2C_Transaction structures cannot be reused until
  286  *  it is known that the previous transaction has completed.)
  287  *
  288  *  First, for the callback function (that is specified in the I2C_open() call)
  289  *  the "arg" in the I2C_Transaction structure is a semaphore handle. When
  290  *  this value is non-NULL, sem_post() is called in the callback using
  291  *  the specified handle, to signal completion to the task that queued the
  292  *  transactions:
  293  *
  294  *  @code
  295  *  Void callbackFxn(I2C_Handle handle, I2C_Transaction *msg, Bool transfer) {
  296  *      if (msg->arg != NULL) {
  297  *          sem_post((sem_t *)(msg->arg));
  298  *      }
  299  *  }
  300  *  @endcode
  301  *
  302  *  Snippets of the task code that initiates the transactions are shown below.
  303  *  Note the use of multiple I2C_Transaction structures, and passing of the
  304  *  handle of the semaphore to be posted via i2cTransaction2.arg.
  305  *  I2C_transfer() is called three times to initiate each transaction.
  306  *  Since callback mode is used, these functions return immediately.  After
  307  *  the transactions have been queued, other work can be done, and then
  308  *  eventually sem_wait() is called to wait for the last I2C
  309  *  transaction to complete.  Once the callback posts the semaphore the task
  310  *  will be moved to the ready state, so the task can resume execution.
  311  *
  312  *  @code
  313  *  Void taskfxn(arg0, arg1) {
  314  *
  315  *      I2C_Transaction i2cTransaction0;
  316  *      I2C_Transaction i2cTransaction1;
  317  *      I2C_Transaction i2cTransaction2;
  318  *
  319  *      ...
  320  *      i2cTransaction0.arg = NULL;
  321  *      i2cTransaction1.arg = NULL;
  322  *      i2cTransaction2.arg = semaphoreHandle;
  323  *
  324  *      ...
  325  *      I2C_transfer(i2c, &i2cTransaction0);
  326  *      I2C_transfer(i2c, &i2cTransaction1);
  327  *      I2C_transfer(i2c, &i2cTransaction2);
  328  *
  329  *      ...
  330  *
  331  *      sem_wait(semaphoreHandle);
  332  *
  333  *      ...
  334  *  }
  335  *  @endcode
  336  *
  337  *  # Implementation #
  338  *
  339  *  This top-level I2C module serves as the main interface for RTOS
  340  *  applications. Its purpose is to redirect the module's APIs to specific
  341  *  peripheral implementations which are specified using a pointer to an
  342  *  #I2C_FxnTable.
  343  *
  344  *  The I2C driver interface module is joined (at link time) to an
  345  *  array of I2C_Config data structures named *I2C_config*.
  346  *  *I2C_config* is typically defined in the Board.c file used for the
  347  *  application.  If there are multiple instances of I2C peripherals on the
  348  *  device, there will typically be multiple I2C_Config structures defined in
  349  *  the board file. Each entry in *I2C_config* contains a:
  350  *  - (I2C_FxnTable *) to a set of functions that implement a I2C peripheral
  351  *  - (void *) data object that is associated with the I2C_FxnTable
  352  *  - (void *) hardware attributes that are associated to the I2C_FxnTable
  353  *
  354  *******************************************************************************
  355  */
  356 
  357 #ifndef ti_drivers_I2C__include
  358 #define ti_drivers_I2C__include
  359 
  360 #ifdef __cplusplus
  361 extern "C" {
  362 #endif
  363 
  364 #include <stdint.h>
  365 #include <stdbool.h>
  366 #include <stddef.h>
  367 
  385 #define I2C_CMD_RESERVED           (32)
  386 
  399 #define I2C_STATUS_RESERVED        (-32)
  400 
  414 #define I2C_STATUS_SUCCESS         (0)
  415 
  422 #define I2C_STATUS_ERROR           (-1)
  423 
  431 #define I2C_STATUS_UNDEFINEDCMD    (-2)
  432 
  442 /* Add I2C_CMD_<commands> here */
  443 
  451 typedef struct I2C_Config_    *I2C_Handle;
  452 
  464 typedef struct I2C_Transaction_ {
  465     void    *writeBuf;    
  466     size_t   writeCount;  
  468     void    *readBuf;     
  469     size_t   readCount;   
  471     uint_least8_t slaveAddress; 
  473     void    *arg;         
  474     void    *nextPtr;     
  475 } I2C_Transaction;
  476 
  485 typedef enum I2C_TransferMode_ {
  486     I2C_MODE_BLOCKING,  
  487     I2C_MODE_CALLBACK   
  488 } I2C_TransferMode;
  489 
  506 typedef void (*I2C_CallbackFxn)(I2C_Handle handle, I2C_Transaction *transaction,
  507     bool transferStatus);
  508 
  515 typedef enum I2C_BitRate_ {
  516     I2C_100kHz = 0,
  517     I2C_400kHz = 1
  518 } I2C_BitRate;
  519 
  542 typedef struct I2C_Params_ {
  543     I2C_TransferMode transferMode;        
  544     I2C_CallbackFxn  transferCallbackFxn; 
  545     I2C_BitRate      bitRate;             
  546     void            *custom;              
  548 } I2C_Params;
  549 
  554 typedef void (*I2C_CancelFxn) (I2C_Handle handle);
  555 
  560 typedef void (*I2C_CloseFxn) (I2C_Handle handle);
  561 
  566 typedef int_fast16_t (*I2C_ControlFxn) (I2C_Handle handle, uint_fast16_t cmd,
  567     void *controlArg);
  568 
  573 typedef void (*I2C_InitFxn) (I2C_Handle handle);
  574 
  579 typedef I2C_Handle (*I2C_OpenFxn) (I2C_Handle handle, I2C_Params *params);
  580 
  585 typedef bool (*I2C_TransferFxn) (I2C_Handle handle,
  586     I2C_Transaction *transaction);
  587 
  593 typedef struct I2C_FxnTable_ {
  595     I2C_CancelFxn   cancelFxn;
  596 
  598     I2C_CloseFxn    closeFxn;
  599 
  601     I2C_ControlFxn  controlFxn;
  602 
  604     I2C_InitFxn     initFxn;
  605 
  607     I2C_OpenFxn     openFxn;
  608 
  610     I2C_TransferFxn transferFxn;
  611 } I2C_FxnTable;
  612 
  624 typedef struct I2C_Config_ {
  626     I2C_FxnTable const *fxnTablePtr;
  627 
  629     void               *object;
  630 
  632     void         const *hwAttrs;
  633 } I2C_Config;
  634 
  655 extern void I2C_cancel(I2C_Handle handle);
  656 
  666 extern void I2C_close(I2C_Handle handle);
  667 
  705 extern int_fast16_t I2C_control(I2C_Handle handle, uint_fast16_t cmd,
  706     void *controlArg);
  707 
  716 extern void I2C_init(void);
  717 
  739 extern I2C_Handle I2C_open(uint_least8_t index, I2C_Params *params);
  740 
  752 extern void I2C_Params_init(I2C_Params *params);
  753 
  805 extern bool I2C_transfer(I2C_Handle handle, I2C_Transaction *transaction);
  806 
  807 #ifdef __cplusplus
  808 }
  809 #endif
  810 
  811 #endif /* ti_drivers_I2C__include */