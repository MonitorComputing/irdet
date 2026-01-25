;**********************************************************************
;                                                                     *
;    Description:   'IrDetector' Microchip PIC assembler model        *
;                   railway Infra-Red train detector.                 *
;                                                                     *
;    Author:        Chris White (whitecf@bcs.org.uk)                  *
;    Company:       Monitor Computing Services Ltd.                   *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Copyright (c)  2021 Monitor Computing Services Ltd               *
;    Unpublished and not for publication                              *
;    All rights reserved                                              *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes: High frequency pulsed Infra-Red reflective train detector *
;           utilising PIC monitor interface via full duplex serial    *
;           @ 4K8 baud, 1 start bit, 8 data bits, no parity,          *
;             1 stop bit                                              *
;                                                                     *
;           'Up' interface, asynchronous serial @ 4K8 baud            *
;           Port B 0 - Rx, 1 - Tx                                     *
;                                                                     *
;                            +---+ +---+                              *
;            !Emitter  <- RA2|1  |_| 18|RA1                           *
;               Sensor -> RA3|2      17|RA0                           *
;            !Detecting < RA4|3      16|                              *
;                            |4      15|                              *
;                            |5      14|                              *
;                   Rx -> RB0|6      13|RB7                           *
;                   Tx <- RB1|7      12|RB6                           *
;                         RB2|8      11|RB5                           *
;                         RB3|9      10|RB4                           *
;                            +---------+                              *
;                                                                     *
;**********************************************************************


;**********************************************************************
; Include and configuration directives                                *
;**********************************************************************

    list      r=dec,p=16f84

#include <p16f84.inc>

    __CONFIG   _CP_OFF & _WDT_OFF & _PWRTE_ON & _XT_OSC

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.

; Include serial interface macros
#include "utility_pic/asyn_srl.inc"


;**********************************************************************
;**************************** System code *****************************
;**********************************************************************


;**********************************************************************
; Constant definitions                                                *
;**********************************************************************

; I/O port direction it masks
PORTASTATUS EQU     B'00001000'
PORTBSTATUS EQU     B'00000001'

; Interrupt & timing constants
RTCCINT     EQU     158         ; 10KHz = (1MHz / 100) - RTCC write inhibit (2)

; Bit timing for 4K8 (actually 5K) baud
INT5KBIT    EQU     2           ; Interrupts per serial bit
INT5KINI    EQU     3           ; Interrupts per initial Rx serial bit

; Monitor interface constants
;  Serial I/F status flags
;   bit 0 - Rx buffer full
;   bit 1 - Rx error
;   bit 2 - Received break
;   bit 3 - Seeking stop bit
;   bit 4 - Tx buffer clear
;   bit 5 - Sending break
RXMFLAG     EQU     0           ; Receive byte buffer 'loaded' status bit
RXMERR      EQU     1           ; Receive error status bit
RXMBREAK    EQU     2           ; Received 'break' status bit
RXMSTOP     EQU     3           ; Seeking stop bit
TXMFLAG     EQU     4           ; Transmit byte buffer 'clear' status bit
TXMBREAK    EQU     5           ; Send 'break' status bit

TXMTRIS     EQU     TRISB       ; Tx port direction register
TXMPORT     EQU     PORTB       ; Tx port data register
TXMBIT      EQU     1           ; Tx output bit
RXMTRIS     EQU     TRISB       ; Rx port direction register
RXMPORT     EQU     PORTB       ; Rx port data register
RXMBIT      EQU     0           ; Rx input bit


;**********************************************************************
; Variable registers                                                  *
;**********************************************************************

            CBLOCK  0x0C

; Status and accumulator storage during interrupt
w_isr           ; 'w' register, accumulator, store during ISR
pclath_isr      ; PCLATH register store during ISR
status_isr      ; status register store during ISR

; Serial interface
srlIfStat       ; Serial I/F status flags

; Monitor interface
serMRxTmr       ; Interrupt counter for serial bit timing
serMRxReg       ; Data shift register
serMRxByt       ; Data byte buffer
serMRxBitCnt    ; Bit down counter
serMTxTmr       ; Interrupt counter for serial bit timing
serMTxReg       ; Data shift register
serMTxByt       ; Data byte buffer
serMTxBitCnt    ; Bit down counter

            ENDC


;**********************************************************************
; EEPROM initialisation                                               *
;**********************************************************************

            ORG     0x2100  ; EEPROM data area

; None


;**********************************************************************
; Reset vector                                                        *
;**********************************************************************

            ORG     0x000   ; Processor reset vector

BootVector
    clrf    INTCON          ; Disable interrupts
    clrf    INTCON          ; Ensure interrupts are disabled
    goto    Boot            ; Jump to beginning of program


;**********************************************************************
; Interrupt vector                                                    *
;**********************************************************************

            ORG     0x004   ; Interrupt vector location

IntVector
    movwf   w_isr           ; Save off current W register contents
    swapf   STATUS,W        ; Swap status register into W register
    BANKSEL TMR0            ; Ensure register page 0 is selected
    movwf   status_isr      ; save off contents of STATUS register
    movf    PCLATH,W        ; Move PCLATH register into W register
    movwf   pclath_isr      ; save off contents of PCLATH register
    movlw   high BeginISR   ; Load ISR address high byte ...
    movwf   PCLATH          ; ... into PCLATH to set code block
    goto    BeginISR        ; Jump to interrupt service routine


;**********************************************************************
; Instance Monitor interface routine macros                           *
;**********************************************************************

EnableMRx   EnableRx  RXMTRIS, RXMPORT, RXMBIT
    return


InitMRx     InitRx  srlIfStat, serMRxTmr, serMRxBitCnt, serMRxReg, RXMFLAG, RXMERR, RXMBREAK, RXMSTOP
    return


SrvcMRx     ServiceRx  srlIfStat, serMRxTmr, serMRxBitCnt, serMRxReg, serMRxByt, RXMPORT, RXMBIT, INT5KINI, INT5KBIT, RXMERR, RXMBREAK, RXMSTOP, RXMFLAG


EnableMTx   EnableTx  TXMTRIS, TXMPORT, TXMBIT
    return


InitMTx     InitTx  srlIfStat, serMTxTmr, serMTxBitCnt, serMTxReg, TXMFLAG, TXMBREAK
    return


SrvcMTx     ServiceTx  srlIfStat, serMTxTmr, serMTxBitCnt, serMTxReg, serMTxByt, TXMPORT, TXMBIT, RXMPORT, RXMBIT, INT5KBIT, TXMFLAG, TXMBREAK


LinkMRx
SerMRx      SerialRx srlIfStat, serMRxByt, RXMFLAG


LinkMTx
SerMTx      SerialTx srlIfStat, serMTxByt, TXMFLAG


;**********************************************************************
; Configure and include Monitor macros                                *
;**********************************************************************

; Set defines to configure Monitor macros
#define GOTUSERBANNER   ; Display 'user' banner
#define MONUSERON       ; Run user code immediately when booted

; Include Monitor macros
#include "utility_pic/eeprom.inc"
#include "utility_pic/monitor.inc"


;**********************************************************************
; Main program initialisation code                                    *
;**********************************************************************

Boot
    ; Clear I/O ports
    clrf    PORTA
    clrf    PORTB

    BANKSEL OPTION_REG

    ; Initialise I/O port bit directions
    movlw   PORTASTATUS
    movwf   TRISA
    movlw   PORTBSTATUS
    movwf   TRISB

    ; Set option register:
    ;   Port B pull-up       - off
    ;   Prescaler assignment - watchdog timer
    clrf    OPTION_REG
    bsf     OPTION_REG,NOT_RBPU
    bsf     OPTION_REG,PSA

    BANKSEL TMR0

    movlw   PORTASTATUS     ; For Port A need to write one to each bit ...
    movwf   PORTA           ; ... being used for input

    ; Initialise Monitor terminal serial link
    call    EnableMRx       ; Enable receive from Monitor terminal
    call    InitMRx         ; Initialise receiver for Monitor terminal
    call    EnableMTx       ; Enable transmit to Monitor terminal
    call    InitMTx         ; Initialise transmitter to Monitor terminal

    call    UserInit        ; Run user initialisation code

    ; Initialise interrupts
    movlw   low RTCCINT
    movwf   TMR0            ; Initialise RTCC for timer interrupts
    clrf    INTCON          ; Disable all interrupt sources
    bsf     INTCON,T0IE     ; Enable RTCC interrupts
    bsf     INTCON,GIE      ; Enable interrupts


    goto    MonitorMain     ; Run Monitor program


;**********************************************************************
; Interrupt service routine (ISR) code                                *
;**********************************************************************

BeginISR
    btfss   INTCON,T0IF     ; Test for RTCC Interrupt
    retfie                  ; If not, skip service routine

    ; Re-enable the timer interrupt and reload the timer
    bcf     INTCON,T0IF     ; Reset the RTCC Interrupt bit
    movlw   low RTCCINT
    addwf   TMR0,F          ; Reload RTCC

    call    SrvcMRx         ; Perform Monitor interface Rx service
    call    SrvcMTx         ; Perform Monitor interface Tx service

    ; Instance the Monitor interrupt service macro
    MonitorISR

EndISR
    movf    pclath_isr,W    ; Retrieve copy of PCLATH register
    movwf   PCLATH          ; Restore pre-isr PCLATH register contents
    swapf   status_isr,W    ; Swap copy of STATUS register into W register
    movwf   STATUS          ; Restore pre-isr STATUS register contents
    swapf   w_isr,F         ; Swap pre-isr W register value nibbles
    swapf   w_isr,W         ; Swap pre-isr W register into W register

    retfie                  ; return from Interrupt


;**********************************************************************
;***************************** User code ******************************
;**********************************************************************


;**********************************************************************
; User constants                                                      *
;**********************************************************************

; I/O constants
EMITTERPORT     EQU     PORTA       ; Emitter drive port
EMITTERBIT      EQU     2           ; Emmitter drive bit (active low)
SENSORPORT      EQU     PORTA       ; Sensor input port
SENSORBIT       EQU     3           ; Sensor input bit (active high)
INDICATORPORT   EQU     PORTA       ; Detection indicator port
INDICATORBIT    EQU     4           ; Detection indicator bit (active low)

; Detection correspondance 'high water' and 'low water' thresholds
DETHIGHWTR      EQU     200         ; Detection "On" threshold
DETLOWWTR       EQU     55          ; Detection "Off" threshold

; Sensor transition determination values
SENSORMASK      EQU     B'00000011' ; Mask to isolate transition bits
SENSORH2L       EQU     B'00000010' ; Transition from high (on) to low (off)
SENSORL2H       EQU     B'00000001' ; Transition from low (off) to high (on)


;**********************************************************************
; User variables                                                      *
;**********************************************************************

            CBLOCK

detAcc          ; Detection correspondance accumulator
sensorStore     ; Store sensor values to determine on/off transitions

            ENDC


;**********************************************************************
; User banner display code                                            *
;**********************************************************************

UserBanner
    movlw   crCode
    call    TxLoop
    movlw   lfCode
    call    TxLoop
    movlw   'I'
    call    TxLoop
    movlw   'R'
    call    TxLoop
    movlw   ' '
    call    TxLoop
    movlw   'D'
    call    TxLoop
    movlw   'e'
    call    TxLoop
    movlw   't'
    call    TxLoop
    movlw   'e'
    call    TxLoop
    movlw   'c'
    call    TxLoop
    movlw   't'
    call    TxLoop
    movlw   'o'
    call    TxLoop
    movlw   'r'
    call    TxLoop
    movlw   ' '
    call    TxLoop
    movlw   '1'
    call    TxLoop
    movlw   'a'
    call    TxLoop
    movlw   crCode
    call    TxLoop
    movlw   lfCode

TxLoop
    movwf   FSR             ; Copy W to FSR, for sending
LoopTx
    call    LinkMTx         ; Try to send data
    btfss   STATUS,Z        ; Skip if data sent ...
    goto    LoopTx          ; ... otherwise keep trying to send

    return


;**********************************************************************
; User initialisation code                                            *
;**********************************************************************

UserInit

    clrf    detAcc          ; Clear detection correspondance accumulator

    bsf     EMITTERPORT,EMITTERBIT      ; Ensure emmitter is off
    bsf     INDICATORPORT,INDICATORBIT  ; Ensure detection indicator is off

    return


;**********************************************************************
; User interrupt service routine (ISR) code                           *
;**********************************************************************

UserInt

    ; Add the current state of the sensor to the stored previous states
    bcf     STATUS,C                ; Clear the carry flag
    btfsc   SENSORPORT,SENSORBIT    ; Test if sensor is off ...
    bsf     STATUS,C                ; ... else set the carry flag
    rlf     sensorStore,F           ; Rotate the sensor state into the store

    ; Test if the emitter is on or off, toggle the emitter state, then
    ; check the sensor for a corresponding transition.

    btfsc   EMITTERPORT,EMITTERBIT  ; Test current state of emitter ...
    goto    EmitterIsOff            ; ... jump if off, else ...

EmitterIsOn
    ; Toggle the present emitter state
    bsf     EMITTERPORT,EMITTERBIT  ; Turn emitter off

    ; Test for sensor transition from off to on,
    ; indicating presence of a reflective target

    movf    sensorStore,W   ; Get the stored sensor states
    andlw   SENSORMASK      ; Isolate the two most recent states
    xorlw   SENSORL2H       ; Test for an off to on transition by the sensor

    btfss   STATUS,Z        ; Check outcome of test, skip if success ...
    goto    SensorNotL2H    ; ... else sensor not in correspondance

    incfsz  detAcc,W        ; Increment detection correspondance accumulator
    movwf   detAcc          ; If result is not zero update the accumulator
    return

SensorNotL2H

    decfsz  detAcc,W        ; Decrement detection correspondance accumulator
    movwf   detAcc          ; If result is not zero update the accumulator
    return

EmitterIsOff
    ; Toggle the present emitter state
    bcf     EMITTERPORT,EMITTERBIT  ; Turn emitter on

    ; Test for sensor transition from on to off,
    ; indicating presence of a reflective target

    movf    sensorStore,W   ; Get the stored sensor states
    andlw   SENSORMASK      ; Isolate the two most recent states
    xorlw   SENSORH2L       ; Test for a on to off transition by the sensor

    btfss   STATUS,Z        ; Check outcome of test, skip if success ...
    goto    SensorNotH2L    ; ... else sensor not in correspondance

    incfsz  detAcc,W        ; Increment detection correspondance accumulator
    movwf   detAcc          ; If result is not zero update the accumulator
    return

SensorNotH2L

    decfsz  detAcc,W        ; Decrement detection correspondance accumulator
    movwf   detAcc          ; If result is not zero update the accumulator
    return


;**********************************************************************
; User main loop code                                                 *
;**********************************************************************

UserMain    ; Top of main processing loop

    ; Check status of detection indicator

    btfsc   INDICATORPORT,INDICATORBIT  ; Test state of detection indicator ...
    goto    IndicatorIsOff              ; ... jump if off, else ...

    ; Detection indicator is currently on
    movf    detAcc,W        ; Test if detection correspondance accumulator ...
    sublw   DETLOWWTR       ; ... is above "Off" threshold
    btfss   STATUS,C        ; Skip if at or below threshold ...
    goto    EndDetection    ; ... else do nothing

    ; Detection correspondance has fallen to or below threshold
    bsf     INDICATORPORT,INDICATORBIT  ; Turn detection indicator off
    goto    EndDetection

IndicatorIsOff
    ; Detection indicator is currently off
    movf    detAcc,W        ; Test if detection correspondance accumulator ...
    sublw   low DETHIGHWTR  ; ... is above "On" threshold
    btfsc   STATUS,C        ; Skip if above threshold ...
    goto    EndDetection    ; ... else do nothing

    ; Detection correspondance has risen above threshold
    bcf     INDICATORPORT,INDICATORBIT  ; Turn detection indicator on

EndDetection

    return  ; End of main processing loop


;**********************************************************************
; End of source code
;**********************************************************************

    end     ; directive 'end of program'
